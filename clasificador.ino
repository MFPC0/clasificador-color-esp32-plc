/*
  Clasificador de color: ESP32-S3 N16R8, TCS34725, VL53L0X, 2 servos, 4 relevadores hacia un PLC.

  ARDUINO IDE
  - Placa "ESP32S3 Dev Module", Flash Size 16MB, PSRAM "OPI PSRAM", USB CDC On Boot "Enabled".
  - Librerías: Adafruit TCS34725, Adafruit VL53L0X, ESP32Servo, ESPAsyncWebServer y AsyncTCP (ESP32Async).

  PRIMERA PRUEBA
  1) Edita AP_SSID y AP_PASS en config.h. Carga el sketch y abre el monitor serial a 115200.
  2) Conéctate a esa red Wi-Fi y abre http://192.168.4.1
  3) Sin el PLC conectado: Calibración, "Usar PLC" = 0 (modo banco). Al terminar las pruebas, ponlo en 1.
  4) Enciende los servos con la charola vacía. En Manual calibra: límites del servo 1, reposo e inclinación
     del servo 2, charola vacía y los tres tonos (pieza bajo el sensor, "Capturar ...").
  5) Automático: Iniciar. Coloca una pieza en la charola.

  PLC (ladder): Q1=I1, Q2=I2, Q3=I3 (copias de verde, amarillo y rojo); I4 recibe la falla de la ESP32;
  el botón de paro (contacto NC) entra en I5 y Q4=I5 va al GPIO 21 (1 = normal, 0 = paro).
  Las señales del PLC regresan a la ESP32 con 3V3 a través del contacto. Nunca conectes 5 V ni 24 V a los pines.
*/
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_VL53L0X.h>
#include "config.h"
#include "web_ui.h"

// ---------- Parámetros editables desde la página (se guardan en memoria) ----------
enum { P_S1MIN, P_S1MAX, P_REJ, P_S2REP, P_S2UP, P_S2WIG, P_V1, P_V2, P_TASENT, P_TCAIDA, P_TECO, P_TSTAB,
       P_BASE, P_DELTA, P_TOL, P_SATMIN, P_HV, P_HA, P_HR, P_SECV, P_SECA, P_SECR, P_PLC, NP };
struct Param { const char* n; float v, lo, hi; };  // lo/hi: rango válido (el mismo que valida la página en PS)
Param P[NP] = {{"s1min", 20, 0, 180}, {"s1max", 160, 0, 180}, {"rej", 20, 0, 180}, {"s2rep", 10, 0, 180}, {"s2up", 45, 0, 180},
               {"s2wig", 6, 0, 20}, {"v1", 90, 10, 360}, {"v2", 120, 10, 360}, {"tasent", 500, 0, 5000}, {"tcaida", 1500, 100, 10000},
               {"teco", 1000, 100, 10000}, {"tstab", 800, 0, 5000}, {"base", 100, 20, 2000}, {"delta", 15, 2, 200}, {"tol", 25, 1, 90},
               {"satmin", 0.25, 0, 1}, {"hV", 125, 0, 359}, {"hA", 55, 0, 359}, {"hR", 0, 0, 359},
               {"secV", 0, 0, 2}, {"secA", 1, 0, 2}, {"secR", 2, 0, 2}, {"plc", 1, 0, 1}};
float DEF[NP];          // copia de los valores de fábrica de P[], para el comando "defaults"
#define PREFS_VER 1     // si la memoria guardada no tiene esta versión, se borra al arrancar

// ---------- Estados ----------
enum St { S_DETENIDO, S_MANUAL, S_ESPERA, S_ESTAB, S_LEER, S_MOVER, S_CONF, S_EXPULSA, S_CAIDA, S_VERIF, S_LIBERA, S_ERROR, S_PARO };
const char* NOM[] = {"DETENIDO", "MANUAL", "Espera de pieza", "Estabilizando", "Leyendo color", "Moviendo carrusel",
                     "Confirmando con el PLC", "Levantando charola", "Esperando caída", "Verificando", "Liberando el PLC", "ERROR", "PARO"};
const char* MC[] = {"Color verde", "Color amarillo", "Color rojo"};
const int NWIG = 3;

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_VL53L0X lox;
Servo sv1, sv2;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences prefs;

const int PIN_REL[4] = {PIN_REL_V, PIN_REL_A, PIN_REL_R, PIN_REL_FALLA};
const int PIN_RET[4] = {PIN_RET_V, PIN_RET_A, PIN_RET_R, PIN_RET_PARO};
const tcs34725Gain_t GAN[4] = {TCS34725_GAIN_1X, TCS34725_GAIN_4X, TCS34725_GAIN_16X, TCS34725_GAIN_60X};

struct Axis { Servo* sv; float cur, tgt; };
Axis ax1 = {&sv1, 20, 20}, ax2 = {&sv2, 10, 10};

St st = S_DETENIDO;
uint32_t tSt = 0, tMark = 0, okSince = 0, tPz = 0;
char msg[64] = "";
int colSel = -1, colLive = -1, wig = 0, reint = 0, gi = 1, okMask = 0, stFalla = -1;  // stFalla: estado donde ocurrió la falla
bool pz = false, relS[4] = {false, false, false, false};
float dist = 999, hue = 0, sat = 0;
uint16_t cR = 0, cG = 0, cB = 0, cC = 0;
uint32_t cnt[4] = {0, 0, 0, 0};
String cola[8];
volatile int qh = 0, qt = 0;
bool wifiOk = false;
uint32_t tReboot = 0;  // momento programado para reiniciar la tarjeta (0 = no)
struct LedUsr { uint8_t r, g, b, fx; bool on; };
LedUsr ledUsr = {0, 0, 0, 0, false};  // juego del modo manual; el indicador de estado no se configura
uint32_t ledCol = 0xFFFFFFFF;

// ---------- Utilidades ----------
// Ninguna función antes de este punto: Arduino inserta los prototipos antes de la primera y aún no existirían St ni Axis.
float pv(int i) { return P[i].v; }
void guardar(int i) { prefs.putFloat(P[i].n, P[i].v); }
void guardaCnt() { prefs.putBytes("cnt", cnt, sizeof(cnt)); }
void escribe(Axis& a) { a.sv->writeMicroseconds((int)(500 + a.cur * (2000.0f / 180.0f))); }
bool llego(const Axis& a) { return fabsf(a.cur - a.tgt) < 0.01f; }
void mueve(Axis& a, float t) { a.tgt = constrain(t, 0.0f, 180.0f); }
void paso(Axis& a, float vel, float dt) {
  float d = a.tgt - a.cur, m = vel * dt;
  if (fabsf(d) <= m) a.cur = a.tgt; else a.cur += (d > 0 ? m : -m);
  escribe(a);
}
void rele(int i, bool on) { relS[i] = on; digitalWrite(PIN_REL[i], on ? LOW : HIGH); }
void reles3Off() { for (int i = 0; i < 3; i++) rele(i, false); }
bool ret(int i) { return digitalRead(PIN_RET[i]) == HIGH; }
bool usaPLC() { return pv(P_PLC) > 0.5f; }
void setSt(St s, const char* m = "") {
  if (s != S_MANUAL) ledUsr.on = false;  // al salir de manual el LED vuelve a indicar el estado
  st = s; tSt = millis(); tMark = tSt; okSince = 0;
  strncpy(msg, m, sizeof(msg) - 1); msg[sizeof(msg) - 1] = 0;
}
void falla(St s, const char* m) {
  if (st < S_ERROR) stFalla = st;
  reles3Off(); rele(3, true);
  ax1.tgt = ax1.cur; ax2.tgt = ax2.cur;
  setSt(s, m);
}

// ---------- LED RGB de la tarjeta: indicador fijo de estado y fallas ----------
// Brillo máximo 40 de 255: el WS2812 deslumbra a pleno. Prioridad de arriba hacia abajo.
uint32_t rgb(uint32_t r, uint32_t g, uint32_t b) { return (r << 16) | (g << 8) | b; }
uint32_t escala(uint32_t c, float k) { return rgb(((c >> 16) & 255) * k, ((c >> 8) & 255) * k, (c & 255) * k); }
uint32_t arcoiris(uint32_t h) {
  uint32_t x = h % 120, up = x * 40 / 120, dn = 40 - up;
  return h < 120 ? rgb(dn, up, 0) : h < 240 ? rgb(0, dn, up) : rgb(up, 0, dn);
}
void ledEstado(uint32_t now) {
  const uint32_t V = rgb(0, 40, 0), A = rgb(40, 24, 0), R = rgb(40, 0, 0), AZ = rgb(0, 0, 40),
                 MAG = rgb(32, 0, 32), BL = rgb(26, 26, 26), CI = rgb(0, 30, 30), PZ[3] = {V, A, R};
  float resp = 0.12f + 0.88f * (0.5f - 0.5f * cosf(now * 0.00314f));  // respiración de 2 s
  bool p4 = (now / 125) & 1, p2 = (now / 250) & 1, p1 = (now / 500) & 1;
  uint32_t c;
  if (!wifiOk) c = p2 ? MAG : R;                       // no se pudo iniciar la red Wi-Fi
  else if (st == S_PARO) c = p4 ? R : 0;               // paro de emergencia
  else if (st == S_ERROR) c = p2 ? A : 0;              // falla del proceso
  else if (okMask != 3) c = p1 ? MAG : 0;              // falta un sensor I2C
  else if (st == S_MANUAL) {
    uint32_t u = rgb(ledUsr.r, ledUsr.g, ledUsr.b);
    c = !ledUsr.on ? CI : ledUsr.fx == 1 ? escala(u, resp) : ledUsr.fx == 2 ? (p2 ? u : 0) : ledUsr.fx == 3 ? arcoiris((now / 15) % 360) : u;
  }
  else if (st == S_DETENIDO) c = ws.count() ? AZ : escala(AZ, resp);  // servidor encendido: fijo con página, respira sin ella
  else if (st <= S_LEER) c = escala(V, resp);          // ciclo: esperando o leyendo pieza
  else c = colSel >= 0 ? PZ[colSel] : BL;              // pieza en camino: su color, o blanco si va al rechazo
  if (c != ledCol) { ledCol = c; rgbLedWrite(PIN_RGB, (c >> 16) & 255, (c >> 8) & 255, c & 255); }
}

// ---------- Sensores ----------
// VL53L0X en modo continuo: mide solo y el programa recoge el dato sin esperar (antes cada lectura bloqueaba ~33 ms).
uint32_t vlMs = 33, rawHasta = 0;  // tiempo por medición (más tiempo = menos ruido) y fin del volcado de datos crudos
void vlInicia(uint32_t ms) {
  vlMs = constrain(ms, 20, 200);
  lox.stopRangeContinuous();
  lox.setMeasurementTimingBudgetMicroSeconds(vlMs * 1000);
  lox.startRangeContinuous(vlMs);
}
// Filtro de Kalman 1D (modelo de paseo aleatorio). Comparado con datos reales de este sensor contra media móvil,
// mediana y wavelet Haar: el de menor ruido (0,4 mm contra 1,4 mm crudo a 33 ms) y sin retraso al entrar o salir una pieza.
// Compuerta de innovación: si la lectura se aleja más de KGATE sigmas, es un cambio real y la covarianza se abre para seguirlo.
const float KQ = 0.05f, KR = 2.0f, KGATE = 4.0f, KSALTO = 50.0f;  // KR = varianza medida del sensor a 33 ms (1,42² mm²)
struct Kal { float x, p; bool ok; } kd = {0, 0, false};
float kalman(float z) {
  if (!kd.ok) { kd = {z, KR, true}; return z; }
  kd.p += KQ;
  float nu = z - kd.x;
  if (nu * nu > KGATE * KGATE * (kd.p + KR)) kd.p += KSALTO;
  float k = kd.p / (kd.p + KR);
  kd.x += k * nu; kd.p *= 1 - k;
  return kd.x;
}
uint32_t tVL = 0;  // última lectura válida, para detectar un sensor de distancia caído
void leerDist() {
  if (!lox.isRangeComplete()) return;
  uint16_t r = lox.readRangeResult();  // 0xFFFF: fuera de rango o error de I2C
  if (r == 0xFFFF) { dist = 999; kd.ok = false; }
  else { dist = kalman(r); tVL = millis(); }
  if (rawHasta && (int32_t)(millis() - rawHasta) < 0) Serial.printf("R,%lu,%u,%.1f\n", (unsigned long)millis(), r, dist);
}
// Histéresis: la pieza "entra" bajo base-delta y "sale" sobre base-delta/2, así el ruido no la hace parpadear en el umbral.
bool pzDet = false;
bool hayPieza() {
  float u = pv(P_BASE) - pv(P_DELTA);
  pzDet = dist < (pzDet ? u + pv(P_DELTA) / 2 : u);
  return pzDet;
}

void ajustaGanancia() {
  for (int k = 0; k < 3; k++) {
    uint16_t R, G, B, C;
    tcs.getRawData(&R, &G, &B, &C);
    if (C > 18000 && gi > 0) gi--; else if (C < 1500 && gi < 3) gi++; else break;
    tcs.setGain(GAN[gi]); delay(130);
  }
}
void leerColor(int n) {
  float r = 0, g = 0, b = 0, c = 0;
  for (int i = 0; i < n; i++) {
    uint16_t R, G, B, C;
    tcs.getRawData(&R, &G, &B, &C);
    r += R; g += G; b += B; c += C;
    if (n > 1) delay(60);
  }
  float fr = r / n, fg = g / n, fb = b / n;
  cR = fr; cG = fg; cB = fb; cC = c / n;
  float mx = max(fr, max(fg, fb)), mn = min(fr, min(fg, fb)), d = mx - mn;
  sat = mx > 0 ? d / mx : 0;
  if (d <= 0) hue = 0;
  else if (mx == fr) hue = 60 * fmodf((fg - fb) / d + 6, 6);
  else if (mx == fg) hue = 60 * ((fb - fr) / d + 2);
  else hue = 60 * ((fr - fg) / d + 4);
  if (hue < 0) hue += 360;
}
int clasifica() {
  if (sat < pv(P_SATMIN)) return -1;
  int best = -1; float bd = 999;
  for (int c = 0; c < 3; c++) {
    float d = fabsf(hue - pv(P_HV + c));
    if (d > 180) d = 360 - d;
    if (d < bd) { bd = d; best = c; }
  }
  return bd <= pv(P_TOL) ? best : -1;
}
float angSector(int c) {
  float a = pv(P_S1MIN), b = pv(P_S1MAX);
  return a + (pv(P_SECV + c) + 0.5f) * (b - a) / 3.0f;
}

// ---------- Comunicación con la página ----------
String cfgJson() {
  String s = "{\"t\":\"c\",\"v\":{";
  for (int i = 0; i < NP; i++) { if (i) s += ','; s += '"'; s += P[i].n; s += "\":"; s += String(P[i].v, 2); }
  return s + "}}";
}
void enviaEstado() {
  char b[512];
  snprintf(b, sizeof b,
    "{\"t\":\"s\",\"si\":%d,\"fs\":%d,\"te\":%lu,\"st\":\"%s\",\"msg\":\"%s\",\"col\":%d,\"h\":%.0f,\"s\":%.2f,\"r\":%u,\"g\":%u,\"b\":%u,\"c\":%u,"
    "\"d\":%.0f,\"pz\":%d,\"rel\":[%d,%d,%d,%d],\"ret\":[%d,%d,%d,%d],\"a1\":%.0f,\"a2\":%.0f,"
    "\"n\":[%lu,%lu,%lu,%lu],\"ok\":%d,\"lc\":\"%06lX\"}",
    (int)st, stFalla, (unsigned long)(millis() - tSt), NOM[st], msg, colLive, hue, sat, (unsigned)cR, (unsigned)cG, (unsigned)cB, (unsigned)cC,
    dist, (int)hayPieza(), relS[0], relS[1], relS[2], relS[3], ret(0), ret(1), ret(2), ret(3), ax1.cur, ax2.cur,
    (unsigned long)cnt[0], (unsigned long)cnt[1], (unsigned long)cnt[2], (unsigned long)cnt[3], okMask, (unsigned long)(ledCol & 0xFFFFFF));
  ws.textAll(b);
}
void comando(const String& m) {
  int k = m.indexOf(':');
  String c = k < 0 ? m : m.substring(0, k), a = k < 0 ? "" : m.substring(k + 1);
  bool libre = (st == S_MANUAL), quieto = (st == S_MANUAL || st == S_DETENIDO);
  bool ajuste = quieto || st == S_ERROR || st == S_PARO;  // máquina parada: se puede calibrar (p. ej. apagar "Usar PLC" en un paro sin PLC)
  if (c == "auto") {
    if ((st == S_DETENIDO || st == S_MANUAL) && okMask == 3) { reles3Off(); rele(3, false); mueve(ax1, pv(P_REJ)); mueve(ax2, pv(P_S2REP)); reint = 0; setSt(S_ESPERA); }
  } else if (c == "stop") {  // también apaga el relé de falla que se haya encendido a mano en manual
    if (st != S_PARO && st != S_ERROR) { reles3Off(); rele(3, false); mueve(ax1, pv(P_REJ)); mueve(ax2, pv(P_S2REP)); setSt(S_DETENIDO); }
  } else if (c == "manual") {
    if (st == S_DETENIDO || st == S_ESPERA) { reles3Off(); setSt(S_MANUAL); }
  } else if (c == "reset") {
    if (st == S_PARO || st == S_ERROR) {
      if (!usaPLC() || ret(3)) { rele(3, false); reles3Off(); stFalla = -1; setSt(S_DETENIDO); }
      else strncpy(msg, "El botón de paro sigue activo", sizeof(msg) - 1);
    }
  } else if (c == "reboot" && (quieto || st == S_ERROR || st == S_PARO)) {  // nunca con el ciclo corriendo
    reles3Off(); rele(3, false);
    tReboot = millis() + 400;  // margen para que la página reciba el último estado
  } else if (c == "defaults" && quieto) {  // valores de fábrica: parámetros y contadores, y todo a reposo
    prefs.clear(); prefs.putUChar("ver", PREFS_VER);
    for (int i = 0; i < NP; i++) P[i].v = DEF[i];
    memset(cnt, 0, sizeof(cnt));
    reles3Off(); rele(3, false); mueve(ax1, pv(P_REJ)); mueve(ax2, pv(P_S2REP)); setSt(S_DETENIDO);
    ws.textAll(cfgJson());
  } else if (c == "led" && libre) {  // solo en manual: color y efecto (0 fijo, 1 respirar, 2 parpadeo, 3 arcoíris)
    int r, g, b, fx;
    if (a == "auto") ledUsr.on = false;
    else if (sscanf(a.c_str(), "%d,%d,%d,%d", &r, &g, &b, &fx) == 4)
      ledUsr = {(uint8_t)(constrain(r, 0, 255) * 40 / 255), (uint8_t)(constrain(g, 0, 255) * 40 / 255),
                (uint8_t)(constrain(b, 0, 255) * 40 / 255), (uint8_t)constrain(fx, 0, 3), true};
  } else if (c == "m1" && libre) mueve(ax1, a.toFloat());
  else if (c == "m2" && libre) mueve(ax2, a.toFloat());
  else if (c == "rele" && libre) {
    int i = a.toInt(), on = a.substring(a.indexOf(':') + 1).toInt();
    if (i >= 0 && i < 4) rele(i, on);
  } else if (c == "calvacia" && ajuste) {
    if (dist < 999) { P[P_BASE].v = constrain(dist, P[P_BASE].lo, P[P_BASE].hi); guardar(P_BASE); }  // 999 = lectura inválida
    ws.textAll(cfgJson());
  } else if (c == "calcolor" && ajuste) {
    int i = a.toInt();
    if (i >= 0 && i < 3) { leerColor(5); P[P_HV + i].v = constrain(hue, 0.0f, 359.0f); guardar(P_HV + i); ws.textAll(cfgJson()); }
  } else if (c == "set" && ajuste) {  // nunca con el ciclo corriendo
    int e = a.indexOf('=');
    if (e > 0) {
      String n = a.substring(0, e); float v = a.substring(e + 1).toFloat();
      for (int i = 0; i < NP; i++)
        if (n == P[i].n && v >= P[i].lo && v <= P[i].hi &&  // fuera de rango no se guarda: la consola USB no pasa por la validación de la página
            !(i == P_S1MIN && v >= pv(P_S1MAX)) && !(i == P_S1MAX && v <= pv(P_S1MIN))) { P[i].v = v; guardar(i); }
      ws.textAll(cfgJson());  // eco: la página confirma "Guardado" con esto (o ve que el valor no cambió)
    }
  }
}
void onWs(AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) c->text(cfgJson());
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo* i = (AwsFrameInfo*)arg;
    if (i->final && i->index == 0 && i->len == len && i->opcode == WS_TEXT) {
      String m; for (size_t k = 0; k < len; k++) m += (char)data[k];
      int n = (qh + 1) % 8;
      if (n != qt) { cola[qh] = m; qh = n; }
    }
  }
}

// ---------- Inicio ----------
void setup() {
  for (int i = 0; i < 4; i++) {            // relevadores apagados lo antes posible
    pinMode(PIN_REL[i], OUTPUT); digitalWrite(PIN_REL[i], HIGH);
    pinMode(PIN_RET[i], INPUT_PULLDOWN);
  }
  pinMode(PIN_LED_TCS, OUTPUT); digitalWrite(PIN_LED_TCS, HIGH);
  rgbLedWrite(PIN_RGB, 20, 20, 20);  // blanco tenue mientras arranca
  Serial.begin(115200);
  Serial.setTimeout(50);  // la consola USB no debe frenar el ciclo si llega una línea sin fin
  prefs.begin("clf", false);
  for (int i = 0; i < NP; i++) DEF[i] = P[i].v;
  if (prefs.getUChar("ver", 0) != PREFS_VER) { prefs.clear(); prefs.putUChar("ver", PREFS_VER); }  // descarta valores de prueba viejos
  for (int i = 0; i < NP; i++) P[i].v = constrain(prefs.getFloat(P[i].n, P[i].v), P[i].lo, P[i].hi);
  prefs.getBytes("cnt", cnt, sizeof(cnt));

  Wire.begin(PIN_SDA0, PIN_SCL0);
  Wire1.begin(PIN_SDA1, PIN_SCL1);
  if (tcs.begin(TCS34725_ADDRESS, &Wire)) { okMask |= 1; tcs.setGain(GAN[gi]); }
  if (lox.begin(VL53L0X_I2C_ADDR, false, &Wire1)) { okMask |= 2; vlInicia(vlMs); }
  Serial.printf("TCS34725: %s | VL53L0X: %s\n", (okMask & 1) ? "OK" : "NO ENCONTRADO", (okMask & 2) ? "OK" : "NO ENCONTRADO");

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  sv1.setPeriodHertz(50); sv2.setPeriodHertz(50);
  ax1.cur = ax1.tgt = pv(P_REJ); ax2.cur = ax2.tgt = pv(P_S2REP);
  sv1.attach(PIN_SERVO1, 500, 2500); sv2.attach(PIN_SERVO2, 500, 2500);
  escribe(ax1); escribe(ax2);

  WiFi.mode(WIFI_AP);
  wifiOk = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("Red: %s  IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  ws.onEvent(onWs);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) { r->send_P(200, "text/html; charset=utf-8", INDEX_HTML); });
  server.begin();
  if (okMask != 3) setSt(S_DETENIDO, "Falta un sensor I2C");
}

// ---------- Ciclo principal ----------
void loop() {
  uint32_t now = millis();
  static uint32_t tL = now, tD = 0, tW = 0, tC = 0;
  // Tope de 100 ms: leer el color bloquea hasta ~1 s y, sin tope, el servo saltaría de golpe en vez de seguir su rampa.
  float dt = min((now - tL) / 1000.0f, 0.1f); tL = now;

  while (qt != qh) { comando(cola[qt]); qt = (qt + 1) % 8; }
  if (Serial.available()) {  // consola USB: mismos comandos que la página; "?" solo consulta
    String l = Serial.readStringUntil('\n'); l.trim();
    if (l.length()) {
      bool quieto = (st == S_MANUAL || st == S_DETENIDO);
      if (l.startsWith("vl:") && quieto && (okMask & 2)) vlInicia(l.substring(3).toInt());     // diagnóstico: tiempo de medición
      else if (l.startsWith("raw:")) rawHasta = millis() + 1000 * l.substring(4).toInt();     // diagnóstico: volcar lecturas crudas N s
      else if (l != "?") comando(l);
      Serial.printf("%s | n=%lu,%lu,%lu,%lu | s1=%.0f s2=%.0f | rel=%d%d%d%d ret=%d%d%d%d ok=%d d=%.0f | %s\n", NOM[st],
                    (unsigned long)cnt[0], (unsigned long)cnt[1], (unsigned long)cnt[2], (unsigned long)cnt[3], ax1.tgt, ax2.tgt,
                    relS[0], relS[1], relS[2], relS[3], ret(0), ret(1), ret(2), ret(3), okMask, dist, msg);
      Serial.println(cfgJson());
    }
  }
  if (now - tD >= 5) { tD = now; if (okMask & 2) leerDist(); }
  if (usaPLC() && !ret(3) && st != S_PARO) falla(S_PARO, "Paro de emergencia activo");
  // Vigilancia en marcha: sin lecturas válidas del sensor de distancia en 1,5 s, se detiene (antes contaba piezas fantasma).
  if (st >= S_ESPERA && st <= S_LIBERA && now - tVL > 1500) falla(S_ERROR, "Sin lectura del sensor de distancia");
  paso(ax1, pv(P_V1), dt); paso(ax2, pv(P_V2), dt);

  switch (st) {
    case S_ESPERA:
      if (llego(ax1) && llego(ax2) && hayPieza()) {
        if (!pz) { pz = true; tPz = now; }
        else if (now - tPz >= pv(P_TSTAB)) { pz = false; setSt(S_LEER); }
      } else pz = false;
      break;

    case S_LEER:
      ajustaGanancia(); leerColor(5);
      if (cC == 0 || cC == 0xFFFF) { falla(S_ERROR, "Sin lectura del sensor de color"); break; }  // 0 o 0xFFFF: I2C caído
      colSel = clasifica(); colLive = colSel;
      mueve(ax1, colSel >= 0 ? angSector(colSel) : pv(P_REJ));
      setSt(S_MOVER, colSel >= 0 ? MC[colSel] : "Color no reconocido");
      break;

    case S_MOVER:
      if (!llego(ax1)) tMark = now;
      else if (now - tMark >= pv(P_TASENT)) {
        if (colSel >= 0) rele(colSel, true);
        setSt(S_CONF, "Esperando el retorno del PLC");
      }
      break;

    case S_CONF: {
      bool ok = true;
      if (usaPLC()) for (int i = 0; i < 3; i++) if (ret(i) != (i == colSel)) ok = false;
      if (ok) {
        if (!okSince) okSince = now;
        if (now - okSince >= 150) {
          mueve(ax2, pv(P_S2UP)); wig = 0;
          setSt(S_EXPULSA, colSel >= 0 ? "Expulsando la pieza al embudo" : "Rechazando la pieza");
        }
      } else {
        okSince = 0;
        if (now - tSt >= pv(P_TECO)) falla(S_ERROR, "Error de comunicación con el PLC");
      }
      break;
    }

    case S_EXPULSA:
      if (llego(ax2)) {
        if (wig < 2 * NWIG && pv(P_S2WIG) > 0) { mueve(ax2, pv(P_S2UP) + ((wig % 2) ? -1 : 1) * pv(P_S2WIG)); wig++; }
        else setSt(S_CAIDA, "Esperando la caída");
      }
      break;

    case S_CAIDA:
      if (now - tSt >= pv(P_TCAIDA)) { mueve(ax2, pv(P_S2REP)); setSt(S_VERIF, "Verificando la charola"); }
      break;

    case S_VERIF:
      if (!llego(ax2)) tMark = now;
      else if (now - tMark >= 500) {
        if (hayPieza()) {
          if (reint < 1) { reint++; mueve(ax2, pv(P_S2UP)); wig = 0; setSt(S_EXPULSA, "Reintentando la expulsión"); }
          else falla(S_ERROR, "Pieza atorada en la charola");
        } else {
          cnt[colSel >= 0 ? colSel : 3]++; guardaCnt(); reint = 0;
          reles3Off(); setSt(S_LIBERA, "Liberando las señales del PLC");
        }
      }
      break;

    case S_LIBERA: {
      bool ok = true;
      if (usaPLC()) for (int i = 0; i < 3; i++) if (ret(i)) ok = false;
      if (ok) { mueve(ax1, pv(P_REJ)); if (llego(ax1)) setSt(S_ESPERA); }
      else if (now - tSt >= pv(P_TECO)) falla(S_ERROR, "El PLC no liberó las señales");
      break;
    }
    default: break;
  }

  if ((st == S_MANUAL || st == S_DETENIDO) && (okMask & 1) && now - tC >= 400) { tC = now; leerColor(1); colLive = clasifica(); }
  if (now - tW >= 200) { tW = now; enviaEstado(); }
  if (tReboot && (int32_t)(now - tReboot) >= 0) { rgbLedWrite(PIN_RGB, 20, 20, 20); ESP.restart(); }
  static uint32_t tLed = 0;
  if (now - tLed >= 20) { tLed = now; ledEstado(now); }
  ws.cleanupClients();
}
