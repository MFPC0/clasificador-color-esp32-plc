#pragma once

// Red Wi-Fi propia de la tarjeta. Cambia ambos valores (la clave debe tener 8 o más caracteres).
#define AP_SSID "Clasif-Color-7Q4"
#define AP_PASS "cambia-esta-clave"

// I2C: bus 0 para el TCS34725 y bus 1 para el VL53L0X (evita el conflicto de dirección 0x29)
#define PIN_SDA0 8
#define PIN_SCL0 9
#define PIN_SDA1 11
#define PIN_SCL1 12

// Servos (señal)
#define PIN_SERVO1 4    // carrusel
#define PIN_SERVO2 16   // charola

// Módulo de relevadores (activo en bajo): hacia las entradas I1 a I4 del PLC
#define PIN_REL_V 5
#define PIN_REL_A 6
#define PIN_REL_R 7
#define PIN_REL_FALLA 15

// Retornos del PLC: 3V3 -> contacto -> pin (con 330 ohm en serie). 1 = contacto cerrado
#define PIN_RET_V 1
#define PIN_RET_A 2
#define PIN_RET_R 14
#define PIN_RET_PARO 21   // 1 = normal, 0 = paro (botón normalmente cerrado)

// LED blanco del TCS34725 (pin LED del módulo). Si no lo conectas, no pasa nada.
#define PIN_LED_TCS 13

// LED RGB integrado de la tarjeta (WS2812). En algunas placas hay que puentear el pad "RGB" para que funcione.
#define PIN_RGB 48
