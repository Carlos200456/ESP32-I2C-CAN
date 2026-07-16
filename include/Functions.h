#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// I2C
// ============================================================
void i2c_init(void);
void i2c_escanear_bus(void);

// ============================================================
// ADXL345
// ============================================================
bool adxl345_verificar_id(void);
esp_err_t adxl345_init(void);

// ============================================================
// MLX90614
// ============================================================
bool mlx90614_verificar(void);

// ============================================================
// TWAI (CAN)
// ============================================================
void twai_init_bus(void);
void twai_chequear_bus(void);
bool twai_recibir_comando(void);

// ============================================================
// LED WS2812 (RMT driver)
// ============================================================
void ws2812_init(void);
void ws2812_set_color(uint8_t red, uint8_t green, uint8_t blue);
void ws2812_rainbow(uint8_t steps, TickType_t delay_ticks, uint8_t brillo);

// ============================================================
// Envio de datos (armado de tramas CAN)
// ============================================================
void enviar_inclinacion(void);
void enviar_temperatura(void);

void enviar_calibracion(int min_x, int max_x, int min_y, int max_y, int min_z, int max_z);

// ============================================================
// Calibracion del acelerometro (disparada por comando CAN del Nano)
// ============================================================
void calibracion_iniciar(void);
void calibracion_paso(void);
void calibracion_finalizar(void);
bool calibracion_en_curso(void);
void guardar_calibracion(int offset_x, int offset_y, int offset_z, int gain_x, int gain_y, int gain_z);
esp_err_t leer_calibracion(int *offset_x, int *offset_y, int *offset_z, int *gain_x, int *gain_y, int *gain_z);

#endif // FUNCTIONS_H
