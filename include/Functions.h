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

/**
 * @brief Configura e instala el driver I2C maestro en I2C_PORT (pines/frecuencia
 * definidos en Globals.h).
 */
void i2c_init(void);

/**
 * @brief Escanea las direcciones I2C 0x08-0x77 (probe de solo direccion) y loguea
 * los dispositivos encontrados. Solo para diagnostico de wiring por consola.
 */
void i2c_escanear_bus(void);

// ============================================================
// ADXL345
// ============================================================

/**
 * @brief Lee el registro DEVID del ADXL345 y lo compara contra el valor esperado
 * (0xE5), distinguiendo "no responde" de "responde pero no es un ADXL345".
 * @return true si el DEVID coincide con el esperado.
 */
bool adxl345_verificar_id(void);

/**
 * @brief Configura el ADXL345 en modo full-res +-4g y arranca la medicion continua.
 * @return ESP_OK si ambas escrituras I2C (DATA_FORMAT, POWER_CTL) tuvieron exito.
 */
esp_err_t adxl345_init(void);

// ============================================================
// MLX90614
// ============================================================

/**
 * @brief Verifica presencia del MLX90614 (ACK en el bus) y que la temperatura
 * ambiente leida caiga dentro del rango de operacion del sensor (-40..125 C),
 * para descartar datos basura por mal cableado.
 * @return true si el sensor responde y la lectura es plausible.
 */
bool mlx90614_verificar(void);

// ============================================================
// TWAI (CAN)
// ============================================================

/**
 * @brief Instala y arranca el driver TWAI en modo normal, con filtro que acepta
 * todos los IDs. Bitrate configurado en Functions.c (100 kbit/s por defecto).
 */
void twai_init_bus(void);

/**
 * @brief Chequea el estado del bus TWAI y, si esta en BUS-OFF, dispara la
 * recuperacion automatica y reinicia el driver. Pensado para llamarse
 * periodicamente (cada ~2 s) desde el loop principal.
 */
void twai_chequear_bus(void);

/**
 * @brief Revisa la cola de recepcion TWAI sin bloquear (timeout 0) y, si hay un
 * frame de CAN_ID_COMANDO pendiente, lo procesa (inicia/finaliza calibracion).
 * Llamar en loop hasta que devuelva false para drenar todos los comandos
 * pendientes en una misma iteracion.
 * @return true si habia un frame para procesar (de cualquier ID), false si la
 * cola de recepcion estaba vacia.
 */
bool twai_recibir_comando(void);

// ============================================================
// LED WS2812 (RMT driver)
// ============================================================

/**
 * @brief Inicializa el driver led_strip (RMT) para el WS2812 en LED_PIN y lo apaga.
 */
void ws2812_init(void);

/**
 * @brief Fija el color del LED WS2812 y aplica el cambio (refresh) de inmediato.
 * @param red Componente rojo (0-255).
 * @param green Componente verde (0-255).
 * @param blue Componente azul (0-255).
 */
void ws2812_set_color(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Recorre el espectro de colores de forma bloqueante, usado como
 * secuencia de arranque.
 * @param steps Cantidad de pasos/colores del recorrido.
 * @param delay_ticks Espera entre pasos (ticks de FreeRTOS).
 * @param brillo Brillo maximo aplicado a cada componente (0-255).
 */
void ws2812_rainbow(uint8_t steps, TickType_t delay_ticks, uint8_t brillo);

// ============================================================
// Envio de datos (armado de tramas CAN)
// ============================================================

/**
 * @brief Lee el ADXL345, aplica offset/gain de calibracion y calcula pitch/roll,
 * y los transmite en CAN_ID_INCLINACION. No hace nada (ni transmite) si todavia
 * no hay una calibracion valida cargada (ver CalibracionValida en Globals.h).
 */
void enviar_inclinacion(void);

/**
 * @brief Lee temperatura de objeto y ambiente del MLX90614 y las transmite en
 * CAN_ID_TEMPERATURA. No depende de la calibracion del acelerometro.
 */
void enviar_temperatura(void);

/**
 * @brief Transmite los extremos min/max de calibracion acumulados hasta el
 * momento, en dos frames separados (CAN_ID_CALIBRACION_MIN/MAX) porque los 6
 * valores de 16 bits no entran en un solo frame CAN clasico.
 * @param min_x Minimo crudo observado en X. @param max_x Maximo crudo en X.
 * @param min_y Minimo crudo observado en Y. @param max_y Maximo crudo en Y.
 * @param min_z Minimo crudo observado en Z. @param max_z Maximo crudo en Z.
 */
void enviar_calibracion(int min_x, int max_x, int min_y, int max_y, int min_z, int max_z);

// ============================================================
// Calibracion del acelerometro (disparada por comando CAN del Nano)
// ============================================================

/**
 * @brief Arranca una calibracion: reinicia los extremos min/max acumulados y
 * habilita el envio periodico de datos crudos (calibracion_paso). Suspende el
 * envio normal de inclinacion/temperatura hasta calibracion_finalizar().
 */
void calibracion_iniciar(void);

/**
 * @brief Lee el acelerometro, actualiza los extremos min/max observados por eje
 * y los retransmite (CAN + consola USB). Debe llamarse periodicamente
 * (~200 ms) mientras calibracion_en_curso() sea true.
 */
void calibracion_paso(void);

/**
 * @brief Corta la calibracion en curso, calcula offset/gain por eje a partir de
 * los extremos acumulados y los valida contra un rango de ganancia plausible.
 * Si algun eje queda fuera de rango, rechaza la calibracion completa (no
 * persiste ni pisa una calibracion valida anterior); si es aceptada, la
 * persiste en NVS y la deja activa de inmediato.
 */
void calibracion_finalizar(void);

/**
 * @brief Indica si hay una calibracion en curso (entre calibracion_iniciar() y
 * calibracion_finalizar()).
 * @return true si esta en curso.
 */
bool calibracion_en_curso(void);

/**
 * @brief Persiste offset/gain de calibracion en NVS (namespace "calib").
 * @param offset_x Offset eje X (cuentas crudas). @param offset_y Offset eje Y.
 * @param offset_z Offset eje Z. @param gain_x Ganancia eje X (cuentas por g).
 * @param gain_y Ganancia eje Y. @param gain_z Ganancia eje Z.
 */
void guardar_calibracion(int offset_x, int offset_y, int offset_z, int gain_x, int gain_y, int gain_z);

/**
 * @brief Recupera offset/gain de calibracion previamente guardados en NVS.
 * @param[out] offset_x Offset eje X. @param[out] offset_y Offset eje Y.
 * @param[out] offset_z Offset eje Z. @param[out] gain_x Ganancia eje X.
 * @param[out] gain_y Ganancia eje Y. @param[out] gain_z Ganancia eje Z.
 * @return ESP_OK si habia una calibracion guardada; ESP_ERR_NVS_NOT_FOUND si el
 * equipo nunca fue calibrado (namespace/clave inexistente).
 */
esp_err_t leer_calibracion(int *offset_x, int *offset_y, int *offset_z, int *gain_x, int *gain_y, int *gain_z);

#endif // FUNCTIONS_H
