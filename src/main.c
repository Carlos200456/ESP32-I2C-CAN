/*
 * ============================================================================
 *  NODO REMOTO - ESP32-C6 (ESP-IDF puro, para PlatformIO framework=espidf)
 *  Lee inclinacion (ADXL345) y temperatura (MLX90614) por I2C
 *  y los transmite por CAN (TWAI nativo) al Arduino Nano base,
 *  compartiendo el mismo bus fisico que ya usa el colimador.
 *
 *  *** REVISAR/AJUSTAR ANTES DE COMPILAR ***
 *   1) Pines I2C y TWAI: confirmar contra el pinout de tu placa.
 *   2) Bitrate CAN: debe ser IGUAL al que ya usa el bus del colimador.
 *   3) IDs CAN: no deben colisionar con los que ya usa el colimador.
 *   4) Mapeo de ejes del ADXL345: pitch/roll dependen de como quede
 *      montado fisicamente el sensor sobre el tubo - calibrar en campo.
 *   5) Este codigo usa el driver I2C "legacy" de ESP-IDF (driver/i2c.h).
 *      Si tu version de ESP-IDF (segun la plataforma espressif32 que baje
 *      PlatformIO) ya no lo soporta, avisame el error de compilacion y lo
 *      migro a la API nueva (driver/i2c_master.h).
 *   6) La lectura del MLX90614 NO valida el byte PEC (checksum SMBus).
 *      Funciona para uso normal, pero no tiene esa capa extra de
 *      verificacion de integridad.
 *
 *  La implementacion de I2C/ADXL345/MLX90614/TWAI/LED vive en Functions.c;
 *  las firmas publicas estan declaradas en Functions.h.
 * ============================================================================
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "Functions.h"

static const char *TAG = "NODO_REMOTO";

#define PERIODO_INCLINACION_MS  1000   // 1 Hz
#define PERIODO_TEMPERATURA_MS  2000   // 0.5 Hz

// ============================================================
// app_main
// ============================================================

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando nodo remoto...");
    ws2812_init();
    ws2812_rainbow(64, pdMS_TO_TICKS(50), 0x10);
    ws2812_set_color(0x00, 0x02, 0x00); // estado inicial: verde tenue

    i2c_init();
    i2c_escanear_bus();

    if (adxl345_verificar_id() && adxl345_init() != ESP_OK) {
        ESP_LOGE(TAG, "ADXL345 detectado pero fallo la configuracion inicial.");
    }
    mlx90614_verificar();

    twai_init_bus();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    TickType_t t_ultima_inclinacion  = xTaskGetTickCount();
    TickType_t t_ultima_temperatura  = xTaskGetTickCount();
    TickType_t t_ultimo_chequeo_bus  = xTaskGetTickCount();

    while (1) {
        TickType_t ahora = xTaskGetTickCount();

        if ((ahora - t_ultima_inclinacion) >= pdMS_TO_TICKS(PERIODO_INCLINACION_MS)) {
            t_ultima_inclinacion = ahora;
            enviar_inclinacion();
        }

        if ((ahora - t_ultima_temperatura) >= pdMS_TO_TICKS(PERIODO_TEMPERATURA_MS)) {
            t_ultima_temperatura = ahora;
            enviar_temperatura();
        }

        if ((ahora - t_ultimo_chequeo_bus) >= pdMS_TO_TICKS(2000)) {
            t_ultimo_chequeo_bus = ahora;
            twai_chequear_bus();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
