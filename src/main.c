/*
 * ============================================================================
 *  NODO REMOTO - ESP32-C6 (ESP-IDF puro, para PlatformIO framework=espidf)
 *  Lee inclinacion (ADXL345) y temperatura (MLX90614) por I2C
 *  y los transmite por CAN (TWAI nativo) al Arduino Nano base,
 *  compartiendo el mismo bus fisico que ya usa el colimador.
 *
 *  La implementacion de I2C/ADXL345/MLX90614/TWAI/LED vive en Functions.c;
 *  las firmas publicas estan declaradas en Functions.h.
 * ============================================================================
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/usb_serial_jtag.h"
#include "Functions.h"
#include "Globals.h"

static const char *TAG = "NODO_REMOTO";

#define PERIODO_INCLINACION_MS  1000   // 1 Hz
#define PERIODO_TEMPERATURA_MS  2000   // 0.5 Hz
#define PERIODO_CALIBRACION_MS  200    // 5 Hz, solo mientras dura la calibracion

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

    // Driver USB Serial/JTAG para sondear el teclado del monitor serie de
    // forma no bloqueante. No usamos fgetc(stdin): con el VFS por defecto
    // (sin este driver) la primera lectura sin datos deja el stream stdio
    // en estado de error y no vuelve a intentar leer nunca mas, aun
    // llamando clearerr(). usb_serial_jtag_read_bytes() con timeout 0 no
    // tiene ese problema.
    usb_serial_jtag_driver_config_t usj_conf = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usj_conf));

    printf("\n=== Calibracion manual del acelerometro (por USB) ===\n");
    printf("  s + ENTER : iniciar calibracion\n");
    printf("  f + ENTER : finalizar y guardar\n");
    printf("  Mover el sensor barriendo lentamente los 3 ejes (+-1g)\n");
    printf("=======================================================\n\n");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    TickType_t t_ultima_inclinacion  = xTaskGetTickCount();
    TickType_t t_ultima_temperatura  = xTaskGetTickCount();
    TickType_t t_ultimo_chequeo_bus  = xTaskGetTickCount();
    TickType_t t_ultima_calibracion  = xTaskGetTickCount();

    // Recuperar valores de calibracion guardados en NVS (si existen).
    // Si nunca se calibro (primera instalacion), CalibracionValida
    // queda en false y enviar_inclinacion() no manda pitch/roll.
    esp_err_t err_calib = leer_calibracion(&offsetX, &offsetY, &offsetZ, &gainX, &gainY, &gainZ);
    if (err_calib == ESP_OK) {
        CalibracionValida = true;
        ESP_LOGI(TAG, "Calibracion cargada desde NVS: offsetX=%d, offsetY=%d, offsetZ=%d, gainX=%d, gainY=%d, gainZ=%d",
                 offsetX, offsetY, offsetZ, gainX, gainY, gainZ);
    } else {
        ESP_LOGW(TAG, "No hay calibracion guardada (%s). Enviar comando de calibracion desde el Nano antes de operar.",
                 esp_err_to_name(err_calib));
    }

    while (1) {
        TickType_t ahora = xTaskGetTickCount();
        // Leer comandos CAN enviados por el Nano y procesarlos
        while (twai_recibir_comando()) {
            // drena todos los frames de comando pendientes en la cola RX
        }

        // Disparo manual de calibracion por teclado (USB Serial/JTAG),
        // para poder calibrar en fabrica sin necesidad del bus CAN.
        uint8_t tecla;
        if (usb_serial_jtag_read_bytes(&tecla, 1, 0) > 0) {
            if (tecla == 's' || tecla == 'S') {
                calibracion_iniciar();
            } else if (tecla == 'f' || tecla == 'F') {
                calibracion_finalizar();
            }
        }

        if (!calibracion_en_curso() &&
            (ahora - t_ultima_inclinacion) >= pdMS_TO_TICKS(PERIODO_INCLINACION_MS)) {
            t_ultima_inclinacion = ahora;
            enviar_inclinacion();
        }

        if (!calibracion_en_curso() &&
            (ahora - t_ultima_temperatura) >= pdMS_TO_TICKS(PERIODO_TEMPERATURA_MS)) {
            t_ultima_temperatura = ahora;
            enviar_temperatura();
        }

        if ((ahora - t_ultimo_chequeo_bus) >= pdMS_TO_TICKS(2000)) {
            t_ultimo_chequeo_bus = ahora;
            twai_chequear_bus();
        }

        if (calibracion_en_curso() &&
            (ahora - t_ultima_calibracion) >= pdMS_TO_TICKS(PERIODO_CALIBRACION_MS)) {
            t_ultima_calibracion = ahora;
            calibracion_paso();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
