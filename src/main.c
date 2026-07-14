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
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "led_strip.h"
#define CONFIG_TWAI_SUPPRESS_DEPRECATE_WARN 1
#include "driver/twai.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "NODO_REMOTO";

// ---------------- Configuracion I2C (AJUSTAR segun tu placa) ----------------
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     2
#define I2C_SCL_PIN     3
#define I2C_FREQ_HZ     100000

// ---------------- Direcciones I2C de los sensores ----------------
#define ADXL345_ADDR    0x53   // 0x1D si el pin SDO esta a VCC
#define MLX90614_ADDR   0x5A

// Registros ADXL345
#define ADXL345_REG_DEVID       0x00
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0      0x32
#define ADXL345_DEVID_ESPERADO  0xE5

// Registros MLX90614 (RAM)
#define MLX90614_REG_TA     0x06
#define MLX90614_REG_TOBJ1  0x07

// ---------------- Configuracion TWAI/CAN (AJUSTAR) ----------------
#define CAN_TX_PIN   GPIO_NUM_4
#define CAN_RX_PIN   GPIO_NUM_5
#define CAN_ID_INCLINACION  0x200
#define CAN_ID_TEMPERATURA  0x201

#define LED_PIN GPIO_NUM_8
#define PERIODO_INCLINACION_MS  1000   // 1 Hz
#define PERIODO_TEMPERATURA_MS  2000   // 0.5 Hz

static led_strip_handle_t led_strip;

static uint8_t seq_inclinacion = 0;
static uint8_t seq_temperatura = 0;

// ============================================================
// I2C - funciones de bajo nivel
// ============================================================

static esp_err_t i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(I2C_PORT, dev_addr, buf, sizeof(buf),
                                       pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_PORT, dev_addr, &reg, 1, data, len,
                                         pdMS_TO_TICKS(100));
}

static void i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
}

// ============================================================
// I2C - diagnostico de cableado (debug por USB Serial/JTAG)
// ============================================================

// Transaccion de solo direccion (sin datos): si el dispositivo ACKea,
// esta presente y bien cableado en esa direccion.
static esp_err_t i2c_probe(uint8_t dev_addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

static void i2c_escanear_bus(void) {
    ESP_LOGI(TAG, "Escaneando bus I2C (SDA=%d, SCL=%d)...", I2C_SDA_PIN, I2C_SCL_PIN);
    uint8_t encontrados = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_probe(addr) == ESP_OK) {
            ESP_LOGI(TAG, "  -> dispositivo I2C en 0x%02X", addr);
            encontrados++;
        }
    }
    if (encontrados == 0) {
        ESP_LOGE(TAG, "No se encontro ningun dispositivo I2C. Revisar SDA/SCL, pull-ups y alimentacion de los sensores.");
    } else {
        ESP_LOGI(TAG, "Escaneo I2C completo: %d dispositivo(s) encontrado(s).", encontrados);
    }
}

// ============================================================
// ADXL345
// ============================================================

// Lee el registro DEVID (siempre disponible, incluso sin configurar el
// sensor) y lo compara contra el valor fijo que documenta Analog Devices.
// Distingue "no hay nada en esa direccion" de "hay algo pero no es un
// ADXL345" (direccion pisada por otro dispositivo, wiring cruzado, etc).
static bool adxl345_verificar_id(void) {
    uint8_t devid = 0;
    esp_err_t err = i2c_read_regs(ADXL345_ADDR, ADXL345_REG_DEVID, &devid, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADXL345 (0x%02X): sin respuesta I2C (%s). Revisar SDA/SCL/VCC/GND.",
                 ADXL345_ADDR, esp_err_to_name(err));
        return false;
    }
    if (devid != ADXL345_DEVID_ESPERADO) {
        ESP_LOGE(TAG, "ADXL345 (0x%02X): DEVID=0x%02X, se esperaba 0x%02X. Revisar direccion (SDO) y wiring.",
                 ADXL345_ADDR, devid, ADXL345_DEVID_ESPERADO);
        return false;
    }
    ESP_LOGI(TAG, "ADXL345 (0x%02X): OK, DEVID=0x%02X", ADXL345_ADDR, devid);
    return true;
}

static esp_err_t adxl345_init(void) {
    esp_err_t err;
    // DATA_FORMAT: full-res (bit3=0x08) + rango +-4g (bits1:0=0x01)
    err = i2c_write_reg(ADXL345_ADDR, ADXL345_REG_DATA_FORMAT, 0x08 | 0x01);
    if (err != ESP_OK) return err;
    // POWER_CTL: bit Measure = 0x08 (arranca la medicion)
    err = i2c_write_reg(ADXL345_ADDR, ADXL345_REG_POWER_CTL, 0x08);
    return err;
}

static esp_err_t adxl345_leer_g(float *ax, float *ay, float *az) {
    uint8_t data[6];
    esp_err_t err = i2c_read_regs(ADXL345_ADDR, ADXL345_REG_DATAX0, data, sizeof(data));
    if (err != ESP_OK) return err;

    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    const float MG2G = 0.004f; // 4 mg/LSB en modo full-res
    *ax = raw_x * MG2G;
    *ay = raw_y * MG2G;
    *az = raw_z * MG2G;
    return ESP_OK;
}

// ============================================================
// MLX90614 (lectura simplificada, sin validar PEC/CRC del SMBus)
// ============================================================

static esp_err_t mlx90614_leer_temp(uint8_t reg, float *temp_c) {
    uint8_t data[3]; // LSB, MSB, PEC (PEC se lee pero no se valida)
    esp_err_t err = i2c_read_regs(MLX90614_ADDR, reg, data, sizeof(data));
    if (err != ESP_OK) return err;

    uint16_t raw = (data[1] << 8) | data[0];
    *temp_c = (raw * 0.02f) - 273.15f;
    return ESP_OK;
}

// El MLX90614 no tiene un registro DEVID simple: se valida presencia
// (ACK a la direccion) y que la temperatura ambiente devuelta caiga
// dentro del rango util del sensor, para descartar datos basura por
// mal cableado o SDA/SCL en corto.
static bool mlx90614_verificar(void) {
    if (i2c_probe(MLX90614_ADDR) != ESP_OK) {
        ESP_LOGE(TAG, "MLX90614 (0x%02X): sin respuesta I2C. Revisar SDA/SCL/VCC/GND.", MLX90614_ADDR);
        return false;
    }
    float t_amb;
    esp_err_t err = mlx90614_leer_temp(MLX90614_REG_TA, &t_amb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MLX90614 (0x%02X): responde en el bus pero fallo la lectura (%s).",
                 MLX90614_ADDR, esp_err_to_name(err));
        return false;
    }
    if (t_amb < -40.0f || t_amb > 125.0f) { // rango de operacion del sensor
        ESP_LOGE(TAG, "MLX90614 (0x%02X): temperatura ambiente fuera de rango (%.2f C). Revisar wiring/alimentacion.",
                 MLX90614_ADDR, t_amb);
        return false;
    }
    ESP_LOGI(TAG, "MLX90614 (0x%02X): OK, Tamb=%.2f C", MLX90614_ADDR, t_amb);
    return true;
}

// ============================================================
// TWAI (CAN)
// ============================================================

static void twai_init_bus(void) {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    // Elegir la linea que coincida con el bitrate del bus existente:
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
    // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI iniciado correctamente");
}

static void twai_chequear_bus(void) {
    twai_status_info_t estado;
    twai_get_status_info(&estado);
    if (estado.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGW(TAG, "Bus CAN en BUS-OFF, recuperando...");
        twai_initiate_recovery();
        vTaskDelay(pdMS_TO_TICKS(100));
        twai_start();
    }
}

static bool twai_enviar_frame(uint32_t id, const uint8_t *datos, uint8_t largo) {
    twai_message_t msg = {0};
    msg.identifier = id;
    msg.extd = 0; // ID estandar de 11 bits
    msg.data_length_code = largo;
    memcpy(msg.data, datos, largo);
    return twai_transmit(&msg, pdMS_TO_TICKS(50)) == ESP_OK;
}

// ============================================================
// LED WS2812 (RMT driver)
// ============================================================
static void ws2812_init(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_PIN,
        .max_leds = 1,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
        .led_model = LED_MODEL_WS2812,
        .flags = {
            .invert_out = 0,
        },
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,
        .flags = {
            .with_dma = 0,
        },
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    ESP_LOGI(TAG, "WS2812 initialized using led_strip (RMT)");
}

static void ws2812_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, red, green, blue));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

static void ws2812_rainbow(uint8_t steps, TickType_t delay_ticks, uint8_t brillo) {
    for (uint8_t i = 0; i < steps; ++i) {
        uint8_t hue = (uint8_t)((uint16_t)i * 255U / steps);
        uint8_t red, green, blue;

        if (hue < 85) {
            red = 255 - (hue * 3);
            green = hue * 3;
            blue = 0;
        } else if (hue < 170) {
            hue -= 85;
            red = 0;
            green = 255 - (hue * 3);
            blue = hue * 3;
        } else {
            hue -= 170;
            red = hue * 3;
            green = 0;
            blue = 255 - (hue * 3);
        }

        ws2812_set_color((uint8_t)((red * brillo) / 255),
                          (uint8_t)((green * brillo) / 255),
                          (uint8_t)((blue * brillo) / 255));
        vTaskDelay(delay_ticks);
    }
}


// ============================================================
// Envio de datos (armado de tramas)
// ============================================================

static void enviar_inclinacion(void) {
    float ax, ay, az;
    if (adxl345_leer_g(&ax, &ay, &az) != ESP_OK) {
        ESP_LOGW(TAG, "Error leyendo ADXL345");
        return;
    }

    // Calibrar mapeo de ejes segun montaje fisico real sobre el tubo
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / (float)M_PI;
    float roll  = atan2f(ay, az) * 180.0f / (float)M_PI;

    int16_t pitch_centi = (int16_t)(pitch * 100.0f); // centesimas de grado
    int16_t roll_centi  = (int16_t)(roll * 100.0f);

    uint8_t datos[5];
    datos[0] = pitch_centi & 0xFF;
    datos[1] = (pitch_centi >> 8) & 0xFF;
    datos[2] = roll_centi & 0xFF;
    datos[3] = (roll_centi >> 8) & 0xFF;
    datos[4] = seq_inclinacion++; // contador para detectar datos viejos/perdidos

    if (!twai_enviar_frame(CAN_ID_INCLINACION, datos, sizeof(datos))) {
        ESP_LOGW(TAG, "Fallo al enviar frame de inclinacion");
    }
}

static void enviar_temperatura(void) {
    float t_obj, t_amb;
    esp_err_t e1 = mlx90614_leer_temp(MLX90614_REG_TOBJ1, &t_obj);
    esp_err_t e2 = mlx90614_leer_temp(MLX90614_REG_TA, &t_amb);
    if (e1 != ESP_OK || e2 != ESP_OK) {
        ESP_LOGW(TAG, "Error leyendo MLX90614");
        return;
    }

    int16_t obj_centi = (int16_t)(t_obj * 100.0f); // centesimas de grado C
    int16_t amb_centi = (int16_t)(t_amb * 100.0f);

    uint8_t datos[5];
    datos[0] = obj_centi & 0xFF;
    datos[1] = (obj_centi >> 8) & 0xFF;
    datos[2] = amb_centi & 0xFF;
    datos[3] = (amb_centi >> 8) & 0xFF;
    datos[4] = seq_temperatura++;

    if (!twai_enviar_frame(CAN_ID_TEMPERATURA, datos, sizeof(datos))) {
        ESP_LOGW(TAG, "Fallo al enviar frame de temperatura");
    }
}

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
