#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "led_strip.h"
#define CONFIG_TWAI_SUPPRESS_DEPRECATE_WARN 1
#include "driver/twai.h"
#include "esp_log.h"
#include "Functions.h"
#include "Globals.h"

static const char *TAG = "NODO_REMOTO";

// Estado interno, solo usado dentro de este archivo.
static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t adxl345_dev;
static i2c_master_dev_handle_t mlx90614_dev;
static led_strip_handle_t led_strip;
static uint8_t seq_inclinacion = 0;
static uint8_t seq_temperatura = 0;
static bool CalibracionInProgress = false; // seteado por comando CAN del Nano o teclado USB
static int AccelMinX = 0, AccelMaxX = 0;
static int AccelMinY = 0, AccelMaxY = 0;
static int AccelMinZ = 0, AccelMaxZ = 0;

// Estado compartido con main.c (declarado extern en Globals.h).
bool CalibracionValida = false;
int offsetX = 0, offsetY = 0, offsetZ = 0;
int gainX = 255, gainY = 255, gainZ = 255;

// ============================================================
// I2C - funciones de bajo nivel
// ============================================================

// Nota timeouts: a diferencia del driver legacy (que tomaba ticks via
// pdMS_TO_TICKS), i2c_master.h toma milisegundos directos.
static esp_err_t i2c_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

static esp_err_t i2c_read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, 100);
}

static esp_err_t i2c_agregar_dispositivo(uint16_t dev_addr, i2c_master_dev_handle_t *out_dev) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(i2c_bus, &dev_config, out_dev);
}

void i2c_init(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));
    ESP_ERROR_CHECK(i2c_agregar_dispositivo(ADXL345_ADDR, &adxl345_dev));
    ESP_ERROR_CHECK(i2c_agregar_dispositivo(MLX90614_ADDR, &mlx90614_dev));
}

// ============================================================
// I2C - diagnostico de cableado (debug por USB Serial/JTAG)
// ============================================================

void i2c_escanear_bus(void) {
    ESP_LOGI(TAG, "Escaneando bus I2C (SDA=%d, SCL=%d)...", I2C_SDA_PIN, I2C_SCL_PIN);
    uint8_t encontrados = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(i2c_bus, addr, 100) == ESP_OK) {
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
bool adxl345_verificar_id(void) {
    uint8_t devid = 0;
    esp_err_t err = i2c_read_regs(adxl345_dev, ADXL345_REG_DEVID, &devid, 1);
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

esp_err_t adxl345_init(void) {
    esp_err_t err;
    // DATA_FORMAT: full-res (bit3=0x08) + rango +-4g (bits1:0=0x01)
    err = i2c_write_reg(adxl345_dev, ADXL345_REG_DATA_FORMAT, 0x08 | 0x01);
    if (err != ESP_OK) return err;
    // POWER_CTL: bit Measure = 0x08 (arranca la medicion)
    err = i2c_write_reg(adxl345_dev, ADXL345_REG_POWER_CTL, 0x08);
    return err;
}

static esp_err_t adxl345_leer_raw(int16_t *raw_x, int16_t *raw_y, int16_t *raw_z) {
    uint8_t data[6];
    esp_err_t err = i2c_read_regs(adxl345_dev, ADXL345_REG_DATAX0, data, sizeof(data));
    if (err != ESP_OK) return err;

    *raw_x = (int16_t)((data[1] << 8) | data[0]);
    *raw_y = (int16_t)((data[3] << 8) | data[2]);
    *raw_z = (int16_t)((data[5] << 8) | data[4]);
    return ESP_OK;
}

// ============================================================
// MLX90614 (lectura simplificada, sin validar PEC/CRC del SMBus)
// ============================================================

static esp_err_t mlx90614_leer_temp(uint8_t reg, float *temp_c) {
    uint8_t data[3]; // LSB, MSB, PEC (PEC se lee pero no se valida)
    esp_err_t err = i2c_read_regs(mlx90614_dev, reg, data, sizeof(data));
    if (err != ESP_OK) return err;

    uint16_t raw = (data[1] << 8) | data[0];
    *temp_c = (raw * 0.02f) - 273.15f;
    return ESP_OK;
}

// El MLX90614 no tiene un registro DEVID simple: se valida presencia
// (ACK a la direccion) y que la temperatura ambiente devuelta caiga
// dentro del rango util del sensor, para descartar datos basura por
// mal cableado o SDA/SCL en corto.
bool mlx90614_verificar(void) {
    if (i2c_master_probe(i2c_bus, MLX90614_ADDR, 100) != ESP_OK) {
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

void twai_init_bus(void) {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    // Elegir la linea que coincida con el bitrate del bus existente:
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_100KBITS();
    // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
    // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI iniciado correctamente");
}

void twai_chequear_bus(void) {
    twai_status_info_t estado;
    twai_get_status_info(&estado);
    if (estado.state == TWAI_STATE_BUS_OFF) {
        ESP_LOGW(TAG, "Bus CAN en BUS-OFF, recuperando...");
        twai_initiate_recovery();
        vTaskDelay(pdMS_TO_TICKS(100));
        twai_start();
    }
}

// Revisa la cola de recepcion TWAI sin bloquear (timeout=0) y procesa
// un comando si llego. Devuelve true si habia un frame para procesar,
// para poder llamarla en loop y drenar varios comandos por iteracion.
bool twai_recibir_comando(void) {
    twai_message_t msg;
    if (twai_receive(&msg, 0) != ESP_OK) {
        return false; // no hay ningun frame pendiente
    }
    if (msg.identifier != CAN_ID_COMANDO || msg.data_length_code < 1) {
        return true;
    }

    switch (msg.data[0]) {
        case CAN_CMD_CALIBRACION_START:
            calibracion_iniciar();
            break;
        case CAN_CMD_CALIBRACION_STOP:
            calibracion_finalizar();
            break;
        default:
            ESP_LOGW(TAG, "Comando CAN desconocido: 0x%02X", msg.data[0]);
            break;
    }
    return true;
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
void ws2812_init(void) {
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

void ws2812_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, red, green, blue));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
}

void ws2812_rainbow(uint8_t steps, TickType_t delay_ticks, uint8_t brillo) {
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

void enviar_inclinacion(void) {
    if (!CalibracionValida) {
        ESP_LOGW(TAG, "Sin calibracion guardada: no se envia pitch/roll. Calibrar antes de operar.");
        ws2812_set_color(10, 8, 0); // Ambar: pendiente de calibracion
        return;
    }

    int16_t raw_x, raw_y, raw_z;
    if (adxl345_leer_raw(&raw_x, &raw_y, &raw_z) != ESP_OK) {
        ESP_LOGW(TAG, "Error leyendo ADXL345");
        // Cambiar el color del LED a rojo para indicar fallo de sensor
        ws2812_set_color(0, 0, 10); // Blue
        return;
    }
    // offsetX/gainX salen de AccelMin?/AccelMax?, que se calibran en
    // cuentas crudas (ver calibracion_paso): hay que restar/dividir en
    // ese mismo dominio para que el resultado quede normalizado a ~1g
    // por eje, no mezclar cuentas crudas con g ya convertidos.
    float accX, accY, accZ;
    accX = (float)(raw_x - offsetX) / gainX;
    accY = (float)(raw_y - offsetY) / gainY;
    accZ = (float)(raw_z - offsetZ) / gainZ;

    // Calibrar mapeo de ejes segun montaje fisico real sobre el tubo
    float pitch = atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * 180.0f / (float)M_PI;
    float roll  = atan2f(accY, accZ) * 180.0f / (float)M_PI;

    int16_t pitch_centi = (int16_t)(pitch * 100.0f); // centesimas de grado
    int16_t roll_centi  = (int16_t)(roll * 100.0f);

    uint8_t datos[5];
    datos[0] = pitch_centi & 0xFF;
    datos[1] = (pitch_centi >> 8) & 0xFF;
    datos[2] = roll_centi & 0xFF;
    datos[3] = (roll_centi >> 8) & 0xFF;
    datos[4] = seq_inclinacion++; // contador para detectar datos viejos/perdidos

    if (!twai_enviar_frame(CAN_ID_INCLINACION, datos, sizeof(datos))) {
        ESP_LOGW(TAG, "Pitch=%.0f, Roll=%.0f", pitch, roll);
        ESP_LOGW(TAG, "Fallo al enviar frame de inclinacion");
    }
}

void enviar_temperatura(void) {
    float t_obj, t_amb;
    esp_err_t e1 = mlx90614_leer_temp(MLX90614_REG_TOBJ1, &t_obj);
    esp_err_t e2 = mlx90614_leer_temp(MLX90614_REG_TA, &t_amb);
    if (e1 != ESP_OK || e2 != ESP_OK) {
        ESP_LOGW(TAG, "Error leyendo MLX90614");
        // Cambiar el color del LED a rojo para indicar fallo de sensor
        ws2812_set_color(10, 0, 0); // Red
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
        ESP_LOGW(TAG, "t_obj=%.2f, t_amb=%.2f", t_obj, t_amb);
        ESP_LOGW(TAG, "Fallo al enviar frame de temperatura");
    }
}

// Envia los extremos min/max acumulados de los 3 ejes para que el Nano
// los muestre al usuario durante la calibracion. Van en dos frames
// separados (min y max) porque 6 valores de 16 bits (12 bytes) no
// entran en un solo frame CAN clasico (maximo 8 bytes de datos).
void enviar_calibracion(int min_x, int max_x, int min_y, int max_y, int min_z, int max_z) {
    uint8_t datos_min[6];
    datos_min[0] = min_x & 0xFF;
    datos_min[1] = (min_x >> 8) & 0xFF;
    datos_min[2] = min_y & 0xFF;
    datos_min[3] = (min_y >> 8) & 0xFF;
    datos_min[4] = min_z & 0xFF;
    datos_min[5] = (min_z >> 8) & 0xFF;

    uint8_t datos_max[6];
    datos_max[0] = max_x & 0xFF;
    datos_max[1] = (max_x >> 8) & 0xFF;
    datos_max[2] = max_y & 0xFF;
    datos_max[3] = (max_y >> 8) & 0xFF;
    datos_max[4] = max_z & 0xFF;
    datos_max[5] = (max_z >> 8) & 0xFF;

    bool env_min = twai_enviar_frame(CAN_ID_CALIBRACION_MIN, datos_min, sizeof(datos_min));
    bool env_max = twai_enviar_frame(CAN_ID_CALIBRACION_MAX, datos_max, sizeof(datos_max));
    if (!env_min || !env_max) {
        // Nivel debug (no warning): durante una calibracion de fabrica por
        // USB el bus CAN suele estar desconectado, y esto se repite en
        // cada paso. No es un error real que le importe al operador.
        ESP_LOGD(TAG, "Fallo al enviar frame de calibracion (min=(%d,%d,%d) max=(%d,%d,%d)) - normal si no hay Nano conectado",
                 min_x, min_y, min_z, max_x, max_y, max_z);
    }
}

// Arranca la calibracion: reinicia los extremos min/max para que el
// primer dato leido los establezca, y prende el flag que habilita el
// envio periodico de datos crudos (ver calibracion_paso).
void calibracion_iniciar(void) {
    AccelMinX = AccelMinY = AccelMinZ = INT16_MAX;
    AccelMaxX = AccelMaxY = AccelMaxZ = INT16_MIN;
    CalibracionInProgress = true;
    ESP_LOGI(TAG, "Calibracion iniciada");
}

bool calibracion_en_curso(void) {
    return CalibracionInProgress;
}

// Se llama periodicamente desde el loop principal mientras
// calibracion_en_curso() sea true: lee el acelerometro, actualiza los
// extremos min/max observados y los reenvia por CAN (para el Nano) y por
// consola USB, reescribiendo una sola linea en el lugar (\r, sin \n).
// No usamos secuencias ANSI de mover el cursor entre filas: si el
// contenido ya llego al borde inferior de la terminal, cada \n fuerza
// un scroll de toda la pantalla y la "linea fija" termina subiendo sin
// parar. Con \r alcanza porque nunca cambiamos de fila.
void calibracion_paso(void) {
    int16_t raw_x, raw_y, raw_z;
    if (adxl345_leer_raw(&raw_x, &raw_y, &raw_z) != ESP_OK) {
        ESP_LOGW(TAG, "Error leyendo ADXL345 durante calibracion");
        return;
    }

    if (raw_x < AccelMinX) AccelMinX = raw_x;
    if (raw_x > AccelMaxX) AccelMaxX = raw_x;
    if (raw_y < AccelMinY) AccelMinY = raw_y;
    if (raw_y > AccelMaxY) AccelMaxY = raw_y;
    if (raw_z < AccelMinZ) AccelMinZ = raw_z;
    if (raw_z > AccelMaxZ) AccelMaxZ = raw_z;

    enviar_calibracion(AccelMinX, AccelMaxX, AccelMinY, AccelMaxY, AccelMinZ, AccelMaxZ);

    // Ancho fijo (%5d) en vez de \033[K: la terminal del usuario no
    // interpreta esa secuencia de escape y la muestra como texto suelto.
    // Con ancho fijo la linea siempre mide lo mismo, asi que no hace
    // falta borrar restos del valor anterior.
    printf("\rMin X=%5d Y=%5d Z=%5d | Max X=%5d Y=%5d Z=%5d",
           AccelMinX, AccelMinY, AccelMinZ, AccelMaxX, AccelMaxY, AccelMaxZ);
    // No hay ningun \n en este printf, asi que stdio no lo manda solo:
    // forzamos el flush para que se vea en el momento y no se acumule
    // sin transmitir hasta el proximo log con salto de linea.
    fflush(stdout);
    fsync(fileno(stdout));
}

// Un gain fuera de este rango indica que ese eje no roto realmente por
// +-1g durante la calibracion (sensor quieto, mal movimiento, etc).
static bool gain_es_plausible(int gain) {
    return gain >= ADXL345_GAIN_MIN_ESPERADO && gain <= ADXL345_GAIN_MAX_ESPERADO;
}

// Corta el envio de datos crudos, calcula offset/gain a partir de los
// extremos acumulados desde calibracion_iniciar() y los persiste en NVS.
// Si el resultado no parece una calibracion real (gain nulo o fuera de
// rango en algun eje) la descarta: no la guarda ni pisa una calibracion
// valida anterior, para que el usuario tenga que repetirla.
void calibracion_finalizar(void) {
    CalibracionInProgress = false;
    printf("\n"); // cerrar la linea que se venia reescribiendo en calibracion_paso()

    int nuevo_offsetX = (AccelMaxX + AccelMinX) / 2;
    int nuevo_gainX   = (AccelMaxX - AccelMinX) / 2;
    int nuevo_offsetY = (AccelMaxY + AccelMinY) / 2;
    int nuevo_gainY   = (AccelMaxY - AccelMinY) / 2;
    int nuevo_offsetZ = (AccelMaxZ + AccelMinZ) / 2;
    int nuevo_gainZ   = (AccelMaxZ - AccelMinZ) / 2;

    if (!gain_es_plausible(nuevo_gainX) || !gain_es_plausible(nuevo_gainY) || !gain_es_plausible(nuevo_gainZ)) {
        ESP_LOGE(TAG, "Calibracion rechazada: gain fuera de rango (esperado %d..%d) gainX=%d, gainY=%d, gainZ=%d. Repetir moviendo bien los 3 ejes.",
                 ADXL345_GAIN_MIN_ESPERADO, ADXL345_GAIN_MAX_ESPERADO, nuevo_gainX, nuevo_gainY, nuevo_gainZ);
        ws2812_set_color(10, 0, 0); // rojo: calibracion invalida
        return;
    }

    offsetX = nuevo_offsetX;
    gainX   = nuevo_gainX;
    offsetY = nuevo_offsetY;
    gainY   = nuevo_gainY;
    offsetZ = nuevo_offsetZ;
    gainZ   = nuevo_gainZ;
    guardar_calibracion(offsetX, offsetY, offsetZ, gainX, gainY, gainZ);
    CalibracionValida = true;

    ESP_LOGI(TAG, "Calibracion finalizada: offsetX=%d, gainX=%d, offsetY=%d, gainY=%d, offsetZ=%d, gainZ=%d",
             offsetX, gainX, offsetY, gainY, offsetZ, gainZ);
}
