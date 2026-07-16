 // Pin definitions
#define StopCalIn  GPIO_NUM_01   // Pin to stop calibration process

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

// En modo full-res son ~4 mg/LSB sea cual sea el rango, o sea 1g equivale
// a ~250 cuentas. Un gain de calibracion muy alejado de eso indica que el
// eje no roto realmente por +-1g (sensor quieto) o que la lectura fue mala.
#define ADXL345_GAIN_MIN_ESPERADO  100
#define ADXL345_GAIN_MAX_ESPERADO  400

// Registros MLX90614 (RAM)
#define MLX90614_REG_TA     0x06
#define MLX90614_REG_TOBJ1  0x07

// ---------------- Configuracion TWAI/CAN (AJUSTAR) ----------------
#define CAN_TX_PIN   GPIO_NUM_4
#define CAN_RX_PIN   GPIO_NUM_5
#define CAN_ID_INCLINACION  0x200
#define CAN_ID_TEMPERATURA  0x201
#define CAN_ID_CALIBRACION_MIN  0x202
#define CAN_ID_CALIBRACION_MAX  0x203
#define CAN_ID_COMANDO       0x210   // Comandos enviados por el Arduino Nano

// Comandos aceptados en CAN_ID_COMANDO (byte 0 del frame)
#define CAN_CMD_CALIBRACION_START  0x01
#define CAN_CMD_CALIBRACION_STOP   0x02

#define LED_PIN GPIO_NUM_8

// ---------------- Estado de calibracion compartido con main.c ----------------
// Declaraciones extern: la definicion real (con storage) vive en Functions.c,
// para que no se dupliquen simbolos al incluir este header en varios .c.
extern bool CalibracionValida;   // true cuando offset/gain son de una calibracion real (NVS o recien hecha)

extern int offsetX, offsetY, offsetZ;  // OFFSET values
extern int gainX, gainY, gainZ;        // GAIN factors