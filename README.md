# ESP32-I2C-CAN — Nodo remoto (inclinación + temperatura)

Firmware para un **ESP32-C6** que actúa como nodo remoto de sensores: lee inclinación
(acelerómetro **ADXL345**) y temperatura (termómetro IR **MLX90614**) por **I2C**, y
transmite esos datos por **CAN (TWAI nativo)** a un **Arduino Nano** base, compartiendo
el mismo bus físico que ya usa el colimador.

Escrito en **ESP-IDF puro** (no Arduino), usando **PlatformIO** con
`framework = espidf`.

## Hardware

| Función | Pin ESP32-C6 |
|---|---|
| I2C SDA | GPIO2 |
| I2C SCL | GPIO3 |
| CAN TX (TWAI) | GPIO4 |
| CAN RX (TWAI) | GPIO5 |
| LED de estado (WS2812) | GPIO8 |
| Entrada auxiliar `StopCalIn` | GPIO1 |

Los pines se ajustan en [include/Globals.h](include/Globals.h). El bus CAN corre a
**100 kbit/s** por defecto (`TWAI_TIMING_CONFIG_100KBITS()` en
[src/Functions.c](src/Functions.c)); hay líneas comentadas para 125/250/500 kbit/s si
el bus del colimador usa otra velocidad.

Sensores I2C:
- **ADXL345** (acelerómetro) en `0x53` (o `0x1D` si el pin SDO está a VCC).
- **MLX90614** (termómetro IR) en `0x5A`.

Un LED WS2812 en GPIO8 indica el estado del nodo (ver [Indicaciones del LED](#indicaciones-del-led)).

## Arquitectura del código

- [src/main.c](src/main.c) — `app_main()`: inicialización y loop principal (no bloqueante,
  basado en `TickType_t` y periodos, sin FreeRTOS tasks separadas).
- [src/Functions.c](src/Functions.c) — implementación de I2C, ADXL345, MLX90614, TWAI,
  LED WS2812 y armado de tramas CAN. Las firmas públicas están en
  [include/Functions.h](include/Functions.h).
- [src/NVS.c](src/NVS.c) — persistencia de la calibración del acelerómetro en NVS.
- [include/Globals.h](include/Globals.h) — pines, direcciones I2C, registros de los
  sensores, IDs de CAN y estado global compartido (`extern`).

### Loop principal

En cada vuelta del `while(1)` de `app_main()`:
1. Drena comandos CAN pendientes (`twai_recibir_comando`).
2. Sondea el teclado por USB Serial/JTAG sin bloquear, para disparar calibración manual.
3. Si no hay calibración en curso, envía inclinación cada 1000 ms y temperatura cada 2000 ms.
4. Cada 2000 ms chequea el estado del bus CAN y lo recupera si quedó en `BUS-OFF`.
5. Si hay calibración en curso, ejecuta un paso de calibración cada 200 ms.
6. `vTaskDelay(10 ms)`.

## Calibración del acelerómetro

El ADXL345 necesita un offset y una ganancia por eje para convertir cuentas crudas a
`g`. Esa calibración se puede disparar de dos formas:

- **Por USB** (consola serie, `monitor_speed = 115200`): escribir `s` + ENTER para
  iniciar, mover el sensor barriendo lentamente los 3 ejes (±1g), y `f` + ENTER para
  finalizar y guardar.
- **Por CAN**, con un comando enviado por el Nano en `CAN_ID_COMANDO` (`0x210`):
  byte 0 = `0x01` (iniciar) o `0x02` (finalizar).

Mientras la calibración está en curso, el nodo deja de enviar inclinación/temperatura y
en su lugar transmite los extremos min/max observados de cada eje (para que el Nano los
muestre) y los imprime por consola.

Al finalizar, si la ganancia calculada de algún eje cae fuera del rango plausible
(100–400 cuentas ≈ 1g en full-res), la calibración se **rechaza** y no se guarda ni pisa
una calibración previa válida. Si es aceptada, se persiste en NVS (namespace `calib`,
ver [src/NVS.c](src/NVS.c)) y se recupera automáticamente en cada arranque. Hasta que no
haya una calibración válida (ni guardada ni recién hecha), no se envía pitch/roll.

## Indicaciones del LED (WS2812, GPIO8)

| Color | Significado |
|---|---|
| Arcoíris (al arrancar) | Secuencia de inicio |
| Verde tenue | Estado inicial / operación normal |
| Ámbar | Sin calibración guardada, pitch/roll no se envía |
| Azul | Error de lectura del ADXL345 |
| Rojo | Error de lectura del MLX90614, o calibración rechazada |

## Protocolo CAN

Ver [docs/PROTOCOLO_CAN.md](docs/PROTOCOLO_CAN.md) para el detalle de IDs, formato de
bytes y semántica de cada trama.

## Compilar y flashear

Requiere [PlatformIO](https://platformio.org/) (CLI o extensión de VSCode/CLion). El
target es `esp32-c6-devkitc-1` con `framework = espidf` (ver
[platformio.ini](platformio.ini)).

```bash
pio run                 # compilar
pio run --target upload # flashear
pio device monitor      # consola serie (115200 baud)
```

`pio run --target upload --target monitor` compila, flashea y abre la consola en un
solo paso.

### Dependencias

- Componente administrado `espressif/led_strip` (ver
  [src/idf_component.yml](src/idf_component.yml)), usado para el LED WS2812 vía RMT.
- Resto de la funcionalidad usa componentes estándar de ESP-IDF (`driver`, `nvs_flash`,
  `esp_log`, FreeRTOS).
