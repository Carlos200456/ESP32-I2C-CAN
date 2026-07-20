# Protocolo CAN — Nodo remoto ESP32-C6

Bus TWAI/CAN clásico (no FD), IDs estándar de 11 bits, `100 kbit/s` por defecto (ver
[../src/Functions.c](../src/Functions.c), función `twai_init_bus`). Todos los campos
multi-byte van en **little-endian** (LSB primero).

## IDs

| ID | Nombre | Dirección | Largo | Frecuencia |
|---|---|---|---|---|
| `0x200` | Inclinación | Nodo → Nano | 5 | 1 Hz (pausado durante calibración) |
| `0x201` | Temperatura | Nodo → Nano | 5 | 0.5 Hz (pausado durante calibración) |
| `0x202` | Calibración — mínimos | Nodo → Nano | 6 | 5 Hz, solo durante calibración |
| `0x203` | Calibración — máximos | Nodo → Nano | 6 | 5 Hz, solo durante calibración |
| `0x210` | Comando | Nano → Nodo | ≥1 | A demanda |

## `0x200` — Inclinación

| Byte | Campo | Tipo | Unidad |
|---|---|---|---|
| 0–1 | `pitch` | `int16` LE | centésimas de grado |
| 2–3 | `roll` | `int16` LE | centésimas de grado |
| 4 | `seq` | `uint8` | contador de secuencia (0–255, desborda) |

- `pitch = ángulo / 100.0` en grados. Ídem `roll`.
- `seq` incrementa en cada envío; sirve para detectar frames perdidos o datos viejos del
  lado del Nano.
- Solo se envía si hay una calibración válida cargada (guardada en NVS o recién hecha).
  Si no la hay, el nodo no transmite este frame.

## `0x201` — Temperatura

| Byte | Campo | Tipo | Unidad |
|---|---|---|---|
| 0–1 | `t_obj` (temperatura objeto, IR) | `int16` LE | centésimas de °C |
| 2–3 | `t_amb` (temperatura ambiente) | `int16` LE | centésimas de °C |
| 4 | `seq` | `uint8` | contador de secuencia (0–255, desborda) |

`t_obj`/`t_amb = valor / 100.0` en °C. No depende de la calibración del acelerómetro.

## `0x202` / `0x203` — Extremos de calibración (min / max)

Se emiten únicamente mientras hay una calibración en curso (iniciada por `0x01` en
`0x210` o por teclado USB), a 5 Hz. Llevan las cuentas **crudas** del ADXL345 (no
convertidas a `g`) observadas hasta ese momento en cada eje.

| Byte | Campo | Tipo |
|---|---|---|
| 0–1 | eje X | `int16` LE |
| 2–3 | eje Y | `int16` LE |
| 4–5 | eje Z | `int16` LE |

`0x202` lleva los mínimos acumulados, `0x203` los máximos. Se envían en frames separados
porque los 6 valores (12 bytes) no entran en un frame CAN clásico (máximo 8 bytes de
datos).

Estos frames pueden fallar al transmitir sin que sea un error real (p. ej. calibración de
fábrica por USB con el bus CAN desconectado); el nodo los ignora silenciosamente
(log a nivel `DEBUG`) en ese caso.

## `0x210` — Comando (Nano → Nodo)

| Byte | Campo | Valores |
|---|---|---|
| 0 | comando | `0x01` = iniciar calibración, `0x02` = finalizar y guardar calibración |

Cualquier otro valor en el byte 0 se ignora (se loguea como advertencia). El nodo drena
todos los frames pendientes de este ID en cada vuelta del loop principal, así que varios
comandos en la cola se procesan en orden dentro de la misma iteración.

Iniciar calibración (`0x01`) tiene el mismo efecto que escribir `s` + ENTER por USB;
finalizar (`0x02`) el mismo efecto que `f` + ENTER. Ver
[../README.md](../README.md#calibración-del-acelerómetro) para el flujo completo de
calibración y las condiciones de aceptación/rechazo.

## Recuperación de bus

El nodo chequea el estado del bus TWAI cada 2 segundos (`twai_chequear_bus`). Si detecta
`TWAI_STATE_BUS_OFF`, inicia recuperación automática (`twai_initiate_recovery`) y
reinicia el driver (`twai_start`) tras 100 ms.
