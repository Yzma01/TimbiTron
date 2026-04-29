# BrachioGraph con ESP32 — guía de setup y troubleshooting

Este fork reemplaza el Raspberry Pi Zero original por un ESP32 conectado
por USB serial al PC. **Toda la cinemática sigue corriendo en el PC**;
el ESP32 solo recibe comandos y mueve los servos.

## Arquitectura en 1 párrafo

```
   [ PC (Python) ]  --USB serial 115200--  [ ESP32 ]  --PWM--  [ 3 servos ]
        |                                      |
   brachiograph.py                      brachiograph_esp32.ino
   plotter.py
```

El PC manda JSON línea por línea. El ESP32 responde `OK` o `ERR:<motivo>`
a cada comando. El PC **espera el ACK** antes de mandar el siguiente —
esto es lo que evita que el buffer UART se sature y se pierdan datos.

## Protocolo (por si necesitas debuggearlo a mano)

```
PC -> ESP32                         ESP32 -> PC
------------------------------      ----------------------
{"s1":1450,"s2":1300}               OK
{"pen":1200,"pen_ms":250}           OK
{"s1":0,"s2":0,"pen":0}             OK          (detach all)
{}                                  OK          (ping / handshake)
{"s1":99999}                        ERR:pw_out_of_range
basura no-JSON                      ERR:InvalidInput
                                    # cualquier info de debug
                                    READY       (una vez al bootear)
```

Las líneas que empiezan con `#` son informativas — el driver Python las
imprime en pantalla pero no las procesa como parte del protocolo.

## Setup paso a paso

### 1. Flashear el ESP32

1. Abrir `esp32/brachiograph_esp32/brachiograph_esp32.ino` en Arduino IDE.
2. Instalar las librerías necesarias (Tools → Manage Libraries):
   - **ESP32Servo** (Kevin Harrington)
   - **ArduinoJson v6.x** (Benoît Blanchon) — ⚠ NO usar v7, cambia la API.
3. Seleccionar la placa (Tools → Board → ESP32 Arduino → tu modelo,
   p. ej. "ESP32 Dev Module").
4. Seleccionar el puerto (Tools → Port → COMx / /dev/ttyUSBx).
5. Si quieres cambiar los pines de los servos, edita las tres constantes
   al inicio del sketch:
   ```cpp
   const int SERVO1_PIN = 27;   // shoulder
   const int SERVO2_PIN = 33;   // elbow
   const int SERVO3_PIN = 32;   // pen
   ```
   **Evita:** 0, 2, 5, 12, 15 (strapping pins que afectan el boot),
   y 34–39 (input-only, no pueden manejar un servo).
6. Click en Upload.
7. Abrir Serial Monitor a 115200 baud. Deberías ver:
   ```
   # BrachioGraph ESP32 firmware v2 (serial + ACK)
   # pins s1=27 s2=33 pen=32
   # pw range 500..2500
   READY
   ```
   Si ves eso, el firmware está funcionando.

### 2. Probar el link PC ↔ ESP32 antes de plotear

**Cierra Arduino Serial Monitor** (no puede estar abierto si Python quiere el puerto)
y corre:

```bash
python test_esp32.py --port COM4
# o en Linux/Mac:
python test_esp32.py --port /dev/ttyUSB0
```

Debería imprimir una serie de tildes ✓ y `All tests passed`. Si ves ✗,
no sigas hasta resolverlo — los problemas solo empeoran plotando.

### 3. Plotear

Editar el `serial_port` en `bg.py` al puerto correcto y correr:

```python
python
>>> from bg import bg
>>> bg.box()                      # dibuja el rectángulo de los límites
>>> bg.plot_file("images/demo.json")
>>> bg.quiet()                    # libera los servos cuando termines
```

## Troubleshooting

### "No response from ESP32" / el handshake falla

1. **¿El puerto es el correcto?** En Windows, Device Manager → Ports (COM & LPT).
2. **¿Hay otro programa usando el puerto?** Arduino Serial Monitor, screen,
   minicom, PuTTY. Ciérralos todos.
3. **¿El firmware correcto está flasheado?** Abre Serial Monitor (con otro
   programa Python cerrado) y resetea el ESP32. Tiene que salir
   `READY`. Si no, re-flashea.
4. **¿Driver USB-UART?** CP2102 (chip silabs), CH340 o CH9102 según la placa.
   Windows a veces no instala el driver automático — descárgalo del fabricante.

### Funciona unos segundos y después se desconecta / resetea

Esto es casi siempre un **brownout**: los servos al moverse jalan demasiada
corriente y el ESP32 se resetea por caída de voltaje. Síntomas:
- Durante un plot, de repente aparece de nuevo `READY` en el log.
- Los primeros movimientos funcionan, luego nada.
- El LED del ESP32 parpadea raro cuando los servos arrancan.

**Solución:** los servos **NO** deben alimentarse del USB. Usa una fuente
externa de 5 V con al menos 2 A (idealmente 3 A para 3 servos), y conecta
**todas las tierras juntas**: GND del ESP32, GND de la fuente, GND de los servos.

```
          +5V fuente ─────┬───── rojo servo 1
                          ├───── rojo servo 2
                          └───── rojo servo 3
                              
          GND fuente ─────┬───── negro servo 1
                          ├───── negro servo 2
                          ├───── negro servo 3
                          └───── GND ESP32     ← ¡obligatorio!
          
          GPIO 27 ESP32  ─────── señal servo 1
          GPIO 33 ESP32  ─────── señal servo 2
          GPIO 32 ESP32  ─────── señal servo 3
          
          USB PC ────────────── ESP32 (solo para datos + alimentar al micro)
```

### "ERR:pw_out_of_range"

El pulso está fuera de 500–2500 µs. Revisa los arrays `servo_1_angle_pws` /
`servo_2_angle_pws` en `bg.py` — seguramente un valor extremo está fuera
del rango físico del servo. También puede venir de `pw_up` / `pw_down`.

### "ERR:InvalidInput" o "ERR:NoMemory"

Son errores de ArduinoJson. Significa que llegó basura al ESP32 — típicamente
porque otra cosa está escribiendo al puerto, o el cable USB es malo.
Prueba otro cable (los cables baratos de carga a veces no tienen líneas de
datos decentes).

### El plot sale deforme / con líneas fuera de lugar

Esto **no** es problema de comunicación — el PC está mandando los comandos
correctos y el ESP32 los acepta, pero los servos están mal calibrados.
Ejecuta el flujo de calibración: `bg.capture_pws()` en una sesión
interactiva y anota los valores en `bg.py`.

### Quiero verificar que los servos se muevan sin usar Python

Flashear `esp32/calibrate/calibrate.ino` — manda los tres servos a 1500 µs
(centro mecánico) y los mantiene ahí. Útil para verificar el cableado.

### Quiero ver los comandos que manda el PC en tiempo real

En Python:

```python
import plotter
plotter.ESP32_ACK_TIMEOUT = 0.5   # ya es el default, pero por si acaso
```

El firmware imprime todas las respuestas en el mismo canal serial. Si
quieres ver los comandos crudos, agrega un `print(payload)` en
`plotter.Plotter.send_command()`.

## ¿Por qué serial y no WiFi?

Probamos WebSocket, TCP y UDP — los tres son viables pero tienen más
puntos de falla (WiFi flaco, routers con filtros, NAT en AP mode,
IPs que cambian). Serial es más simple y **la latencia es mejor**:
~1 ms round-trip por comando, sin jitter de WiFi. El protocolo de este
repo está diseñado pensando en ack-por-comando, así que si más adelante
quieres pasarlo a TCP, es directo: el mismo framing línea-por-línea y las
mismas respuestas `OK` / `ERR:` funcionan igual sobre un socket.

## Archivos clave

| Archivo                                          | Qué hace                                      |
|--------------------------------------------------|-----------------------------------------------|
| `esp32/brachiograph_esp32/brachiograph_esp32.ino`| **firmware activo** (serial + ACK)            |
| `esp32/calibrate/calibrate.ino`                  | sketch de centrado de servos (verificar cableado) |
| `esp32/archive/brachiograph_esp32_websocket.ino.txt` | firmware WebSocket viejo, archivado como referencia |
| `plotter.py`                                     | driver Python: thread lector, handshake, ACK  |
| `test_esp32.py`                                  | smoke test del link — correr antes de plotear |
| `bg.py`                                          | punto de entrada; aquí se configura el puerto |
