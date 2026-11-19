# Autonomous-Mosquito-Detector
 
An ESP32 pan-tilt turret that patrols a room, wakes up when a **24 GHz mmWave radar** senses movement, and confirms a mosquito by **listening for its wingbeat** with an I2S digital microphone and a 512-point FFT. The firmware runs on **FreeRTOS**, with three tasks sharing one mutex-protected state.
 
> Mini-project for the Real-Time Embedded Systems course (Year 3).

---
 
## How it works
 
The turret is always in one of three states:
 
```
  SCANNING (green)  ──radar sees motion──▶  ALERT (yellow)  ──mic confirms mosquito──▶  LOCKED (red + buzzer)
        ▲                                        │                                          │
        └────────── 5 s with no motion ──────────┘◀──────────── sound stops ────────────────┘
```
 
| State | LED | Servos | Buzzer | Triggered by |
|---|---|---|---|---|
| `SCANNING` | Green | Sweep at normal speed, random bounds and pauses | Off | Default / nothing detected |
| `ALERT` | Yellow | Sweep at half speed to search | Off | Radar motion within the last 5 s |
| `LOCKED` | Red | Freeze in place | Pulsing | Mosquito sound confirmed for 4 frames in a row |
 
**Two-stage detection:**
 
1. **Eyes: radar (stage 1).** The HLK-LD2410C's `OUT` pin goes HIGH when it senses presence. A 5-second software hold window keeps the state from flickering.
2. **Ears: microphone (stage 2).** The INMP441 streams 16 kHz audio over I2S with DMA. Each frame of 512 samples goes through an FFT (31.25 Hz per bin), and a frame counts as "mosquito" only if:
   - the loudest frequency is between **420 and 800 Hz** (the wingbeat),
   - the 406–812 Hz band is **4× louder than the background noise** and above an energy threshold,
   - there is also a **harmonic** in the 906–1593 Hz band that is 2× above the noise.
   Four such frames in a row (about 250 ms) lock the target. Hand claps and servo noise are rejected because they raise every frequency at once instead of one narrow band.
---
 
## Hardware
 
| Part | Role |
|---|---|
| ESP32 DevKit V1 (DOIT, 30-pin, ESP-WROOM-32) | Brain: runs FreeRTOS and the FFT |
| HLK-LD2410C 24 GHz mmWave radar | Motion / presence sensor |
| INMP441 I2S MEMS microphone | Captures the wingbeat sound |
| 2× servo (pan + tilt) on a pan-tilt bracket | Moves the sensor head |
| Traffic-light LED module (R / Y / G, built-in resistors) | State display |
| Active-low buzzer module | Alarm in `LOCKED` |
| External 5–6 V DC adapter + barrel-jack screw terminal | Servo power only |
| 2× breadboard, jumper wires (M-M and M-F) | Wiring |
 
---
 
## Wiring
 
### Pin map
 
| Component | Component pin | ESP32 pin |
|---|---|---|
| **INMP441 mic** | SCK | GPIO33 |
| | WS | GPIO25 |
| | SD | GPIO32 |
| | L/R | GND (selects the left channel) |
| | VDD | 3V3 |
| | GND | GND |
| **LD2410C radar** | OUT | GPIO18 |
| | VCC | VIN (5 V from USB) |
| | GND | GND |
| | TX / RX | Not connected (the firmware only uses `OUT`) |
| **Pan servo** | Signal (orange) | GPIO13 |
| **Tilt servo** | Signal (orange) | GPIO14 |
| **Both servos** | Power (red) | External adapter **+** |
| | Ground (brown) | GND rail |
| **LED module** | R | GPIO27 |
| | Y | GPIO26 |
| | G | GPIO23 |
| | GND | GND |
| **Buzzer** | I/O | GPIO21 |
| | VCC | 3V3 |
| | GND | GND |
 
### Where the pins are on the DevKit V1
 
Looking down at the board, with the antenna at the top:
 
```
                 ┌──────────┐
             EN  │ antenna  │  D23  ── Green LED
            D36  │          │  D22
            D39  │          │  TX0
            D34  │          │  RX0
            D35  │          │  D21  ── Buzzer I/O
 Mic SD  ── D32  │          │  D19
 Mic SCK ── D33  │          │  D18  ── Radar OUT
 Mic WS  ── D25  │          │  D5
 Yellow  ── D26  │          │  TX2
 Red     ── D27  │          │  RX2
 Tilt    ── D14  │          │  D4
   (keep empty) D12  │      │  D2
 Pan     ── D13  │          │  D15
 GND rail ─ GND  │          │  GND  ── GND rail
 Radar 5V ─ VIN  │   USB    │  3V3  ── Mic VDD, buzzer VCC
                 └──────────┘
```
 
> GPIO12 is a boot-strapping pin. Leave it empty or the board may not boot.
 
### Power
 
```
 DC adapter (5–6 V) ──+──▶  Servo + rail  ──▶  both red servo wires
                      └─–─▶ ┐
 ESP32 GND ─────────────────┼──▶  Shared GND rail  ◀── every GND / brown wire
                            ┘
 ESP32 (USB) 3V3 / VIN ──────▶  mic, buzzer, LEDs, radar
```
 
- **Never power the servos from the ESP32.** Two servos draw more current than the board can supply, so the ESP32 browns out and resets.
- **All grounds must be joined**, including the adapter's –. Without a shared ground the servos jitter or don't move.
- **Never connect the servo + rail to 3V3 or VIN.**
- Check the adapter label. If it is 9 V or 12 V, add a buck converter set to about 5 V before the servo rail.
- If the ESP32 resets when the servos move fast, add a 470–1000 µF capacitor across the servo + and GND rails.
- On long breadboards the power rails are often split in the middle. Bridge the gap with a jumper if needed.
---
 
## Software
 
### Requirements
 
- Arduino IDE with the **esp32** board package (Espressif). Select **DOIT ESP32 DEVKIT V1**.
- Libraries (Library Manager):
  - **ESP32Servo**
  - **arduinoFFT** v2.x (uses the `ArduinoFFT<double>` template API)
- `driver/i2s.h` comes with the ESP32 core. No install needed.
### Upload and run
 
1. Open `mos_detector.ino` in Arduino IDE.
2. Select the board and COM port, then upload.
3. Open the Serial Monitor at **115200 baud**.
4. Wait about 5 seconds while the radar warms up (`Boot complete. Commencing scan.`).
### Serial output (example)
 
```
Radar booting up. Please wait 5 seconds...
Boot complete. Commencing scan.
Hearing Peak: 187.5 Hz
Scanning Right. New boundary: 142° | Tilt: 91°
[STATE] --> ALERT (Yellow - Suspicious Movement)
Hearing Peak: 593.8 Hz
CONFIRMED MOSQUITO! Peak: 593.8 Hz | Energy: 124000
[STATE] --> LOCKED (Red - MOSQUITO CONFIRMED!)
```
 
State changes are printed only when the state actually changes.
 
### Testing without a real mosquito
 
Play a mosquito buzzing sound from a phone at 80–100% volume, about **10–20 cm** from the microphone. Too close can clip the mic and blur the frequency peaks.
 
---
 
## FreeRTOS design
 
All three tasks are pinned to the application core (core 1).
 
| Task | Priority | Stack | Job |
|---|---|---|---|
| `taskSensors` | 2 (highest) | 10240 | Reads the radar and I2S audio, runs the FFT, decides the state |
| `taskServos` | 1 | 2048 | Pan/tilt sweep based on the state |
| `taskFeedback` | 1 | 2048 | LEDs and buzzer based on the state |
 
Real-time techniques used:
 
- **Mutex with priority inheritance.** `globalTurretState` is shared by all tasks and protected by `xSemaphoreCreateMutex()`, which raises a low-priority holder's priority to avoid priority inversion.
- **Bounded timeouts.** Every `xSemaphoreTake()` and `i2s_read()` waits at most 100 ms, so no task can block forever (breaks "hold and wait", which prevents deadlock).
- **Short critical sections.** Tasks copy the state into a local variable and release the lock immediately.
- **Starvation prevention.** The high-priority sensor task blocks for 50 ms every loop so the priority-1 tasks get CPU time.
- **Time-slicing.** The two priority-1 tasks share the CPU through the FreeRTOS preemptive scheduler.
- `setup()` deletes itself with `vTaskDelete(NULL)` after creating the tasks, and `loop()` is empty.
---
 
## Tuning
 
| Constant | Default | Effect |
|---|---|---|
| `MOSQUITO_THRESHOLD` | `80000` | Minimum energy in the wingbeat band. Raise it if quiet noise triggers `LOCKED`, lower it if real buzzing is missed. |
| Pitch range | `420–800 Hz` | Range for the strongest frequency (`isMosquitoPitch`). |
| `ALERT_COOLDOWN` | `5000` ms | How long `ALERT` stays after the radar last saw motion. |
| Confirm frames | `4` | Frames in a row needed to lock (about 250 ms). |
| Sweep bounds | `20–70°` / `110–160°` | Random left/right limits for the pan servo. |
 
---
 
## Troubleshooting
 
| Symptom | Likely cause / fix |
|---|---|
| ESP32 keeps resetting when the servos move | Servos are powered from the ESP32, or the grounds aren't joined. Use the external adapter and a shared GND. Add a capacitor on the servo rail. |
| Servos jitter or don't move | Adapter – isn't connected to ESP32 GND. |
| Pan and tilt motions are swapped | Swap the orange wires on GPIO13 and GPIO14. |
| `Hearing Peak` stays at 0 or never changes | Check the mic wiring and that VDD is on 3V3. If it is still silent, move L/R from GND to 3V3 (some ESP32 core versions swap the I2S channels). |
| Buzzer beeps all the time in `SCANNING` | The buzzer is active-high, not active-low. Swap `HIGH` and `LOW` in the buzzer lines. |
| Stuck in `ALERT` | The radar keeps seeing you (it detects people too). Step out of range, or wait 5 s after the last motion. |
| Board won't boot after wiring | Something is connected to GPIO12. Leave it empty. |
 
---
 
## Project structure
 
```
Autonomous_Mosquito_Detector/
├── mos_detector.ino   # Firmware (FreeRTOS tasks, I2S + FFT, servo and feedback logic)
└── README.md
```
 
