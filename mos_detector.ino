#include <Arduino.h>
#include <ESP32Servo.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>

#define SAMPLES 512              // FFT sample size (power of 2 required for Radix-2 FFT)
#define SAMPLING_FREQUENCY 16000 // 16 kHz sampling rate (Nyquist bandwidth: 0 to 8 kHz)
                                 // Bin resolution = 16000 / 512 = 31.25 Hz per bin (Bin 1 = 31.25 Hz)

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// minimum energy threshold to reject background acoustic noise
const double MOSQUITO_THRESHOLD = 80000;

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;  // Pin real-time tasks to Core 1
#endif

static const int PAN_PIN       = 13;
static const int TILT_PIN      = 14;
static const int RADAR_OUT     = 18;
static const int LED_R         = 27;
static const int LED_Y         = 26;
static const int LED_G         = 23;
static const int BUZZER_PIN    = 21;

// I2S Bus Connections (DMA) for Mic
static const int I2S_WS        = 25;
static const int I2S_SCK       = 33;
static const int I2S_SD        = 32;

enum TurretState { SCANNING, ALERT, LOCKED };

// Shared Resource across multiple tasks
TurretState globalTurretState = SCANNING; 
TurretState lastLoggedState = SCANNING;

// Synchronization - Mutex with native Priority Inheritance Protocol (PIP)
// Automatically mitigates Priority Inversion if a lower-priority task holds the lock.
SemaphoreHandle_t stateMutex;        

Servo panServo;
Servo tiltServo;

// Task 1: sensor array & signal processing
// Priority: 2 (Highest - Strict real-time deadline for acoustic DSP (Digital Signal Processing))
void taskSensors(void *parameter) {
  Serial.println("Radar booting up. Please wait 5 seconds...");
  vTaskDelay(pdMS_TO_TICKS(5000));
  Serial.println("Boot complete. Commencing scan.");

  size_t bytesIn = 0;
  int32_t i2sBuffer[SAMPLES]; 

  int consecutiveMosquitoReads = 0;
  unsigned long lastMotionTime = 0;                // Software memory timer for radar debouncing
  const unsigned long ALERT_COOLDOWN = 5000;      // 5-second hysteresis/memory window to prevent rapid state flipping

  while (1) {
    // mmWave Radar Read with Software Hysteresis
    bool rawRadar = (digitalRead(RADAR_OUT) == HIGH);
    if (rawRadar) {
        lastMotionTime = millis();
    }
    bool radarDetected = (millis() - lastMotionTime < ALERT_COOLDOWN) && (lastMotionTime != 0);
    bool mosquitoDetected = false;

    // I2S DMA Audio Ingestion
    // Bounded wait time prevents permanent lockup if I2S DMA hangs
    esp_err_t result = i2s_read(I2S_NUM_0, &i2sBuffer, sizeof(i2sBuffer), &bytesIn, pdMS_TO_TICKS(100));
    if (result == ESP_OK) {
      int samplesRead = bytesIn / sizeof(int32_t);

      // DC Offset Removal (Bin 0 = 0 Hz)
      double mean = 0;
      for (int i = 0; i < samplesRead; i++) {
        mean += (double)(i2sBuffer[i] >> 14);
      }
      mean /= samplesRead;
      
      for (int i = 0; i < SAMPLES; i++) {
        if (i < samplesRead) {
          vReal[i] = (double)(i2sBuffer[i] >> 14) - mean;
        } else {
          vReal[i] = 0.0;
        }
        vImag[i] = 0.0;
      }

      // Fast Fourier Transform (FFT) Analysis
      FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); 
      FFT.compute(FFTDirection::Forward);
      FFT.complexToMagnitude();
      
      // for testing the sound frequency
      double dominantFreq = FFT.majorPeak();

      Serial.printf("Hearing Peak: %.1f Hz\n", dominantFreq);

      // Fundamental Band Extraction (Bins 13 to 26: ~406 Hz - 812 Hz)  (13 * 31.25 Hz to 26 * 31.25 Hz)
      double mosquitoEnergy = 0;
      double maxMosquitoBin = 0;
      for (int i = 13; i <= 26; i++) {
        mosquitoEnergy += vReal[i];
        if (vReal[i] > maxMosquitoBin) {
          maxMosquitoBin = vReal[i]; // Peak narrow-band power detection
        }
      }

      // Harmonic Band Extraction (Bins 29 to 51: ~906 Hz - 1593 Hz)  (29 * 31.25 Hz to 51 * 31.25 Hz)
      double maxHarmonicBin = 0;
      for (int i = 29; i <= 51; i++) {
        if (vReal[i] > maxHarmonicBin) {
          maxHarmonicBin = vReal[i]; // Second harmonic overtone detection
        }
      }
      
      // Dynamic Ambient Noise Floor Estimation (Exclude target bands)
      double noiseFloor = 0;
      int noiseBins = 0;
      for (int i = 2; i < (SAMPLES / 2); i++) {
        if (i < 13 || i > 51) {
          noiseFloor += vReal[i];
          noiseBins++;
        }
      }
      noiseFloor = noiseFloor / noiseBins; // average ambient noise

      bool isMosquitoPitch = (dominantFreq >= 420.0 && dominantFreq <= 800.0);
      
      // Narrowband Classification & Impulse Filtering
      // Rejects broadband impulses (hand claps) where noiseFloor rises equally
      if (isMosquitoPitch && maxMosquitoBin > (noiseFloor * 4.0) && mosquitoEnergy > MOSQUITO_THRESHOLD) {   
        if (maxHarmonicBin > (noiseFloor * 2.0)) {
            consecutiveMosquitoReads++; // Verified fundamental + harmonic signature
        } else {
            consecutiveMosquitoReads = 0; // Failed the harmonic check. Reset to 0.
        }
      } else {
        consecutiveMosquitoReads = 0; // Immediate reset prevents servo noise accumulation
      }

      // Temporal Integration: Require 4 sustained frames (~250 ms) to confirm target
      if (consecutiveMosquitoReads >= 4) {
        mosquitoDetected = true;
        Serial.printf("CONFIRMED MOSQUITO! Peak: %.1f Hz | Energy: %.0f\n", dominantFreq, mosquitoEnergy);
      }
      
    }

    // Critical Section State Update
    // Bounded timeout (100 ms) breaks indefinite "Hold and Wait" to prevent Deadlock
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {

      if (mosquitoDetected) {
          globalTurretState = LOCKED;
      }
      else if (radarDetected) {
          globalTurretState = ALERT;
      } 
      else {
          globalTurretState = SCANNING;
      }

      // Non-blocking serial logging (only executes on state transitions)
      if (globalTurretState != lastLoggedState) {
          lastLoggedState = globalTurretState;
          if (globalTurretState == SCANNING) Serial.println("[STATE] --> SCANNING (Green)");
          if (globalTurretState == ALERT)    Serial.println("[STATE] --> ALERT (Yellow - Suspicious Movement)");
          if (globalTurretState == LOCKED)   Serial.println("[STATE] --> LOCKED (Red - MOSQUITO CONFIRMED!)");
      }

      xSemaphoreGive(stateMutex);       // Release lock immediately
    }

    // Starvation Prevention
    // High-priority task voluntarily enters Blocked state for 50 ms to yield CPU to Priority 1 tasks
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Task 2: Servo Sweep Engine
// Priority: 1 (Shares CPU with Task 3 via FreeRTOS Preemptive Time-Slicing)
void taskServos(void *parameter) {
  int panPos = 90;
  int tiltPos = 90;
  int panDirection = 1;

  int leftBound = 45;
  int rightBound = 135; 

  while (1) {
    TurretState localState = SCANNING;

    // Read shared state within a minimal critical section, then release
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      localState = globalTurretState;
      xSemaphoreGive(stateMutex);
    }

    // Actuation Logic based on local snapshot
    if (localState == SCANNING || localState == ALERT) {
      panPos += panDirection;
      panServo.write(panPos);
      tiltServo.write(tiltPos);

      // Boundary calculation with randomized limits for natural sweeping behavior
      if (panPos >= rightBound || panPos <= leftBound) {
        panDirection *= -1;
        
        if (panDirection == 1) {
          rightBound = random(110, 160);
          Serial.printf("Scanning Right. New boundary: %d° | Tilt: %d°\n", rightBound, tiltPos);
        } else {
          leftBound = random(20, 70);
          Serial.printf("Scanning Left. New boundary: %d° | Tilt: %d°\n", leftBound, tiltPos);
        }

        tiltPos = random(80, 100);
        vTaskDelay(pdMS_TO_TICKS(random(300, 1200)));
      }
      
      // Organic Area Inspection (2% probability pause)
      if (random(0, 100) > 97) {
          Serial.println("Turret is inspecting an area...");
          vTaskDelay(pdMS_TO_TICKS(random(1000, 3000))); // Pause for 1 to 3 seconds
      } else {
          if (localState == ALERT) {
              vTaskDelay(pdMS_TO_TICKS(60)); // Slower search speed in ALERT state
          } else {
              vTaskDelay(pdMS_TO_TICKS(30)); // Normal speed when clear area in SCANNING state
          }
      }
    } else {
        // Hold servo position and yield CPU cycles in LOCKED state
        vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

// Task 3: visual & audio feedback
// Priority: 1 (Shares CPU with Task 2 via FreeRTOS Preemptive Time-Slicing)
void taskFeedback(void *parameter) {
  digitalWrite(BUZZER_PIN, HIGH); // Active-low buzzer off

  while (1) {
    TurretState localState = SCANNING;

    // Minimal Critical Section
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      localState = globalTurretState;
      xSemaphoreGive(stateMutex);
    }

    // Hardware State Indication
    switch (localState) {
      case SCANNING:
        digitalWrite(LED_G, HIGH);
        digitalWrite(LED_Y, LOW);
        digitalWrite(LED_R, LOW);
        digitalWrite(BUZZER_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(200));
        break;

      case ALERT:
        digitalWrite(LED_G, LOW);
        digitalWrite(LED_Y, HIGH);
        digitalWrite(LED_R, LOW);
        digitalWrite(BUZZER_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(200));
        break;

      case LOCKED:
        digitalWrite(LED_G, LOW);
        digitalWrite(LED_Y, LOW);
        digitalWrite(LED_R, HIGH);
        
        // Target engagement acoustic alert pattern
        digitalWrite(BUZZER_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(BUZZER_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(400));
        break;
    }
  }
}

// System setup & Task scheduling
void setup() {
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Initialize Standard GPIO
  pinMode(RADAR_OUT, INPUT_PULLDOWN);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  
  // Initialize Servos
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  // Initialize I2S Controller for INMP441 Microphone
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  // Mutex creation with Priority Inheritance enabled by default
  stateMutex = xSemaphoreCreateMutex();

  if (stateMutex != NULL) {
    // Priority Assignment based on Rate-Monotonic / Real-Time Urgency
    // taskSensors (Priority=2) preempts mechanical and feedback tasks (Priority=1)
    xTaskCreatePinnedToCore(taskSensors,  "Sensors",  10240, NULL, 2, NULL, app_cpu);
    xTaskCreatePinnedToCore(taskFeedback, "Feedback", 2048, NULL, 1, NULL, app_cpu);
    xTaskCreatePinnedToCore(taskServos,   "Servos",   2048, NULL, 1, NULL, app_cpu);
  } else {
    Serial.println("Mutex creation failed! System halted.");
    while(1); 
  }

  // Terminate setup() task to let FreeRTOS scheduler take over
  vTaskDelete(NULL); 
}

void loop() {}