/*
  ===================================================================================
  Project  : A Low-Cost IoT-Enabled Bridge Structural Health Monitoring (SHM) System
  Author   : MD. Rakib Hassan Dipu
  Dept.    : Department of IoT and Robotics Engineering
             University of Frontier Technology, Bangladesh
  Board    : ESP32-S3 Dev Module
  Sensors  : MPU9250 (9-DOF IMU) + SW-420 (Piezo/Spring Vibration Sensor)
  Features : Hardware-timed 200 Hz sampling, Baseline calibration, 1024-point FFT,
             Dominant Frequency Peak Extraction, RMS & Peak Energy calculation,
             Tilt angle computation, and Multi-parameter Damage Assessment Logic.
  ===================================================================================
  Pin Configuration:
    - MPU9250 SDA  -> GPIO 41
    - MPU9250 SCL  -> GPIO 42
    - SW-420 DOUT  -> GPIO 47
    - VCC          -> 3.3V
    - GND          -> GND
  ===================================================================================
*/

#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <arduinoFFT.h>

// ==================== USER CONFIGURATION ====================
// 0 = Full formatted condition monitoring report (default)
// 1 = Serial Plotter mode (prints pure FFT magnitude spectrum)
#define SERIAL_PLOTTER   0

// ==================== PIN DEFINITIONS ====================
#define SDA_PIN         41
#define SCL_PIN         42
#define MPU_INT_PIN     21      // Optional hardware interrupt
#define VIB_PIN         47      // SW-420 digital trigger input

// ==================== CONSTANTS & THRESHOLDS ====================
const uint16_t BASELINE_SAMPLES   = 1000;   // Number of samples for initial baseline calibration
const uint16_t FFT_SAMPLES        = 1024;   // FFT buffer size (must be power of 2)
const double   SAMPLING_FREQUENCY = 200.0;  // Sampling frequency in Hz (5 ms interval)
const uint32_t SAMPLE_INTERVAL_US = (uint32_t)(1000000.0 / SAMPLING_FREQUENCY); // 5000 µs

// Structural Damage Criteria Thresholds
const float DAMAGE_FREQ_DROP_PERCENT = 5.0f;   // Alert if dominant frequency drops > 5.0%
const float DAMAGE_RMS_RISE_PERCENT  = 30.0f;  // Alert if RMS acceleration increases > 30.0%
const float DAMAGE_TILT_DEG          = 2.0f;   // Alert if inclination exceeds ±2.0 degrees

const uint32_t FFT_INTERVAL_MS       = 1000;   // Compute FFT once every 1 second

// ==================== GLOBAL OBJECTS ====================
MPU9250_asukiaaa mySensor;
arduinoFFT FFT = arduinoFFT();

// FFT Working Buffers
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
static double baselineBuf[BASELINE_SAMPLES];

// ==================== SYSTEM STATE VARIABLES ====================
float baselineRMS          = 0.0f;
float baselinePeak         = 0.0f;
float baselineDominantFreq = 0.0f;

float currentRMS        = 0.0f;
float currentPeak       = 0.0f;
float currentMax        = 0.0f;
float currentMin        = 0.0f;
float currentAvg        = 0.0f;
float dominantFrequency = 0.0f;
float peakMagnitude     = 0.0f;
float freqShiftPercent  = 0.0f;

float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float roll = 0, pitch = 0;

bool     damageDetected     = false;
bool     vibrationDetected  = false;
uint32_t lastFFTMillis      = 0;

// ==================== FUNCTION PROTOTYPES ====================
void  initializeMPU();
void  calibrateBaseline();
void  readMPU();
float calculateRMS(double* data, uint16_t n);
float calculatePeak(double* data, uint16_t n, float &maxVal, float &minVal, float &avgVal);
void  calculateTilt();
void  collectSamples(double* buffer, uint16_t n);
void  performFFT(double* real, double* imag, uint16_t n);
float calculateDominantFrequency(double* real, uint16_t n, float &magnitude);
void  compareWithBaseline();
void  printHealthy();
void  printDamage();
void  printFFT(double* real, uint16_t n);

// =====================================================================
//                                SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 3000) {
    // Wait for serial enumeration
  }

  pinMode(VIB_PIN, INPUT);

  Serial.println();
  Serial.println(F("====================================================="));
  Serial.println(F("  Bridge Structural Health Monitoring (SHM) System   "));
  Serial.println(F("  ESP32-S3 Edge Vibration & FFT Analyzer Initialized "));
  Serial.println(F("====================================================="));

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400 kHz Fast-mode I2C

  initializeMPU();

  Serial.println(F("[SYSTEM] Waiting 5 seconds for bridge stabilization before baseline calibration..."));
  uint32_t startWait = millis();
  while (millis() - startWait < 5000) {
    // Non-blocking wait
  }

  calibrateBaseline();
}

// =====================================================================
//                                LOOP
// =====================================================================
void loop() {
  readMPU();
  calculateTilt();

  // Read auxiliary vibration sensor
  vibrationDetected = (digitalRead(VIB_PIN) == HIGH);
  if (vibrationDetected) {
    Serial.println(F("[EVENT] Vibration shock triggered (SW-420)"));
  }

  // Periodic FFT Processing
  if (millis() - lastFFTMillis >= FFT_INTERVAL_MS) {
    lastFFTMillis = millis();

    // 1. Collect 1024 Z-axis acceleration samples at 200 Hz
    collectSamples(vReal, FFT_SAMPLES);
    for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
      vImag[i] = 0.0;
    }

    // 2. Time-domain statistical feature extraction
    currentRMS  = calculateRMS(vReal, FFT_SAMPLES);
    currentPeak = calculatePeak(vReal, FFT_SAMPLES, currentMax, currentMin, currentAvg);

    // 3. Frequency-domain signal processing via FFT
    performFFT(vReal, vImag, FFT_SAMPLES);
    dominantFrequency = calculateDominantFrequency(vReal, FFT_SAMPLES, peakMagnitude);

    // 4. Multi-parameter Condition Assessment Logic
    compareWithBaseline();

    // 5. Output Results
    if (SERIAL_PLOTTER) {
      printFFT(vReal, FFT_SAMPLES);
    } else {
      if (damageDetected) {
        printDamage();
      } else {
        printHealthy();
      }
    }
  }
}

// =====================================================================
//                          SENSOR INITIALIZATION
// =====================================================================
void initializeMPU() {
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();

  Serial.println(F("[INIT] MPU9250 9-DOF IMU Ready"));
  Serial.println(F("[INIT] SW-420 Vibration Sensor Ready"));
}

// =====================================================================
//                          BASELINE CALIBRATION
// =====================================================================
void calibrateBaseline() {
  Serial.println(F("[CALIB] Recording nominal baseline parameters..."));

  // Step 1: RMS & Peak from 1000 raw samples
  collectSamples(baselineBuf, BASELINE_SAMPLES);
  baselineRMS  = calculateRMS(baselineBuf, BASELINE_SAMPLES);
  baselinePeak = calculatePeak(baselineBuf, BASELINE_SAMPLES, currentMax, currentMin, currentAvg);

  // Step 2: Baseline Dominant Frequency via 1024-point FFT
  collectSamples(vReal, FFT_SAMPLES);
  for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
    vImag[i] = 0.0;
  }
  performFFT(vReal, vImag, FFT_SAMPLES);
  float baseMag = 0.0f;
  baselineDominantFreq = calculateDominantFrequency(vReal, FFT_SAMPLES, baseMag);

  Serial.println(F("================ BASELINE CALIBRATION ================"));
  Serial.print(F("Baseline RMS                 : ")); Serial.print(baselineRMS, 4);  Serial.println(F(" g"));
  Serial.print(F("Baseline Peak                : ")); Serial.print(baselinePeak, 4); Serial.println(F(" g"));
  Serial.print(F("Baseline Dominant Frequency   : ")); Serial.print(baselineDominantFreq, 2); Serial.println(F(" Hz"));
  Serial.println(F("======================================================"));
}

// =====================================================================
//                          READ MPU9250
// =====================================================================
void readMPU() {
  mySensor.accelUpdate();
  accelX = mySensor.accelX();
  accelY = mySensor.accelY();
  accelZ = mySensor.accelZ();

  mySensor.gyroUpdate();
  gyroX = mySensor.gyroX();
  gyroY = mySensor.gyroY();
  gyroZ = mySensor.gyroZ();
}

// =====================================================================
//                          TILT (ROLL / PITCH)
// =====================================================================
void calculateTilt() {
  roll  = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
}

// =====================================================================
//                          SAMPLE COLLECTION @ 200 Hz
// =====================================================================
void collectSamples(double* buffer, uint16_t n) {
  uint32_t nextSampleTime = micros();

  for (uint16_t i = 0; i < n; i++) {
    while ((int32_t)(micros() - nextSampleTime) < 0) {
      // Hardware-timed microsecond polling
    }
    nextSampleTime += SAMPLE_INTERVAL_US;

    mySensor.accelUpdate();
    buffer[i] = (double)mySensor.accelZ();
  }
}

// =====================================================================
//                          RMS CALCULATION
// =====================================================================
float calculateRMS(double* data, uint16_t n) {
  double sumSq = 0.0;
  for (uint16_t i = 0; i < n; i++) {
    sumSq += data[i] * data[i];
  }
  return (float)sqrt(sumSq / n);
}

// =====================================================================
//                          PEAK CALCULATION
// =====================================================================
float calculatePeak(double* data, uint16_t n, float &maxVal, float &minVal, float &avgVal) {
  double mx = data[0];
  double mn = data[0];
  double sum = 0.0;

  for (uint16_t i = 0; i < n; i++) {
    if (data[i] > mx) mx = data[i];
    if (data[i] < mn) mn = data[i];
    sum += data[i];
  }

  maxVal = (float)mx;
  minVal = (float)mn;
  avgVal = (float)(sum / n);

  double peak = fabs(mx) > fabs(mn) ? fabs(mx) : fabs(mn);
  return (float)peak;
}

// =====================================================================
//                          FFT COMPUTATION
// =====================================================================
void performFFT(double* real, double* imag, uint16_t n) {
  FFT.Windowing(real, n, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(real, imag, n, FFT_FORWARD);
  FFT.ComplexToMagnitude(real, imag, n);
}

// =====================================================================
//                          DOMINANT FREQUENCY EXTRACTION
// =====================================================================
float calculateDominantFrequency(double* real, uint16_t n, float &magnitude) {
  uint16_t peakIndex = 1; // Skip DC component (bin 0)
  double peakVal = real[1];

  for (uint16_t i = 2; i < (n / 2); i++) {
    if (real[i] > peakVal) {
      peakVal = real[i];
      peakIndex = i;
    }
  }

  magnitude = (float)peakVal;
  float freq = (float)peakIndex * ((float)SAMPLING_FREQUENCY / (float)n);
  return freq;
}

// =====================================================================
//                          CONDITION ASSESSMENT LOGIC
// =====================================================================
void compareWithBaseline() {
  freqShiftPercent = ((baselineDominantFreq - dominantFrequency) / baselineDominantFreq) * 100.0f;
  float rmsRisePercent = ((currentRMS - baselineRMS) / baselineRMS) * 100.0f;

  bool freqDrop   = freqShiftPercent > DAMAGE_FREQ_DROP_PERCENT;
  bool rmsRise    = rmsRisePercent > DAMAGE_RMS_RISE_PERCENT;
  bool tiltExceed = (fabs(roll) > DAMAGE_TILT_DEG) || (fabs(pitch) > DAMAGE_TILT_DEG);

  damageDetected = freqDrop || rmsRise || tiltExceed;
}

// =====================================================================
//                          SERIAL REPORT: HEALTHY
// =====================================================================
void printHealthy() {
  Serial.println(F("========================="));
  Serial.println(F("Bridge Status : HEALTHY"));
  Serial.print(F("Dominant Frequency : ")); Serial.print(dominantFrequency, 2); Serial.println(F(" Hz"));
  Serial.print(F("Baseline Frequency : ")); Serial.print(baselineDominantFreq, 2); Serial.println(F(" Hz"));
  Serial.print(F("Frequency Shift : "));    Serial.print(freqShiftPercent, 1);    Serial.println(F(" %"));
  Serial.print(F("RMS : "));   Serial.print(currentRMS, 3);  Serial.println(F(" g"));
  Serial.print(F("Peak : "));  Serial.print(currentPeak, 3); Serial.println(F(" g"));
  Serial.print(F("Roll : "));  Serial.print(roll, 1);  Serial.println(F(" deg"));
  Serial.print(F("Pitch : ")); Serial.print(pitch, 1); Serial.println(F(" deg"));
  Serial.print(F("Vibration : ")); Serial.println(vibrationDetected ? F("DETECTED") : F("NORMAL"));
  Serial.println(F("========================="));
}

// =====================================================================
//                          SERIAL REPORT: DAMAGE ALERT
// =====================================================================
void printDamage() {
  Serial.println(F("========================="));
  Serial.println(F("Bridge Status : POSSIBLE DAMAGE"));
  Serial.print(F("Dominant Frequency : ")); Serial.print(dominantFrequency, 2); Serial.println(F(" Hz"));
  Serial.print(F("Baseline Frequency : ")); Serial.print(baselineDominantFreq, 2); Serial.println(F(" Hz"));
  Serial.print(F("Frequency Shift : "));    Serial.print(freqShiftPercent, 1);    Serial.println(F(" %"));
  Serial.print(F("RMS : "));   Serial.print(currentRMS, 3);  Serial.println(F(" g"));
  Serial.print(F("Peak : "));  Serial.print(currentPeak, 3); Serial.println(F(" g"));
  Serial.print(F("Roll : "));  Serial.print(roll, 1);  Serial.println(F(" deg"));
  Serial.print(F("Pitch : ")); Serial.print(pitch, 1); Serial.println(F(" deg"));
  Serial.println(F("ALERT GENERATED"));
  Serial.println(F("========================="));
}

// =====================================================================
//                          SERIAL PLOTTER OUTPUT
// =====================================================================
void printFFT(double* real, uint16_t n) {
  for (uint16_t i = 1; i < (n / 2); i++) {
    Serial.println(real[i], 4);
  }
}
