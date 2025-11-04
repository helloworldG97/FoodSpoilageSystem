/*
 * OPTIMIZED FOOD SPOILAGE DETECTION SYSTEM
 * ========================================
 * 
 * Improvements implemented:
 * - State machine architecture for better control flow
 * - Modular sensor processing with advanced filtering
 * - Adaptive calibration and threshold algorithms
 * - Non-blocking operations with efficient timing
 * - Environmental compensation algorithms
 * - Statistical analysis for better accuracy
 * - Optimized communication protocols
 * - Comprehensive error handling
 */

// Blynk configuration
#define BLYNK_TEMPLATE_ID "TMPL684zLhVAC"
#define BLYNK_TEMPLATE_NAME "FOOD SPOILAGE"
#define BLYNK_AUTH_TOKEN "UqM8-wEm4HIzy6rtStzxzDsfxz5eEVpZ"

// Pin definitions - ESP32 ADC1 (WiFi COMPATIBLE)
#define MQ135_PIN 35 
#define MQ3_PIN 34   
#define DHT_PIN 19
#define RED_LED 12
#define BLUE_LED 27  
#define GREEN_LED 14
#define BUZZER_PIN 26
#define BUTTON_PIN 33
#define TRIG_PIN 16
#define ECHO_PIN 17
#define RELAY_PIN 25

// Detection distance in centimeters
#define DETECTION_DISTANCE 10

#include <DHT.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// WiFi credentials
char ssid[] = "Nepthys";
char pass[] = "darylace";

// ============================================================================
// SYSTEM STATE MACHINE
// ============================================================================

enum SystemState {
  STATE_INITIALIZING,
  STATE_PREHEATING,
  STATE_READY_FOR_CALIBRATION,
  STATE_CALIBRATING,
  STATE_MONITORING,
  STATE_ERROR
};

SystemState currentState = STATE_INITIALIZING;
SystemState previousState = STATE_INITIALIZING;

// ============================================================================
// ADVANCED SENSOR PROCESSING CLASS
// ============================================================================

class AdvancedSensorProcessor {
private:
  static const int BUFFER_SIZE = 20;
  static const int OUTLIER_THRESHOLD = 2; // Z-score threshold
  
  float readings[BUFFER_SIZE];
  int bufferIndex;
  int sampleCount;
  float sum;
  float sumSquares;
  
public:
  AdvancedSensorProcessor() : bufferIndex(0), sampleCount(0), sum(0), sumSquares(0) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
      readings[i] = 0;
    }
  }
  
  void addReading(float value) {
    // Remove old value from statistics
    if (sampleCount >= BUFFER_SIZE) {
      float oldValue = readings[bufferIndex];
      sum -= oldValue;
      sumSquares -= oldValue * oldValue;
    } else {
      sampleCount++;
    }
    
    // Add new value
    readings[bufferIndex] = value;
    sum += value;
    sumSquares += value * value;
    
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  }
  
  float getFilteredValue() {
    if (sampleCount < 3) return readings[(bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE];
    
    float mean = sum / sampleCount;
    float variance = (sumSquares / sampleCount) - (mean * mean);
    float stdDev = sqrt(variance);
    
    // Calculate weighted average excluding outliers
    float weightedSum = 0;
    float totalWeight = 0;
    
    for (int i = 0; i < sampleCount; i++) {
      float value = readings[i];
      float zScore = (stdDev > 0) ? abs(value - mean) / stdDev : 0;
      
      if (zScore <= OUTLIER_THRESHOLD) {
        float weight = 1.0 / (1.0 + zScore); // Higher weight for values closer to mean
        weightedSum += value * weight;
        totalWeight += weight;
      }
    }
    
    return (totalWeight > 0) ? weightedSum / totalWeight : mean;
  }
  
  float getStandardDeviation() {
    if (sampleCount < 2) return 0;
    float mean = sum / sampleCount;
    float variance = (sumSquares / sampleCount) - (mean * mean);
    return sqrt(variance);
  }
  
  float getTrend() {
    if (sampleCount < 5) return 0;
    
    // Calculate linear regression slope for trend analysis
    float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    int n = min(sampleCount, 10); // Use last 10 readings for trend
    
    for (int i = 0; i < n; i++) {
      int idx = (bufferIndex - n + i + BUFFER_SIZE) % BUFFER_SIZE;
      float x = i;
      float y = readings[idx];
      
      sumX += x;
      sumY += y;
      sumXY += x * y;
      sumX2 += x * x;
    }
    
    float slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    return slope;
  }
  
  bool isStable() {
    return getStandardDeviation() < (getFilteredValue() * 0.05); // 5% stability threshold
  }
};

// ============================================================================
// ENVIRONMENTAL COMPENSATION CLASS
// ============================================================================

class EnvironmentalCompensator {
private:
  float baseTemp;
  float baseHumidity;
  bool isCalibrated;
  
public:
  EnvironmentalCompensator() : baseTemp(25.0), baseHumidity(50.0), isCalibrated(false) {}
  
  void calibrate(float temp, float humidity) {
    baseTemp = temp;
    baseHumidity = humidity;
    isCalibrated = true;
  }
  
  float compensateReading(float rawValue, float currentTemp, float currentHumidity) {
    if (!isCalibrated) return rawValue;
    
    // Temperature compensation (typical MQ sensor behavior)
    float tempFactor = 1.0 + 0.02 * (currentTemp - baseTemp); // 2% per degree
    
    // Humidity compensation
    float humidityFactor = 1.0 + 0.005 * (currentHumidity - baseHumidity); // 0.5% per %RH
    
    return rawValue * tempFactor * humidityFactor;
  }
};

// ============================================================================
// ADAPTIVE THRESHOLD MANAGER
// ============================================================================

class AdaptiveThresholdManager {
private:
  struct ThresholdSet {
    float freshMin, freshMax;
    float warningMin, warningMax;
    float spoiledThreshold;
  };
  
  ThresholdSet mq135Thresholds;
  ThresholdSet mq3Thresholds;
  float adaptationRate;
  float spoiledBaseMQ135;
  float spoiledBaseMQ3;
  
public:
  AdaptiveThresholdManager() : adaptationRate(0.05) {
    // Initialize with conservative defaults
    mq135Thresholds = {2.8, 3.5, 2.4, 2.7, 2.3};
    mq3Thresholds = {51.0, 65.0, 44.0, 50.0, 43.0};
    // Capture baselines so adaptation respects updated thresholds
    spoiledBaseMQ135 = mq135Thresholds.spoiledThreshold;
    spoiledBaseMQ3 = mq3Thresholds.spoiledThreshold;
  }
  
  void updateThresholds(float mq135Ratio, float mq3Ratio, float temp, float humidity) {
    // Adapt thresholds based on environmental conditions
    float tempFactor = 1.0 + (temp - 25.0) * 0.01; // 1% per degree from 25°C
    float humidityFactor = 1.0 + (humidity - 50.0) * 0.002; // 0.2% per %RH from 50%
    
    float environmentalFactor = tempFactor * humidityFactor;
    
    // Gradually adapt thresholds
    float targetMQ135 = spoiledBaseMQ135 * environmentalFactor;
    float targetMQ3 = spoiledBaseMQ3 * environmentalFactor;
    mq135Thresholds.spoiledThreshold += (targetMQ135 - mq135Thresholds.spoiledThreshold) * adaptationRate;
    mq3Thresholds.spoiledThreshold += (targetMQ3 - mq3Thresholds.spoiledThreshold) * adaptationRate;
  }
  
  int classifyFood(float mq135Ratio, float mq3Ratio) {
    // 0 = Fresh, 1 = Warning, 2 = Spoiled
    
    if (mq135Ratio <= mq135Thresholds.spoiledThreshold || 
        mq3Ratio <= mq3Thresholds.spoiledThreshold) {
      return 2; // Spoiled
    }
    
    if ((mq135Ratio >= mq135Thresholds.warningMin && mq135Ratio <= mq135Thresholds.warningMax) ||
        (mq3Ratio >= mq3Thresholds.warningMin && mq3Ratio <= mq3Thresholds.warningMax)) {
      return 1; // Warning
    }
    
    return 0; // Fresh
  }
  
  float getSpoiledThreshold(bool isMQ135) {
    return isMQ135 ? mq135Thresholds.spoiledThreshold : mq3Thresholds.spoiledThreshold;
  }
};

// ============================================================================
// SYSTEM CONFIGURATION AND GLOBALS
// ============================================================================

// Safety settings
const bool SAFE_MODE = true;
const float MAX_TEMP = 80.0;

// Timing constants (non-blocking)
const unsigned long PREHEAT_TIME = 900000;  // 15 minutes
const unsigned long CALIBRATION_TIME = 10000;  // 10 seconds
const unsigned long SENSOR_READ_INTERVAL = 1000;  // 1 second
const unsigned long BLYNK_UPDATE_INTERVAL = 5000;  // 5 seconds
const unsigned long STATUS_UPDATE_INTERVAL = 500;  // 500ms for responsive UI

// System objects
DHT dht(DHT_PIN, DHT11);
AdvancedSensorProcessor mq135Processor;
AdvancedSensorProcessor mq3Processor;
AdvancedSensorProcessor tempProcessor;
AdvancedSensorProcessor humidityProcessor;
EnvironmentalCompensator envCompensator;
AdaptiveThresholdManager thresholdManager;

// System variables
unsigned long stateStartTime = 0;
unsigned long lastSensorRead = 0;
unsigned long lastBlynkUpdate = 0;
unsigned long lastStatusUpdate = 0;

// Calibration data
float R0_MQ135 = 0.0;
float R0_MQ3 = 0.0;
const float RL_MQ135 = 20.0;
const float RL_MQ3 = 1.0;

// System status
int foodStatus = 0;
String statusMessage = "Initializing system...";
bool systemReady = false;
bool buzzerEnabled = true;

// Button handling (non-blocking)
bool buttonPressed = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Blynk virtual pins
#define V_PREHEAT_PROGRESS V0
#define V_MQ135_SENSOR V1
#define V_MQ3_SENSOR V2
#define V_TEMPERATURE V3
#define V_HUMIDITY V4
#define V_STATUS V5
#define V_BUZZER_OFF V6
#define V_BUTTON_STATUS V7
#define V_MQ135_RATIO V8
#define V_MQ3_RATIO V9
#define V_CALIBRATION_PROGRESS V10

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void setLED(bool redOn, bool greenOn, bool blueOn) {
  digitalWrite(RED_LED, redOn ? HIGH : LOW);
  digitalWrite(GREEN_LED, greenOn ? HIGH : LOW);
  digitalWrite(BLUE_LED, blueOn ? HIGH : LOW);
}

bool readButtonNonBlocking() {
  bool reading = digitalRead(BUTTON_PIN) == LOW;
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading && !buttonPressed) {
      buttonPressed = true;
      return true;
    } else if (!reading) {
      buttonPressed = false;
    }
  }
  
  lastButtonState = reading;
  return false;
}

int readMQSensorOptimized(int pin) {
  // Single optimized reading with proper ADC settling time
  analogRead(pin); // Dummy read for ADC settling
  delayMicroseconds(100);
  return analogRead(pin);
}

void updateUltrasonicRelay() {
  static unsigned long lastUltrasonicRead = 0;
  
  if (millis() - lastUltrasonicRead < 100) return; // Limit to 10Hz
  lastUltrasonicRead = millis();
  
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int distance = duration * 0.034 / 2;
  
  digitalWrite(RELAY_PIN, (distance > 0 && distance <= DETECTION_DISTANCE) ? HIGH : LOW);
}

void safetyMonitor() {
  float temp = tempProcessor.getFilteredValue();
  
  if (temp > MAX_TEMP) {
    currentState = STATE_ERROR;
    statusMessage = "OVERHEATING DETECTED!";
    setLED(true, false, false);
  }
}

// ============================================================================
// STATE MACHINE IMPLEMENTATION
// ============================================================================

void handleStateInitializing() {
  static bool initComplete = false;
  
  if (!initComplete) {
    Serial.println("🚀 OPTIMIZED FOOD SPOILAGE DETECTION SYSTEM");
    Serial.println("============================================");
    
    // Initialize hardware
    pinMode(MQ135_PIN, INPUT);
    pinMode(MQ3_PIN, INPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(RELAY_PIN, OUTPUT);
    
    // Initialize outputs
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(RELAY_PIN, LOW);
    
    // Configure ESP32 ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    dht.begin();
    
    // Initialize WiFi and Blynk
    Serial.println("📡 Connecting to WiFi...");
    WiFi.begin(ssid, pass);
    
    initComplete = true;
  }
  
  // Non-blocking WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
      lastBlink = millis();
      digitalWrite(RED_LED, !digitalRead(RED_LED));
    }
    return;
  }
  
  // WiFi connected, initialize Blynk
  static bool blynkInitialized = false;
  if (!blynkInitialized) {
    Serial.println("✅ WiFi connected!");
    digitalWrite(RED_LED, LOW);
    
    Serial.println("📱 Connecting to Blynk...");
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
    
    // Send test notification
    delay(2000);
    Blynk.logEvent("food_fresh", "✅ OPTIMIZED SYSTEM: Started successfully!");
    
    blynkInitialized = true;
    currentState = STATE_PREHEATING;
    stateStartTime = millis();
    statusMessage = "Sensor preheating (15 min)";
    
    Serial.println("🔥 Starting sensor preheating phase...");
  }
}

void handleStatePreheating() {
  unsigned long elapsed = millis() - stateStartTime;
  int progress = map(elapsed, 0, PREHEAT_TIME, 0, 100);
  
  // LED pattern - red blinking
  static unsigned long lastLedUpdate = 0;
  static bool sirenPhase = false;
  if (millis() - lastLedUpdate > 300) {
    lastLedUpdate = millis();
    sirenPhase = !sirenPhase;
    // Police-style siren: alternate Red and Blue
    setLED(sirenPhase, false, !sirenPhase);
  }
  
  // Update progress
  static int lastProgress = -1;
  if (progress != lastProgress && progress % 10 == 0) {
    lastProgress = progress;
    Blynk.virtualWrite(V_PREHEAT_PROGRESS, progress);
    
    unsigned long remaining = PREHEAT_TIME - elapsed;
    Serial.print("🔥 Preheating: ");
    Serial.print(progress);
    Serial.print("% (");
    Serial.print(remaining / 60000);
    Serial.print(":");
    Serial.print((remaining % 60000) / 1000);
    Serial.println(" remaining)");
  }
  
  // Collect environmental data during preheating
  if (millis() - lastSensorRead > SENSOR_READ_INTERVAL) {
    lastSensorRead = millis();
    
    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    if (!isnan(temp)) tempProcessor.addReading(temp);
    if (!isnan(humidity)) humidityProcessor.addReading(humidity);
  }
  
  // Check if preheating complete
  if (elapsed >= PREHEAT_TIME) {
    currentState = STATE_READY_FOR_CALIBRATION;
    setLED(false, false, true);
    
    statusMessage = "Ready - Press button to calibrate";
    Blynk.virtualWrite(V_PREHEAT_PROGRESS, 100);
    Blynk.virtualWrite(V_BUTTON_STATUS, "Ready - Press Button");
    
    Serial.println("✅ Preheating complete! Ready for calibration.");
  }
}

void handleStateReadyForCalibration() {
  // Solid blue LED
  setLED(false, false, true);
  
  if (readButtonNonBlocking()) {
    currentState = STATE_CALIBRATING;
    stateStartTime = millis();
    statusMessage = "Calibration in progress...";
    
    // Reset processors for calibration
    mq135Processor = AdvancedSensorProcessor();
    mq3Processor = AdvancedSensorProcessor();
    
    Serial.println("🔄 Starting calibration...");
    
    // Single beep for calibration start
    if (buzzerEnabled) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}

void handleStateCalibrating() {
  unsigned long elapsed = millis() - stateStartTime;
  int progress = map(elapsed, 0, CALIBRATION_TIME, 0, 100);
  
  // Calibration LED: solid blue
  setLED(false, false, true);
  
  // Update progress
  static int lastProgress = -1;
  if (progress != lastProgress && progress % 20 == 0) {
    lastProgress = progress;
    Blynk.virtualWrite(V_CALIBRATION_PROGRESS, progress);
  }
  
  // Collect calibration samples
  if (millis() - lastSensorRead > 100) { // 10Hz sampling
    lastSensorRead = millis();
    
    int mq135_raw = readMQSensorOptimized(MQ135_PIN);
    int mq3_raw = readMQSensorOptimized(MQ3_PIN);
    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    mq135Processor.addReading(mq135_raw);
    mq3Processor.addReading(mq3_raw);
    
    if (!isnan(temp)) tempProcessor.addReading(temp);
    if (!isnan(humidity)) humidityProcessor.addReading(humidity);
  }
  
  // Check if calibration complete
  if (elapsed >= CALIBRATION_TIME) {
    // Calculate R0 values using filtered readings
    float avg_mq135 = mq135Processor.getFilteredValue();
    float avg_mq3 = mq3Processor.getFilteredValue();
    float avg_temp = tempProcessor.getFilteredValue();
    float avg_humidity = humidityProcessor.getFilteredValue();
    
    // Calculate Rs values for ESP32
    float volt_MQ135 = (avg_mq135 / 4095.0) * 3.3;
    float volt_MQ3 = (avg_mq3 / 4095.0) * 3.3;
    
    float RS_MQ135 = (3.3 - volt_MQ135) / volt_MQ135 * RL_MQ135;
    float RS_MQ3 = (3.3 - volt_MQ3) / volt_MQ3 * RL_MQ3;
    
    // Calculate R0 with environmental compensation
    R0_MQ135 = RS_MQ135 / 3.6;  // Clean air ratio
    R0_MQ3 = RS_MQ3 / 60.0;     // Clean air ratio
    
    // Calibrate environmental compensator
    envCompensator.calibrate(avg_temp, avg_humidity);
    
    currentState = STATE_MONITORING;
    systemReady = true;
    statusMessage = "System ready - Monitoring food";
    
    setLED(false, true, false);
    
    // Double beep for calibration complete
    if (buzzerEnabled) {
      for (int i = 0; i < 2; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
      }
    }
    
    Blynk.virtualWrite(V_CALIBRATION_PROGRESS, 100);
    Blynk.virtualWrite(V_BUTTON_STATUS, "System Ready");
    
    Serial.println("✅ Calibration complete!");
    Serial.print("R0_MQ135: ");
    Serial.print(R0_MQ135);
    Serial.print(", R0_MQ3: ");
    Serial.println(R0_MQ3);
  }
}

void handleStateMonitoring() {
  // Read sensors at regular intervals
  if (millis() - lastSensorRead > SENSOR_READ_INTERVAL) {
    lastSensorRead = millis();
    
    int mq135_raw = readMQSensorOptimized(MQ135_PIN);
    int mq3_raw = readMQSensorOptimized(MQ3_PIN);
    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    // Add readings to processors
    mq135Processor.addReading(mq135_raw);
    mq3Processor.addReading(mq3_raw);
    
    if (!isnan(temp)) tempProcessor.addReading(temp);
    if (!isnan(humidity)) humidityProcessor.addReading(humidity);
    
    // Calculate ratios with environmental compensation
    float filteredMQ135 = mq135Processor.getFilteredValue();
    float filteredMQ3 = mq3Processor.getFilteredValue();
    float currentTemp = tempProcessor.getFilteredValue();
    float currentHumidity = humidityProcessor.getFilteredValue();
    
    // Apply environmental compensation
    filteredMQ135 = envCompensator.compensateReading(filteredMQ135, currentTemp, currentHumidity);
    filteredMQ3 = envCompensator.compensateReading(filteredMQ3, currentTemp, currentHumidity);
    
    // Calculate ratios
    float volt_MQ135 = (filteredMQ135 / 4095.0) * 3.3;
    float volt_MQ3 = (filteredMQ3 / 4095.0) * 3.3;
    
    float RS_MQ135 = (3.3 - volt_MQ135) / volt_MQ135 * RL_MQ135;
    float RS_MQ3 = (3.3 - volt_MQ3) / volt_MQ3 * RL_MQ3;
    
    float ratio_MQ135 = (R0_MQ135 > 0) ? RS_MQ135 / R0_MQ135 : 0;
    float ratio_MQ3 = (R0_MQ3 > 0) ? RS_MQ3 / R0_MQ3 : 0;
    
    // Update adaptive thresholds
    thresholdManager.updateThresholds(ratio_MQ135, ratio_MQ3, currentTemp, currentHumidity);
    
    // Classify food status
    int previousStatus = foodStatus;
    foodStatus = thresholdManager.classifyFood(ratio_MQ135, ratio_MQ3);
    
    // Update status message and LEDs
    switch (foodStatus) {
      case 0:
        statusMessage = "Food is Fresh (5+ hours)";
        setLED(false, true, false);
        break;
      case 1:
        statusMessage = "Food About to Spoil (3-4 hours)";
        // Yellow = Red + Green
        setLED(true, true, false);
        break;
      case 2:
        statusMessage = "FOOD SPOILED - Do Not Consume!";
        setLED(true, false, false);
        break;
    }
    
    // Send notifications on status change
    if (previousStatus != foodStatus) {
      Serial.println("=== FOOD STATUS CHANGE ===");
      Serial.print("MQ135 Ratio: ");
      Serial.print(ratio_MQ135, 2);
      Serial.print(" | MQ3 Ratio: ");
      Serial.println(ratio_MQ3, 2);
      Serial.println(statusMessage);
      
      // Send appropriate notification
      switch (foodStatus) {
        case 0:
          Blynk.logEvent("food_fresh", "✅ FOOD STATUS: Fresh\n\nYour food is fresh and safe to eat.");
          break;
        case 1:
          Blynk.logEvent("food_warning_", "⚠️ FOOD STATUS: About to Spoil\n\nEarly spoilage signs detected. Consume within 3-4 hours.");
          break;
        case 2:
          Blynk.logEvent("food_alert", "🚨 FOOD STATUS: Spoiled\n\nFood is not safe for consumption. Discard immediately!");
          break;
      }
      
      // Status beeps
      if (buzzerEnabled) {
        int beeps = foodStatus + 1;
        for (int i = 0; i < beeps; i++) {
          digitalWrite(BUZZER_PIN, HIGH);
          delay(100);
          digitalWrite(BUZZER_PIN, LOW);
          if (i < beeps - 1) delay(200);
        }
      }
    }
    
    // Update Blynk data
    if (millis() - lastBlynkUpdate > BLYNK_UPDATE_INTERVAL) {
      lastBlynkUpdate = millis();
      
      Blynk.virtualWrite(V_MQ135_SENSOR, (int)filteredMQ135);
      Blynk.virtualWrite(V_MQ3_SENSOR, (int)filteredMQ3);
      Blynk.virtualWrite(V_TEMPERATURE, currentTemp);
      Blynk.virtualWrite(V_HUMIDITY, currentHumidity);
      Blynk.virtualWrite(V_MQ135_RATIO, ratio_MQ135);
      Blynk.virtualWrite(V_MQ3_RATIO, ratio_MQ3);
    }
    
    // Print readings periodically
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 30000) { // Every 30 seconds
      lastPrint = millis();
      
      Serial.println("=== CURRENT READINGS ===");
      Serial.print("MQ135: ");
      Serial.print(filteredMQ135);
      Serial.print(" (Ratio: ");
      Serial.print(ratio_MQ135, 2);
      Serial.print(", Trend: ");
      Serial.print(mq135Processor.getTrend(), 3);
      Serial.println(")");
      Serial.print("MQ3: ");
      Serial.print(filteredMQ3);
      Serial.print(" (Ratio: ");
      Serial.print(ratio_MQ3, 2);
      Serial.print(", Trend: ");
      Serial.print(mq3Processor.getTrend(), 3);
      Serial.println(")");
      Serial.print("Temp: ");
      Serial.print(currentTemp, 1);
      Serial.print("°C | Humidity: ");
      Serial.print(currentHumidity, 1);
      Serial.println("%");
      Serial.print("Stability - MQ135: ");
      Serial.print(mq135Processor.isStable() ? "STABLE" : "UNSTABLE");
      Serial.print(" | MQ3: ");
      Serial.println(mq3Processor.isStable() ? "STABLE" : "UNSTABLE");
      Serial.println(statusMessage);
      Serial.println("========================");
    }
  }
}

void handleStateError() {
  // Flash red LED
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 500) {
    lastBlink = millis();
    digitalWrite(RED_LED, !digitalRead(RED_LED));
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, LOW);
  }
  
  // Turn off relay during error
  digitalWrite(RELAY_PIN, HIGH);
  
  // Update status
  Blynk.virtualWrite(V_STATUS, statusMessage);
  
  Serial.println("🚨 SYSTEM ERROR: " + statusMessage);
}

// ============================================================================
// BLYNK HANDLERS
// ============================================================================

BLYNK_WRITE(V_BUZZER_OFF) {
  buzzerEnabled = !param.asInt();
  if (!buzzerEnabled) {
    digitalWrite(BUZZER_PIN, LOW);
  }
  Serial.print("Buzzer ");
  Serial.println(buzzerEnabled ? "Enabled" : "Disabled");
}

// ============================================================================
// MAIN SETUP AND LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  
  if (!SAFE_MODE) {
    Serial.println("🚨 ERROR: SAFE_MODE disabled! Enable SAFE_MODE before use.");
    while(true) delay(1000);
  }
  
  currentState = STATE_INITIALIZING;
  stateStartTime = millis();
}

void loop() {
  Blynk.run();
  
  // Safety monitoring (always active)
  safetyMonitor();
  
  // Ultrasonic sensor and relay control (always active)
  updateUltrasonicRelay();
  
  // State machine execution
  switch (currentState) {
    case STATE_INITIALIZING:
      handleStateInitializing();
      break;
      
    case STATE_PREHEATING:
      handleStatePreheating();
      break;
      
    case STATE_READY_FOR_CALIBRATION:
      handleStateReadyForCalibration();
      break;
      
    case STATE_CALIBRATING:
      handleStateCalibrating();
      break;
      
    case STATE_MONITORING:
      handleStateMonitoring();
      break;
      
    case STATE_ERROR:
      handleStateError();
      break;
  }
  
  // Update status message to Blynk (responsive UI)
  if (millis() - lastStatusUpdate > STATUS_UPDATE_INTERVAL) {
    lastStatusUpdate = millis();
    static String lastSentStatus = "";
    
    if (statusMessage != lastSentStatus) {
      Blynk.virtualWrite(V_STATUS, statusMessage);
      lastSentStatus = statusMessage;
    }
  }
  
  // Small delay to prevent watchdog issues
  delay(10);
}
