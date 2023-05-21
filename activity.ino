#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <arduinoFFT.h>

#define INTERVAL 15                   // Interval for collecting accelerometer data
#define INTERVAL_MQTT_RECONNECT 5000  // Interval for delay of reconnecting to mqtt
#define INTERVAL_MQTT_SEND 2000       // Interval for sending data to server
#define PIN_LED 2
#define I2C_ADD_MPU 104
#define TABLE_SIZE_MPU 7
#define RATE 10
#define ACC_X_OUT 59
#define GYRO_X_OUT 0x43
#define AVERAGE_SIZE 64  // Number of past readings to consider for moving average and FFT, must be power of 2 for sampling

// For WiFi and MQTT
const char* ssid = "**";
const char* password = "**";
const char* mqtt_server = "37aad5450fca492297d7d3bc3329f4ba.s2.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "rs-user";
const char* mqtt_password = "RSprojectESP8266";
const char* topic = "RS-data";

WiFiClientSecure espClient;
PubSubClient client(espClient);

int32_t table[TABLE_SIZE_MPU];
float delilnikAcc = 16384.0f;
float delilnikGyro = 131.0f;
float acc_x_calib = 0.0;
float acc_y_calib = 0.0;
float acc_z_calib = 0.0;
float gyro_x_calib = 0.0;
float gyro_y_calib = 0.0;
float gyro_z_calib = 0.0;

// Thresholds
const float stepThreshold = 0.065f;
const double runningThreshold = 0.4;
const double runningFrequencyThreshold = 1.15;
bool isRunning = false;
int walkingCount = 0;
int runningCount = 0;
int requiredMajority = 5;  // Majority of the last eight readings

// Variables for steps
unsigned long currentStepCount = 0;
unsigned long unsentRunningSteps = 0;
unsigned long unsentWalkingSteps = 0;

// Variables for peaks
int peakCount = 0;
bool peakDetected = false;
unsigned long lastPeakTime = 0;
const unsigned long peakTimeThreshold = 100;  // Minimum time difference in order for new step to be valid, in milliseconds

unsigned long lastSentTime = 0;
unsigned long lastReadTime = 0;
unsigned long currentTime = 0;

float previousAccel = 0;
float currentAccel = 0;

float pastReadings[AVERAGE_SIZE];     // Array to store past readings
float pastFrequencies[AVERAGE_SIZE];  // Array to store past frequencies
int currentIndex = 0;  // Current index in pastReadings and pastFrequencies arrays
double vImag[AVERAGE_SIZE] = { 0 }; // For FFT
arduinoFFT FFT;

// functions
bool sendData(String activity, unsigned long steps);
void beriPodatke();
void detectCycling(float gyro_x, float gyro_y, float gyro_z, float acc_x, float acc_y, float acc_z);
void acc_config();
void gryo_config();

// Send data to server with MQTT
bool sendData(String activity, unsigned long steps) {
  if (client.connected()) {
    String payload = "{ \"activity\": \"" + activity + "\",\"steps\": " + steps + "}";
    char buffer[payload.length() + 1];
    payload.toCharArray(buffer, (payload.length() + 1));
    client.publish(topic, buffer);
    return true;
  }

  reconnect();

  return false;
}

void beriPodatke() {
  static uint32_t count = 0;
  digitalWrite(PIN_LED, 0);
  static float acc_x = 0.0f;
  static float acc_y = 0.0f;
  static float acc_z = 0.0f;
  static float gyro_x = 0.0f;
  static float gyro_y = 0.0f;
  static float gyro_z = 0.0f;
  int32_t table;

  // Set from where to read
  Wire.beginTransmission(I2C_ADD_MPU);
  Wire.write(ACC_X_OUT);
  Wire.endTransmission();

  // Read at once all data in 3-axis
  Wire.requestFrom(I2C_ADD_MPU, 6);
  table = (int8_t)Wire.read();
  table = table << 8;
  table += (uint8_t)Wire.read();

  acc_x += ((table / delilnikAcc) - acc_x_calib) / RATE;

  table = (int8_t)Wire.read();
  table = table << 8;
  table += (uint8_t)Wire.read();
  acc_y += ((table / delilnikAcc) - acc_y_calib) / RATE;

  table = (int8_t)Wire.read();
  table = table << 8;
  table += (uint8_t)Wire.read();
  acc_z += ((table / delilnikAcc) - acc_z_calib) / RATE;

  // Read gyro data
  Wire.beginTransmission(I2C_ADD_MPU);
  Wire.write(GYRO_X_OUT);
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADD_MPU, 6);
  table = (int8_t)Wire.read();
  table = table << 8;
  table += (uint8_t)Wire.read();
  gyro_x += ((table / delilnikGyro) - gyro_x_calib) / RATE;

  table = (int8_t)Wire.read();
  table = table << 8;
  table += (uint8_t)Wire.read();
  gyro_y += ((table / delilnikGyro) - gyro_y_calib) / RATE;

  table = (int8_t)Wire.read();
  table = table << 8;
  table += (uint8_t)Wire.read();
  gyro_z += ((table / delilnikGyro) - gyro_z_calib) / RATE;

  // Every RATE reading, we check for activity
  if (count % RATE == 0) {
    // Print out
    /*
    Serial.print("ACC: X= ");
    Serial.print(acc_x);
    Serial.print(" Y= ");
    Serial.print(acc_y);
    Serial.print(" Z= ");
    Serial.print(acc_z);
    Serial.println("");*/

    float totalAccel = sqrt(sq(acc_x) + sq(acc_y) + sq(acc_z));
    previousAccel = currentAccel;
    currentAccel = totalAccel;

    float accelDifference = abs(currentAccel - previousAccel);

    // To better utilize FFT, we set some values to 0
    if (accelDifference < stepThreshold) {
      accelDifference = 0;
    }

    // Shift elements to left by one, to make room for new data
    for (int i = 0; i < AVERAGE_SIZE - 1; i++) {
      pastReadings[i] = pastReadings[i + 1];
      pastFrequencies[i] = pastFrequencies[i + 1];
    }

    // Calculate moving average
    pastReadings[AVERAGE_SIZE - 1] = accelDifference;
    double vReal[AVERAGE_SIZE];
    // Calculate min and max of past readings
    vReal[0] = pastReadings[0];
    vImag[0] = 0.0;
    double sumFreq = pastFrequencies[0];
    double sumAcc = pastReadings[0];
    for (int i = 1; i < AVERAGE_SIZE; i++) {
      vReal[i] = pastReadings[i];
      vImag[i] = 0.0;
      sumFreq += pastFrequencies[i];
      sumAcc += pastReadings[i];
    }

    // Calculate frequency of peaks
    double sampling_frequency = 1000 / INTERVAL / RATE;

    FFT = arduinoFFT(vReal, vImag, AVERAGE_SIZE, sampling_frequency);
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(FFT_FORWARD);
    FFT.ComplexToMagnitude();
    double peakFrequency = FFT.MajorPeak();
    // Sometimes calculated peak frequency can be same as sampling_frequency/2, because of some bugs
    // Set to zero to reduce noise in averages
    if (peakFrequency >= 0.45 * sampling_frequency || peakFrequency <= 0.1) {
      peakFrequency = 0;
    }
    pastFrequencies[AVERAGE_SIZE - 1] = peakFrequency;
    //currentIndex = (currentIndex + 1) % AVERAGE_SIZE;  // Update currentIndex

    double averageFreq = sumFreq / AVERAGE_SIZE;

    // Set dynamic threshold, but make it a little smaller than average
    float dynamicStepThreshold = 0.98f * sumAcc / AVERAGE_SIZE;
    currentTime = millis();
    // Acceleration difference has to be larger than some minimum value, because even when standing still we are not completely still
    // so this could introduce some false steps
    if (accelDifference >= dynamicStepThreshold && !peakDetected && accelDifference >= stepThreshold) {
      peakCount++;
      peakDetected = true;

      // Detect running based on the acceleration difference
      if (accelDifference >= runningThreshold && averageFreq >= runningFrequencyThreshold) {
        isRunning = true;
      } else {
        isRunning = false;
      }

      // Increment the counter for the detected activity
      if (isRunning) {
        if (runningCount < 8) {
          runningCount++;
        }
      } else {
        if (walkingCount < 8) {
          walkingCount++;
        }
      }

      if (walkingCount + runningCount > 8) {
        if (isRunning) {
          if (walkingCount > 0) {
            walkingCount--;
          }
        } else {
          if (runningCount > 0) {
            runningCount--;
          }
        }
      }
    } else if (accelDifference <= dynamicStepThreshold && peakDetected) {
      peakDetected = false;
    }

    // Set activity based on majority
    if (walkingCount >= requiredMajority) {
      isRunning = false;
    } else {
      isRunning = true;
    }

    // Send steps to server and print out variables
    if (peakCount > 0 && currentTime - lastPeakTime > peakTimeThreshold) {
      Serial.println("accelDifference: ");
      Serial.println(accelDifference, 2);
      Serial.print("Frequency: ");
      Serial.println(peakFrequency, 2);
      Serial.print("Average frequency: ");
      Serial.println(averageFreq, 2);

      Serial.print("Step threshold: ");
      Serial.println(dynamicStepThreshold, 4);
      lastPeakTime = currentTime;
      if (isRunning) {
        Serial.println("...........RUNNING");
        unsentRunningSteps += peakCount;
        if (lastPeakTime - lastSentTime > INTERVAL_MQTT_SEND) {
          bool sent = sendData("running", unsentRunningSteps);
          // If sent set counter to zero, otherwise we will send it next time
          if (sent) {
            unsentRunningSteps = 0;
            lastSentTime = lastPeakTime;
          }
        }
      } else {
        Serial.println("...........WALKING");
        unsentWalkingSteps += peakCount;
        if (lastPeakTime - lastSentTime > INTERVAL_MQTT_SEND) {
          bool sent = sendData("walking", unsentWalkingSteps);
          // If sent set counter to zero, otherwise we will send it next time
          if (sent) {
            unsentWalkingSteps = 0;
            lastSentTime = lastPeakTime;
          }
        }
      }

      currentStepCount += peakCount;
      peakCount = 0;
      Serial.print("Overall steps: ");
      Serial.println(currentStepCount);
    }

    // Process gyroscope and accelerometer data for cycling detection
    detectCycling(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);

    // Reset values
    acc_x = 0;
    acc_y = 0;
    acc_z = 0;
    gyro_x = 0.0f;
    gyro_y = 0.0f;
    gyro_z = 0.0f;
  }
  // Counter
  count = count + 1;
  digitalWrite(PIN_LED, 1);
}

void detectCycling(float gyro_x, float gyro_y, float gyro_z, float acc_x, float acc_y, float acc_z) {
  /*
  Serial.print("GYRO: X= ");
  Serial.print(gyro_x);
  Serial.print(" Y= ");
  Serial.print(gyro_y);
  Serial.print(" Z= ");
  Serial.print(gyro_z);
  Serial.println("");*/
}

// Calibrate accelerometer
void acc_calib() {
  digitalWrite(PIN_LED, 0);
  delay(1000);

  int32_t table;

  for (int q = 0; q < AVERAGE_SIZE; q++) {
    Wire.beginTransmission(I2C_ADD_MPU);
    Wire.write(ACC_X_OUT);
    Wire.endTransmission();

    Wire.requestFrom(I2C_ADD_MPU, 6);
    table = (int8_t)Wire.read();
    table = table << 8;
    table += (uint8_t)Wire.read();

    acc_x_calib += (table / delilnikAcc) / RATE;

    table = (int8_t)Wire.read();
    table = table << 8;
    table += (uint8_t)Wire.read();

    acc_y_calib += (table / delilnikAcc) / RATE;

    table = (int8_t)Wire.read();
    table = table << 8;
    table += (uint8_t)Wire.read();

    acc_z_calib += (table / delilnikAcc) / RATE;

    delay(1000 / RATE);
  }

  acc_x_calib /= (AVERAGE_SIZE / RATE);
  acc_y_calib /= (AVERAGE_SIZE / RATE);
  acc_z_calib /= (AVERAGE_SIZE / RATE);

  Serial.println("");
  Serial.println("** ACC CALIB: ");
  Serial.print("ACC: X= ");
  Serial.print(acc_x_calib);
  Serial.print(", Y= ");
  Serial.print(acc_y_calib);
  Serial.print(", Z= ");
  Serial.print(acc_z_calib);
  Serial.println("");

  delay(1000);
}

// Calibrate gyro
void gryo_calib() {
  digitalWrite(PIN_LED, 0);
  delay(1000);

  int32_t table;

  for (int q = 0; q < AVERAGE_SIZE; q++) {
    Wire.beginTransmission(I2C_ADD_MPU);
    Wire.write(GYRO_X_OUT);
    Wire.endTransmission();

    Wire.requestFrom(I2C_ADD_MPU, 6);
    table = (int8_t)Wire.read();
    table = table << 8;
    table += (uint8_t)Wire.read();

    gyro_x_calib += (table / delilnikGyro) / RATE;

    table = (int8_t)Wire.read();
    table = table << 8;
    table += (uint8_t)Wire.read();

    gyro_y_calib += (table / delilnikGyro) / RATE;

    table = (int8_t)Wire.read();
    table = table << 8;
    table += (uint8_t)Wire.read();

    gyro_z_calib += (table / delilnikGyro) / RATE;

    delay(1000 / RATE);
  }

  gyro_x_calib /= (AVERAGE_SIZE / RATE);
  gyro_y_calib /= (AVERAGE_SIZE / RATE);
  gyro_z_calib /= (AVERAGE_SIZE / RATE);

  Serial.println("");
  Serial.println("** GRYO CALIB: ");
  Serial.print("GYRO: X= ");
  Serial.print(gyro_x_calib, 2);
  Serial.print(", Y= ");
  Serial.print(gyro_y_calib, 2);
  Serial.print(", Z= ");
  Serial.print(gyro_z_calib, 2);
  Serial.println("");

  delay(1000);
}

// Connect to WiFi network
void setup_wifi() {
  delay(10);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Reconnect to mqtt if not connected
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    // Create a random client ID
    String clientId = "ESP8266Client-123";
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.print(" try again in ");
      Serial.print(INTERVAL_MQTT_RECONNECT / 1000);
      Serial.println(" seconds");
      delay(INTERVAL_MQTT_RECONNECT);
    }
  }
}


void setup() {
  pinMode(PIN_LED, OUTPUT);

  Serial.begin(115200);

  // Init arrays for past readings
  for (int i = 0; i < AVERAGE_SIZE; i++) {
    pastReadings[i] = 0;
    pastFrequencies[i] = 0.0;
  }

  Serial.println(topic);

  setup_wifi();
  // Disable SSL verification
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);

  if (!client.connected()) {
    reconnect();
  }

  // Init I2C on given pins
  Wire.begin(12, 14);
  // Set bus frequency 100 kHz
  Wire.setClock(100000);

  // Calibration of sensors
  acc_calib();
  gryo_calib();
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();
  if (currentTime - lastReadTime >= INTERVAL) {
    lastReadTime = currentTime;
    beriPodatke();
  }
}
