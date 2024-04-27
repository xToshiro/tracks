//================================================================Bibliotecas===================================================================

#include <ESP32Time.h>       // For managing time on ESP32
#include <MPU9250_asukiaaa.h> // For MPU9250 sensor interface
#include <SD.h>              // For SD card operations
#include <SPI.h>             // For SPI communication protocols
#include <TinyGPSPlus.h>     // For GPS functionalities
#include <Wire.h>            // For I2C communication
#include "esp_timer.h"

//=============================================================Final-Bibliotecas==================================================================

//================================================================Configurações===================================================================

constexpr int SDA_PIN = 21;        // SDA pin for I2C communication
constexpr int SCL_PIN = 22;        // SCL pin for I2C communication
constexpr unsigned long I2C_Freq = 400000; // I2C frequency in Hz

// Sampling configuration
constexpr int samplesPerSecond = 10;
//unsigned long lastSampleTime = 0; // Time of the last sample
constexpr long sampleInterval = 1000 / samplesPerSecond;
int sampleIndex = 0; // Sample counter

constexpr int LED_BUILTIN = 2;    // Built-in LED pin on ESP32
constexpr int LED_GREEN = 4;    // LED indicate gps active
ESP32Time rtc(-10800);            // ESP32Time object with GMT-3 offset
constexpr int GPS_BAUDRATE = 9600; // Baud rate for GPS module

//==============================================================Final-Configurações=================================================================

//================================================================Variaveis=========================================================================

MPU9250_asukiaaa mySensor; // Sensor interface object
TinyGPSPlus gps;                  // GPS data parser
String dataMessage;               // Data message storage for SD writing
File dataFile;                    // File object for SD operations
String fileName;
// Time data structure
struct TimeData {
  int day, month, year, hour, minute, second;
} rtcTime, gpsTime;

int gpsUpdate = 0; // GPS data update flag
int lastSecond;
bool ledState = LOW;

char latitudeStr[15], longitudeStr[15]; // Human-readable latitude and longitude
double gpsAltitude = 0;                 // GPS altitude
float gpsSpeed = 0;                     // GPS speed

// Sensor data structure
struct SensorData {
  float ax, ay, az;          // Accelerometer readings
  float gx, gy, gz;          // Gyroscope readings
  float aSqrt;               // Square root of accelerometer sums
  float gxOffset = 0, gyOffset = 0, gzOffset = 0; // Gyroscope offsets
} sensorData;

//==============================================================Final-Variaveis=======================================================================

//================================================================Funções-Gerais======================================================================

// Check if it's time to collect and save data based on the predefined interval
void IRAM_ATTR onTimer(void* arg) {
  updateSampleIndex();
  updateTimeDateRTCdata();
  updateMPUdata();
  saveData(); // Save the collected data
  digitalWrite(LED_BUILTIN, HIGH); // Turn on the built-in LED to indicate data collection
  gpsUpdate = 0; // Reset the GPS update flag
}

void updateTimeDateRTCdata() {
  // Update RTC time variables
  rtcTime.month = (rtc.getMonth() + 1); rtcTime.day = rtc.getDay(); rtcTime.year = rtc.getYear();
  rtcTime.hour = rtc.getHour(true); // true for 24-hour format
  rtcTime.minute = rtc.getMinute(); rtcTime.second = rtc.getSecond();
}

void updateMPUdata() {
  // Update accelerometer values with the latest readings
  if (mySensor.accelUpdate() == 0) {
    sensorData.ax = mySensor.accelX(); sensorData.ay = mySensor.accelY(); sensorData.az = mySensor.accelZ(); sensorData.aSqrt = mySensor.accelSqrt(); // Square root of sum of squares of accelerometer readings
  } else {
    digitalWrite(LED_BUILTIN, LOW); // Turn off the LED if a valid location is obtained
  }
  // Apply offsets to gyroscope values and update
  if (mySensor.gyroUpdate() == 0) {
    sensorData.gx = mySensor.gyroX() - sensorData.gxOffset; sensorData.gy = mySensor.gyroY() - sensorData.gyOffset; sensorData.gz = mySensor.gyroZ() - sensorData.gzOffset;
  } else {
    digitalWrite(LED_BUILTIN, LOW); // Turn off the LED if a valid location is obtained
  }
}

void blinkLed(int count) {
  // Blink the built-in LED a specified number of times
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_GREEN, HIGH); // Turn the LED on
    delay(500); // Wait for half a second
    digitalWrite(LED_GREEN, LOW); // Turn the LED off
    delay(500); // Wait for another half second
  }
}

void ErrorLoopIndicator() {
  // Continuously blink the LED to indicate an error
  while(1) {
      digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
      delay(100); // Wait for 150 milliseconds
      digitalWrite(LED_BUILTIN, LOW); // Turn the LED off
      delay(100); // Wait for 150 milliseconds
  }
}

void clearSerialScreen() {
  // Clear the serial monitor screen by printing 30 new lines
  for (int i = 0; i < 30; i++) {
    Serial.println();
  }
}

void syncGps() {
  digitalWrite(LED_BUILTIN, HIGH);
  // Synchronize the internal RTC with GPS time
  Serial.println(F("Initiating synchronization of the internal RTC with the GPS!"));
  delay(500); // Short delay before starting synchronization
  // Continue trying to synchronize RTC with GPS until the year is within valid range (2023-2026)
  while (rtc.getYear() < 2023 || rtc.getYear() > 2026) {
    if (Serial2.available() > 0) {
      if (gps.encode(Serial2.read())) {
        delay(150); // Small delay may be necessary for some GPS modules to process data
        if (gps.date.isValid() && gps.time.isValid()) {
          // Set the RTC time based on GPS time
          rtc.setTime(gps.time.second(), gps.time.minute(), gps.time.hour(), gps.date.day(), (gps.date.month()), gps.date.year());
        }
      }
    }
    // Provide feedback if no valid GPS data is received within a certain timeframe
    if (millis() > 20000 && gps.charsProcessed() < 10) {
      Serial.println("No valid gps data.");
    }
  }
  Serial.println("Good synchronization between RTC and GPS.");
  Serial.print("GPS date&time: ");
  // Print and set GPS date and time
  Serial.print(gps.date.year()); Serial.print("-"); Serial.print(gps.date.month()); Serial.print("-"); Serial.print(gps.date.day()); Serial.print(" ");
  Serial.print(gps.time.hour()); Serial.print(":"); Serial.print(gps.time.minute()); Serial.print(":"); Serial.println(gps.time.second());
  // Print the synchronized RTC date and time
  Serial.print("RTC date&time: "); Serial.println(rtc.getTime("%Y-%m-%d %H:%M:%S"));
  // Synchronize GPS latitude and longitude
  // Continue to attempt to get valid GPS data until latitude is non-zero
  Serial.println("Waiting for start location.");
  while(gpsUpdate == 0){
    if (Serial2.available() > 0) {
      if (gps.encode(Serial2.read())) {
        if (gps.location.isValid()) {
            // Convert latitude and longitude to string with fixed format
            dtostrf(gps.location.lat(), 12, 8, latitudeStr); dtostrf(gps.location.lng(), 12, 8, longitudeStr);
            gpsUpdate = 1; // Set GPS update flag
            // Print the formatted latitude and longitude strings
            Serial.print("Start location found: ");
            Serial.print("Latitude: "); Serial.print(latitudeStr);
            Serial.print("  /  Longitude: "); Serial.println(longitudeStr);
        }
      }
    }
  }
  blinkLed(4); // Blink LED four times to indicate setup completion
  digitalWrite(LED_BUILTIN, LOW);
}

void updateSampleIndex() {
  // Increment and check the sample index, reset if it exceeds samples per second
  sampleIndex++;
  if (sampleIndex > samplesPerSecond) {
    Serial.println();
    sampleIndex = 1; // Reset index after reaching the limit
  }
}

// Função para alternar o estado do LED
void toggleLED() {
  // Inverte o estado do LED
  ledState = !ledState;
  // Atualiza o estado do LED
  digitalWrite(LED_GREEN, ledState);
}

void updateGpsData() {
  // Check if a second has passed to reduce GPS data processing load
  int nowSecond = rtc.getSecond();
  if (nowSecond != lastSecond) {
    // Check if there is any data available from the GPS module
    while (Serial2.available() > 0) 
      // Try to parse GPS data
      if (gps.encode(Serial2.read())) {
        // Check if the location data is valid
        if (gps.location.isValid()) {
          // Convert latitude and longitude to string with fixed format
          dtostrf(gps.location.lat(), 12, 8, latitudeStr); 
          dtostrf(gps.location.lng(), 12, 8, longitudeStr);
          gpsUpdate = 1; // Set GPS update flag
          // Update altitude if data is valid
          if (gps.altitude.isValid()) {
            gpsAltitude = gps.altitude.meters();
          }
          // Update speed if data is valid
          if (gps.speed.isValid()) {
            gpsSpeed = gps.speed.kmph();
          }
          // Update GPS time and date variables if data is valid
          if (gps.date.isValid() && gps.time.isValid()) {
            // Alterna o estado do LED
            toggleLED();
            gpsTime.year = gps.date.year(); gpsTime.month = gps.date.month(); gpsTime.day = gps.date.day(); gpsTime.hour = gps.time.hour(); gpsTime.minute = gps.time.minute(); gpsTime.second = gps.time.second();
          }
        }
      }
    
    lastSecond = nowSecond;
  }
}

void initProgram() {

  
  pinMode(LED_BUILTIN, OUTPUT); // Set the built-in LED pin as an output
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200); // Initialize serial communication at 115200 bps
  Serial2.begin(GPS_BAUDRATE); // Initialize GPS module serial communication
  delay(2000);
  clearSerialScreen();
  // Display startup message
  Serial.println(F("PaveVibe - Coded by Jairo Ivo"));
  while (!Serial); // Wait for the serial port to connect, necessary for native USB
  pinMode(LED_GREEN, OUTPUT);
  // Initialize I2C communication for the MPU9250 sensor
  Wire.begin(SDA_PIN, SCL_PIN, I2C_Freq); // Start I2C with custom pins and frequency
  mySensor.setWire(&Wire); // Assign Wire object to the sensor for communication
  
  blinkLed(1);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}

void checkAndCalibrateMPU() {
  digitalWrite(LED_BUILTIN, HIGH);
   // Check for sensor presence on the I2C bus at address 0x68
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("Accelerometer not connected, check wiring!"));
    ErrorLoopIndicator(); // Execute error handling routine if sensor is not detected
  } else {
    mySensor.beginAccel(); // Initialize the accelerometer
    mySensor.beginGyro();  // Initialize the gyroscope
    mySensor.beginMag();   // Initialize the magnetometer

    delay(1000);
    Serial.println(F("Calibrating gyroscope, do not move!"));
    delay(500);

    calculateGyroOffsets(); // Calculate gyroscope offsets for calibration
    blinkLed(2); // Blink LED twice to indicate successful sensor setup
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
  }
}

void calculateGyroOffsets() {
  // Initialize accumulators for gyro readings
  long gXTotal = 0, gYTotal = 0, gZTotal = 0;
  const int samples = 100; // Number of samples to collect for calibration
  // Collect 'samples' number of gyro readings
  for (int i = 0; i < samples; i++) {
    // Wait for a new gyro reading to be available
    while (mySensor.gyroUpdate() != 0);
    // Accumulate the gyro readings
    gXTotal += mySensor.gyroX(); gYTotal += mySensor.gyroY(); gZTotal += mySensor.gyroZ();
    // Introduce a delay between readings to allow for fresh data
    // Note: Consider reducing or removing this delay if real-time performance is critical
    delay(100);
  }
  // Calculate the average of the accumulated readings to find the offsets
  sensorData.gxOffset = gXTotal / samples; sensorData.gyOffset = gYTotal / samples; sensorData.gzOffset = gZTotal / samples;
  Serial.println(F("Good calibration."));
}

// Function to wait until a specified number of milliseconds before the next second starts
void waitStart(int leadTimeMs) {
    delay(2000); // Initial delay for stabilization
    Serial.println("Wait to start..."); // Inform about the waiting period beginning

    // Ensure we are not too close to the next second, using correct method
    while((1000 - rtc.getMillis() % 1000) < leadTimeMs) {
        delay(10); // Short active wait to reduce loop load
    }

    // Calculate the start time in milliseconds
    unsigned long startTime = millis();

    // Wait until the specified lead time before the next second begins
    while ((millis() - startTime) < (1000 - leadTimeMs)) {
        // Empty loop to wait until the specified time before the next second
    }

    clearSerialScreen(); // Clear the terminal screen
}

//=============================================================Final-Funções-Gerais======================================================================

//================================================================Funções-SdCard=========================================================================

// Returns the filename based on the current date and time
String getFileName() {
  char fileName[25];  // Buffer to store the filename
  // Format the filename with day, month, year, and hour from the RTC data
  snprintf(fileName, sizeof(fileName), "/D%02d-%02d-%04d--H%02d.csv", rtcTime.day, rtcTime.month, rtcTime.year, rtcTime.hour);
  return String(fileName);
}

// Check if the data file exists on the SD card and create it with a header if it does not or is empty
void checkSDFile() {
  digitalWrite(LED_BUILTIN, HIGH);
  fileName = getFileName();  // Get the current file name based on the RTC date and time
  Serial.print("Checking file: "); Serial.println(fileName);

  // Tenta abrir o arquivo apenas para leitura para verificar sua existência
  File file = SD.open(fileName.c_str(), FILE_READ);
  if (!file) {
    Serial.println("File does not exist or cannot be opened for reading.");
    file = SD.open(fileName.c_str(), FILE_WRITE);  // Cria um novo arquivo para escrita
    if (file) {
      Serial.println("File created. Writing header...");
      const char *header = "Date, Time, GPSDate, GPSTime, Latitude, Longitude, Altitude, Speed, GPSUpdate, AccX, AccY, AccZ, GyroX, GyroY, GyroZ, SampleIndex\r\n";
      if (file.print(header)) {
        //blinkLed(6);
        //digitalWrite(LED_BUILTIN, LOW);
        Serial.println("Header written successfully.");
      } else {
        Serial.println("Failed to write header.");
        ErrorLoopIndicator();  // Indicate an error in a non-recoverable loop
      }
      file.close();
    } else {
      Serial.println("Failed to create file.");
      ErrorLoopIndicator();  // Indicate an error in a non-recoverable loop
    }
  } else {
    Serial.println("File exists, no need to create.");
    file.close();
    
    blinkLed(6);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

// Initialize the SD card and check its status and size
void initSDCard() {
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    ErrorLoopIndicator();  // Indicate an error in a non-recoverable loop
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    ErrorLoopIndicator();  // Indicate an error in a non-recoverable loop
    return;
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);  // Convert bytes to megabytes
  Serial.printf("SD Card Size: %lluMB\n", cardSize);  // Print the size of the SD card
}

// Write to a file, creating it if it does not exist
void writeFile(fs::FS &fs, const char *path, const char *message) {
  //Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    //Serial.println("Failed to open file for writing");
    ErrorLoopIndicator();  // Indicate an error in a non-recoverable loop
    return;
  }
  if (file.print(message)) {
    //Serial.println("File written successfully");
  } else {
    //Serial.println("Write failed");
    ErrorLoopIndicator();  // Indicate an error if writing fails
  }
  file.close();  // Close the file to save the data properly
}

// Adiciona dados ao arquivo
void appendFile(fs::FS &fs, const char *path, const char *message) {
  //Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    //Serial.println("Failed to open file for appending");
    ErrorLoopIndicator();
    return;
  }
  if (file.print(message)) {
    //Serial.println("Message appended");
  } else {
    //Serial.println("Append failed");
    ErrorLoopIndicator();
  }
  file.close();
}

void saveData() {
  // Construct the data message to be saved on the SD card
  dataMessage = String(rtcTime.day) + "/" + String(rtcTime.month) + "/" + String(rtcTime.year) + "," 
              + String(rtcTime.hour) + ":" + String(rtcTime.minute) + ":" + String(rtcTime.second) + ","
              + String(gpsTime.day) + "/" + String(gpsTime.month) + "/" + String(gpsTime.year) + ","
              + String(gpsTime.hour) + ":" + String(gpsTime.minute) + ":" + String(gpsTime.second) + ","
              + String(latitudeStr) + "," + String(longitudeStr) + "," 
              + String(gpsAltitude) + "," + String(gpsSpeed) + "," + String(gpsUpdate) + ","
              + String(sensorData.ax) + "," + String(sensorData.ay) + "," + String(sensorData.az) + ","
              + String(sensorData.gx) + "," + String(sensorData.gy) + "," + String(sensorData.gz) + ","
              + String(sampleIndex) + "\r\n";
  // Display the data message in the Serial Monitor
  Serial.print("Data appended: "); Serial.print(dataMessage);
  // Append the data message to the file on the SD card
  appendFile(SD, fileName.c_str(), dataMessage.c_str());
}

//===========================================================Final-Funções-SdCard=====================================================================

//============================================================Nucleo-de-Execução======================================================================

void setup(){

  initProgram(); // Setup serial and i2c communication
  checkAndCalibrateMPU(); // Function to check connection with MPU and calibrate gyroscope
  syncGps(); // Synchronize the internal RTC with GPS time and GPS latitude and longitude
  updateTimeDateRTCdata(); // Update internal time and date with RTC data
  initSDCard(); // Initialize the SD card module
  checkSDFile(); // Ensure '.csv' file exists on the SD card, create if not found using date and hour in main name
  

  const esp_timer_create_args_t timerArgs = {
    .callback = &onTimer, // Função que será chamada na interrupção
    .name = "sampleTimer"
  };

  esp_timer_handle_t sampleTimer;
  esp_timer_create(&timerArgs, &sampleTimer);
  // Inicia o timer para disparar a cada 100 milissegundos (100000 microssegundos)
  waitStart(100); // Sync to start collecting data in the next second
  esp_timer_start_periodic(sampleTimer, sampleInterval * 1000); // Configura o timer para disparar periodicamente
    
}

void loop(){

  updateGpsData();

}

//=========================================================Final-Nucleo-de-Execução====================================================================

