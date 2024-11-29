/*
  Project:      Temp-Monitor
  Description:  Regularly sample and log temperature and humidity levels
*/

// hardware and internet configuration parameters
#include "config.h"
// private credentials for Internet and network endpoints
#include "secrets.h"

// read/write to ESP32 persistent storage
#include <Preferences.h>
Preferences nvStorage;

// global variables
// IMPROVEMENT: nvStorageRead() should return these locally
float accumulatingTempF;
float accumulatingHumidity;

// environment sensor data
typedef struct {
  float ambientHumidity;      // RH [%]
  float ambientTemperatureF;
} envData;
envData sensorData;

// hardware status data
typedef struct {
  float batteryPercent;
  float batteryVoltage;
  uint8_t rssi;
} hdweData;
hdweData hardwareData;

#ifndef HARDWARE_SIMULATE
  // initialize environment sensor
  #if defined(BME280)
    #include <Adafruit_BME280.h>
    Adafruit_BME280 envSensor;
  #elif defined(AHTX0)
    #include <Adafruit_AHTX0.h>
    Adafruit_AHTX0 envSensor;
  #endif
  Adafruit_Sensor *envSensorHumidity, *envSensorTemp;

  // battery voltage sensor
  #include <Adafruit_LC709203F.h>
  Adafruit_LC709203F lc;

  // WiFi support
  #if defined(ESP8266)
    #include <ESP8266WiFi.h>
  #elif defined(ESP32)
    #include <WiFi.h>
    #include <HTTPClient.h>
  #endif
  WiFiClient client;
#endif

#ifdef INFLUX
  extern boolean post_influx(float temperatureF, float humidity, float batteryVoltage, uint8_t rssi);
#endif

#ifdef MQTT
  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  // Adafruit_MQTT_Client aq_mqtt(&client, MQTT_BROKER, MQTT_PORT, DEVICE_ID, MQTT_USER, MQTT_PASS);
  Adafruit_MQTT_Client aq_mqtt(&client, MQTT_BROKER, MQTT_PORT, DEVICE_ID);
  
  extern bool mqttDeviceWiFiUpdate(uint8_t rssi);
  extern bool mqttDeviceBatteryUpdate(float batteryVoltage);
  extern bool mqttSensorTemperatureFUpdate(float temperatureF);
  extern bool mqttSensorHumidityUpdate(float humidity);
  #ifdef HASSIO_MQTT
    extern void hassio_mqtt_publish(float temperatureF, float humidity, float batteryVoltage);
  #endif
#endif

void setup() {
  // One time run of code, then deep sleep
  // handle Serial first so debugMessage() works
  #ifdef DEBUG
    Serial.begin(115200);
    // wait for serial port connection
    while (!Serial);
    // Display key configuration parameters
    debugMessage(String("Starting Temp Monitor with ") + sensorSampleInterval + " second sample interval",1);
    #if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT)
      debugMessage(String("Reporting after ") + sensorSampleSize + " samples",1);
    #endif
    debugMessage(String("Internet reconnect delay is ") + networkConnectAttemptInterval + " seconds",2);
  #endif

  #ifdef HARDWARE_SIMULATE
    // truely random number per boot cycle
    randomSeed(analogRead(0));
  #endif

  powerEnable();

  // Initialize environment sensor
  if (!sensorInit()) {
    // This error often occurs after a firmware flash and then resetting the board
    // Hardware deep sleep typically resolves it, so quickly cycle the hardware
    powerDisable(hardwareRebootInterval);
  }

  // Environmental sensor available, so fetch values
  if (!sensorRead()) {
    powerDisable(hardwareRebootInterval);
  }

  uint8_t sampleCounter = nvStorageRead();
  sampleCounter++;
  debugMessage(String("Sample count: ") + (sampleCounter) + " of " + sensorSampleSize,1);

  accumulatingTempF += sensorData.ambientTemperatureF;
  accumulatingHumidity += sensorData.ambientHumidity;

  if (sampleCounter < sensorSampleSize) {
    // add to the accumulating, intermediate sensor values, then sleep device
    nvStorageWrite(sampleCounter, accumulatingTempF, accumulatingHumidity);
    powerDisable(sensorSampleInterval);
  }

  // this code only executes if sampleCounter == sensorSampleSize
  // average values, send to network endpoint if possible, reset values, then sleep device

  float averageTempF = accumulatingTempF / sampleCounter;
  float averageHumidity = accumulatingHumidity / sampleCounter;
  debugMessage(String("Reporting averaged TempF:") + averageTempF + "F, Humidity:" + averageHumidity, 1);

  if (!batteryRead(batteryReadsPerSample))
    hardwareData.batteryVoltage = 0;

  // Setup network connection specified in config.h
  networkConnect();

  if (hardwareData.rssi != 0) {
    // write to network endpoints
    #ifdef MQTT
      if ((mqttSensorTemperatureFUpdate(averageTempF)) && (mqttSensorHumidityUpdate(averageHumidity)) && (mqttDeviceWiFiUpdate(hardwareData.rssi)) && (mqttDeviceBatteryUpdate(hardwareData.batteryVoltage)))
        debugMessage("MQTT publish successful",1);
      else
        debugMessage("One or more MQTT publishes unsuccessful",1);
      #ifdef HASSIO_MQTT
          debugMessage("Establishing MQTT for Home Assistant", 1);
          // Either configure sensors in Home Assistant's configuration.yaml file
          // directly or attempt to do it via MQTT auto-discovery
          // hassio_mqtt_setup();  // Config for MQTT auto-discovery
          hassio_mqtt_publish(averageTempF, averageHumidity, hardwareData.batteryVoltage);
      #endif
    #endif

    #ifdef INFLUX
      // Returns true if successful
      if (post_influx(averageTempF, averageHumidity, hardwareData.batteryVoltage, hardwareData.rssi))
        debugMessage("Influx publish successful",1);
      else
        debugMessage("Influx publish unsuccessful",1);
    #endif
  }
  //reset nvStorage values for next recording period
  nvStorageWrite(0, 0, 0);
  powerDisable(sensorSampleInterval);
}

void loop() {}

// Hardware simulation routines
#ifdef HARDWARE_SIMULATE
  void sensorSimulate()
  // Simulate environment sensor data
  // Improvement - implement stable, rapid rise and fall 
  {
    sensorData.ambientTemperatureF = random(sensorTempMinF,sensorTempMaxF) / 100.0;
    sensorData.ambientHumidity = random(sensorHumidityMin,sensorHumidityMax) / 100.0;
    debugMessage(String("SIMULATED sensor: ") + sensorData.ambientTemperatureF + "F, " + sensorData.ambientHumidity + "% Humidity",1);
  }

  void batterySimulate()
  // Simulate battery data
  {
    hardwareData.batteryVoltage = random(batterySimVoltageMin, batterySimVoltageMax) / 100.00;
    hardwareData.batteryPercent = batteryGetChargeLevel(hardwareData.batteryVoltage);
    debugMessage(String("SIMULATED Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%", 1);  
  }

  void networkSimulate()
  // Simulates successful WiFi connection data
  {
    // IMPROVEMENT : simulate IP address?
    hardwareData.rssi = random(networkRSSIMin, networkRSSIMax);
    debugMessage(String("SIMULATED WiFi RSSI: ") + hardwareData.rssi,1);
  }
#endif

bool batteryRead(uint8_t reads)
// sets global battery values from i2c battery monitor or analog pin value on supported boards
{
  #ifdef HARDWARE_SIMULATE
    batterySimulate();
    return true;
  #else
    // is LC709203F on i2c available?
    if (lc.begin())
    {
      debugMessage(String("Version: 0x") + lc.getICversion(), 2);
      lc.setPackAPA(BATTERY_APA);
      //lc.setThermistorB(3950);

      hardwareData.batteryPercent = lc.cellPercent();
      hardwareData.batteryVoltage = lc.cellVoltage();
      //hardwareData.batteryTemperatureF = 32 + (1.8* lc.getCellTemperature());
    }
    else 
    {
      // read gpio pin on supported boards for battery level
      #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
        // modified from the Adafruit power management guide for Adafruit ESP32V2
        float accumulatedVoltage = 0.0;
        for (uint8_t loop = 0; loop < reads; loop++)
        {
          accumulatedVoltage += analogReadMilliVolts(BATTERY_VOLTAGE_PIN);
        }
        hardwareData.batteryVoltage = accumulatedVoltage/reads; // we now have the average reading
        // convert into volts  
        hardwareData.batteryVoltage *= 2;    // we divided by 2, so multiply back
        hardwareData.batteryVoltage /= 1000; // convert to volts!
        hardwareData.batteryPercent = batteryGetChargeLevel(hardwareData.batteryVoltage);
      #endif
    }
    if (hardwareData.batteryVoltage != 0) {
      debugMessage(String("Battery voltage: ") + hardwareData.batteryVoltage + "v, percent: " + hardwareData.batteryPercent + "%", 1);
      return true;
    }
    else
      return false;
  #endif
}

uint8_t batteryGetChargeLevel(float volts) {
  uint8_t idx = 50;
  uint8_t prev = 0;
  uint8_t half = 0;
  if (volts >= 4.2) {
    return 100;
  }
  if (volts <= 3.2) {
    return 0;
  }
  while (true) {
    half = abs(idx - prev) / 2;
    prev = idx;
    if (volts >= batteryVoltageTable[idx]) {
      idx = idx + half;
    } else {
      idx = idx - half;
    }
    if (prev == idx) {
      break;
    }
  }
  debugMessage(String("Computed battery percent ") + idx + "%", 2);
  return idx;
}

bool sensorInit()
// initializes ATHX0 or BME280 environment sensor if available
{

  #ifdef HARDWARE_SIMULATE
    return true;
  #else
    if (envSensor.begin()) {
      envSensorTemp = envSensor.getTemperatureSensor();
      #ifdef DEBUG
        if (DEBUG == 2 )
          envSensorTemp->printSensorDetails();
      #endif
      envSensorHumidity = envSensor.getHumiditySensor();
      #ifdef DEBUG
        if (DEBUG == 2)
          envSensorHumidity->printSensorDetails();
      #endif
      debugMessage("Environment sensor ready", 2);
      return true;
    } else {
      debugMessage("Environment sensor failed to initialize", 1);
      return false;
    }
  #endif
}

bool sensorRead()
// stores environment sensor to environment global
{
  #ifdef HARDWARE_SIMULATE
    sensorSimulate();
    return true;
  #else
    // AHTX0, BME280
    // FIX : Can we get error conditions from this API?
    sensors_event_t humidityEvent, tempEvent;
    envSensorTemp->getEvent(&tempEvent);
    envSensorHumidity->getEvent(&humidityEvent);
    sensorData.ambientTemperatureF = (tempEvent.temperature * 1.8) + 32;
    sensorData.ambientHumidity = humidityEvent.relative_humidity;
  #endif
  debugMessage(String("Environment sensor: ") + sensorData.ambientTemperatureF + "F, " + sensorData.ambientHumidity + "%", 1);
  return true;
}

uint8_t nvStorageRead()
// read data from ESP32 NV storage and store in appropriate global variables
{
  nvStorage.begin("air-quality", false);
  uint8_t storedCounter = nvStorage.getInt("counter", 0);  // counter tracks a 0 based array
  debugMessage(String("Sample count FROM nv storage is ") + storedCounter, 2);

  // read value or insert current sensor reading if this is the first read from nv storage
  accumulatingTempF = nvStorage.getFloat("temp", 0);
  // BME280 often issues nan when not configured properly
  if (isnan(accumulatingTempF)) {
    // bad value, replace with current temp
    accumulatingTempF = (sensorData.ambientTemperatureF * storedCounter);
    debugMessage("Unexpected temperatureF value in nv storage replaced with multiple of current temperature", 2);
  }

  accumulatingHumidity = nvStorage.getFloat("humidity", 0);
  if (isnan(accumulatingHumidity)) {
    // bad value, replace with current temp
    accumulatingHumidity = (sensorData.ambientHumidity * storedCounter);
    debugMessage("Unexpected humidity value in nv storage replaced with multiple of current humidity", 2);
  }

  debugMessage(String("running totals FROM nv storage: TempF:") + accumulatingTempF + ", Humidity:" + accumulatingHumidity, 2);
  return storedCounter;
}

void nvStorageWrite(uint8_t counter, float accumulatedTempF, float accumulatedHumidity)
// write sample counter, accumulated tempF and humidity to ESP32 NV storage
{
  nvStorage.putInt("counter", counter);
  debugMessage(String("Sample count TO nv storage: ") + counter, 2);
  nvStorage.putFloat("temp", accumulatedTempF);
  nvStorage.putFloat("humidity", accumulatedHumidity);
  debugMessage(String("running totals TO nv storage: TempF:") + accumulatedTempF + ", Humidity:" + accumulatedHumidity, 2);
}

void powerEnable()
// enables I2C across multiple Adafruit ESP32 variants
{
  debugMessage("powerEnable() started", 2);

  // enable I2C on devices with two ports
  #if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO)
    // ESP32 is kinda odd in that secondary ports must be manually assigned their pins with setPins()!
    Wire1.setPins(SDA1, SCL1);
    debugMessage("power on: ESP32 variant with two I2C ports", 2);
  #endif

  // Adafruit ESP32 I2C power management
  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    // turn on the I2C power by setting pin to opposite of 'rest state'
    // Rev B board is LOW to enable
    // Rev C board is HIGH to enable
    pinMode(PIN_I2C_POWER, INPUT);
    delay(1);
    bool polarity = digitalRead(PIN_I2C_POWER);
    pinMode(PIN_I2C_POWER, OUTPUT);
    digitalWrite(PIN_I2C_POWER, !polarity);
    debugMessage("power on: Feather ESP32S2 I2C", 2);
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn on the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
    debugMessage("power on: Feather ESP32V2 I2C", 2);
  #endif

  debugMessage("powerEnable() complete", 1);
}

void powerDisable(uint16_t deepSleepTime)
// Powers down hardware activated via powerEnable() then deep sleep ESP32
{
  debugMessage("powerDisable started", 2);

  networkDisconnect();

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)
    // Turn off the I2C power
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, LOW);
    debugMessage("power off: ESP32V2 I2C", 2);
  #endif

  #if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
    // Rev B board is LOW to enable
    // Rev C board is HIGH to enable
    digitalWrite(PIN_I2C_POWER, LOW);
    debugMessage("power off: ESP32S2 I2C", 2);
  #endif

  esp_sleep_enable_timer_wakeup(deepSleepTime * 1000000);  // ESP microsecond modifier
  debugMessage(String("powerDisable complete: ESP32 deep sleep for ") + (deepSleepTime) + " seconds", 1);
  esp_deep_sleep_start();
}

bool networkConnect() 
// Connect to WiFi network specified in secrets.h
{
  #ifdef HARDWARE_SIMULATE
    networkSimulate();
    return true;
  #endif

  // use only if using network data endpoints
  #if defined(MQTT) || defined(INFLUX) || defined(HASSIO_MQTT)

    // reconnect to WiFi only if needed
    if (WiFi.status() == WL_CONNECTED) 
    {
      debugMessage("Already connected to WiFi",2);
      return true;
    }
    // set hostname has to come before WiFi.begin
    WiFi.hostname(DEVICE_ID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    for (uint8_t loop = 1; loop <= networkConnectAttemptLimit; loop++)
    // Attempts WiFi connection, and if unsuccessful, re-attempts after networkConnectAttemptInterval second delay for networkConnectAttemptLimit times
    {
      if (WiFi.status() == WL_CONNECTED) {
        hardwareData.rssi = abs(WiFi.RSSI());
        debugMessage(String("WiFi IP address lease from ") + WIFI_SSID + " is " + WiFi.localIP().toString(), 1);
        debugMessage(String("WiFi RSSI is: ") + hardwareData.rssi + " dBm", 1);
        return true;
      }
      debugMessage(String("Connection attempt ") + loop + " of " + networkConnectAttemptLimit + " to " + WIFI_SSID + " failed", 1);
      debugMessage(String("WiFi status message ") + networkWiFiMessage(WiFi.status()),2);
      // use of delay() OK as this is initialization code
      delay(networkConnectAttemptInterval * 1000);  // converted into milliseconds
    }
  #endif
  hardwareData.rssi = 0;
  return false;
}

void networkDisconnect()
// Disconnect from WiFi network
{
  hardwareData.rssi = 0;
  #ifdef HARDWARE_SIMULATE
    debugMessage("power off: SIMULATED WiFi",2);
    return;
  #else
    // IMPROVEMENT: What if disconnect call fails?
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    debugMessage("power off: WiFi",2);
  #endif
}

#ifndef HARDWARE_SIMULATE
  const char* networkWiFiMessage(wl_status_t status)
  // Converts WiFi.status() to string
  {
    switch (status) {
      case WL_NO_SHIELD: return "WL_NO_SHIELD";
      case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
      case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
      case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
      case WL_CONNECTED: return "WL_CONNECTED";
      case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
      case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
      case WL_DISCONNECTED: return "WL_DISCONNECTED";
    }
  }
#endif

void debugMessage(String messageText, uint8_t messageLevel)
// wraps Serial.println as #define conditional
{
#ifdef DEBUG
  if (messageLevel <= DEBUG) {
    Serial.println(messageText);
    Serial.flush();  // Make sure the message gets output (before any sleeping...)
  }
#endif
}