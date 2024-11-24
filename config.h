/*
  Project Name:   Temp-Monitor
  Description:    public (non-secret) configuration data
*/	

// Configuration Step 1: Create and/or configure secrets.h. Use secrets_template.h as guide to create secrets.h

// Configuration Step 2: Set debug message output
// comment out to turn off; 1 = summary, 2 = verbose
// #define DEBUG 2

// Configuration Step 3: simulate WiFi and sensor hardware,
// returning random but plausible values
// comment out to turn off
// #define HARDWARE_SIMULATE

// Configuration Step 4: Set network data endpoints
// #define MQTT         // log sensor data to MQTT broker
// #define HASSIO_MQTT  // And, if MQTT enabled, with Home Assistant too?
#define INFLUX       // Log data to remote InfluxDB server

// Configuration Step 5: Select environment sensor
#define BME280	// use BME280 to read temperature and humidity
// #define AHTX0		// use AHTX0 sensor

// Configuration Step 6: Set battery size, if applicable
// If LC709203F detected on i2c, define battery pack based on settings curve from datasheet
// #define BATTERY_APA 0x08 // 100mAH
// #define BATTERY_APA 0x0B // 200mAH
// #define BATTERY_APA 0x10 // 500mAH
// #define BATTERY_APA 0x19 // 1000mAH
// #define BATTERY_APA 0x1D // 1200mAH
#define BATTERY_APA 0x2D // 2000mAH
// #define BATTERY_APA 0x32 // 2500mAH
// #define BATTERY_APA 0x36 // 3000mAH

// Configuration variables that change rarely

// network endpoints
#ifdef INFLUX  
	// Specify Measurement to use with InfluxDB for sensor and device info
	const String influxEnvMeasurement = "weather";  // Used for environmental sensor data
	const String influxDevMeasurement =  "device";   // Used for logging device data (e.g. battery)
#endif

// Sample and reporting intervals
#ifdef DEBUG
  // time between samples in seconds
  const uint8_t sensorSampleInterval = 60;
  // number of samples to average
  const uint8_t sensorSampleSize = 2;
#else
  const uint8_t sensorSampleInterval = 300;
  const uint8_t sensorSampleSize = 6;
#endif

// Hardware

// Simulation boundary values
#ifdef HARDWARE_SIMULATE
  const uint16_t sensorTempMinF =      1400; // divided by 100.0 to give floats
  const uint16_t sensorTempMaxF =      14000;
  const uint16_t sensorHumidityMin =   0; // RH%, divided by 100.0 to give float
  const uint16_t sensorHumidityMax =   10000;

  const uint16_t batterySimVoltageMin = 370; // will be divided by 100.0 to give floats
  const uint16_t batterySimVoltageMax = 410;

  const uint8_t networkRSSIMin = 30;
  const uint8_t networkRSSIMax = 90;
#endif

// Sleep time in seconds if hardware error occurs
const uint8_t hardwareRebootInterval = 10;

//Battery 
// analog pin used to reading battery voltage
#define BATTERY_VOLTAGE_PIN A13
// number of analog pin reads sampled to average battery voltage
const uint8_t   batteryReadsPerSample = 5;
// battery charge level lookup table
const float batteryVoltageTable[101] = {
  3.200,  3.250,  3.300,  3.350,  3.400,  3.450,
  3.500,  3.550,  3.600,  3.650,  3.700,  3.703,
  3.706,  3.710,  3.713,  3.716,  3.719,  3.723,
  3.726,  3.729,  3.732,  3.735,  3.739,  3.742,
  3.745,  3.748,  3.752,  3.755,  3.758,  3.761,
  3.765,  3.768,  3.771,  3.774,  3.777,  3.781,
  3.784,  3.787,  3.790,  3.794,  3.797,  3.800,
  3.805,  3.811,  3.816,  3.821,  3.826,  3.832,
  3.837,  3.842,  3.847,  3.853,  3.858,  3.863,
  3.868,  3.874,  3.879,  3.884,  3.889,  3.895,
  3.900,  3.906,  3.911,  3.917,  3.922,  3.928,
  3.933,  3.939,  3.944,  3.950,  3.956,  3.961,
  3.967,  3.972,  3.978,  3.983,  3.989,  3.994,
  4.000,  4.008,  4.015,  4.023,  4.031,  4.038,
  4.046,  4.054,  4.062,  4.069,  4.077,  4.085,
  4.092,  4.100,  4.111,  4.122,  4.133,  4.144,
  4.156,  4.167,  4.178,  4.189,  4.200 };

// Network
// max connection attempts to network services
const uint8_t networkConnectAttemptLimit = 3;
// seconds between network service connect attempts
const uint8_t networkConnectAttemptInterval = 10;