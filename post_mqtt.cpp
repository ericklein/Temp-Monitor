/*
  Project:      Temp-Monitor
  Description:  write sensor data to MQTT broker
*/

#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// Overall data and metadata naming scheme
#include "data.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// required external functions and data structures
extern void debugMessage(String messageText, uint8_t messageLevel);

#ifdef HASSIO_MQTT
  extern void hassio_mqtt_setup();
  extern void hassio_mqtt_publish(float temperatureF, float humidity, float batteryVoltage);
#endif

#ifdef MQTT
  // MQTT setup
  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  extern Adafruit_MQTT_Client aq_mqtt;

  void mqttConnect()
  // Connects and reconnects to MQTT broker, call as needed to maintain connection
  {  
    // exit if already connected
    if (aq_mqtt.connected())
    {
      debugMessage(String("Already connected to MQTT broker ") + MQTT_BROKER,2);
      return;
    }

    int8_t mqttErr;

    for(int tries = 1; tries <= networkConnectAttemptLimit; tries++)
    {
      if ((mqttErr = aq_mqtt.connect()) == 0)
      {
        debugMessage(String("Connected to MQTT broker ") + MQTT_BROKER,2);
        return;
      }
      aq_mqtt.disconnect();
      debugMessage(String("MQTT connection attempt ") + tries + " of " + networkConnectAttemptLimit + " failed with error msg: " + aq_mqtt.connectErrorString(mqttErr),1);
      delay(networkConnectAttemptInterval*1000);
    }
  }

  // Utility function to streamline dynamically generating MQTT topics using site and device 
  // parameters defined in config.h and our standard naming scheme using values set in data.h
  String generateTopic(char *key)
  {
    String topic;
    topic = String(DEVICE_SITE) + "/" + String(DEVICE_LOCATION) + "/" + String(DEVICE_ROOM) +
            "/" + String(DEVICE) + "/" + String(key);
    debugMessage(String("Generated MQTT topic: ") + topic,2);
    return(topic);
  }

  bool mqttDeviceBatteryUpdate(float batteryVoltage)
  {
    bool result = false;
    if (batteryVoltage>0)
    {
      String topic;
      topic = generateTopic(VALUE_KEY_BATTERY_VOLTS);  // Generate topic using config.h and data.h parameters
      // add ,MQTT_QOS_1); if problematic, remove QOS parameter
      Adafruit_MQTT_Publish batteryVoltagePub = Adafruit_MQTT_Publish(&aq_mqtt,topic.c_str());
      
      mqttConnect();

      // publish battery voltage
      if (batteryVoltagePub.publish(batteryVoltage))
      {
        debugMessage("MQTT publish: Battery Voltage succeeded",2);
        result = true;
      }
      else
      {
        debugMessage("MQTT publish: Battery Voltage failed",2);
      }
    }
    return(result);
  }

  bool mqttDeviceWiFiUpdate(uint8_t rssi)
  {
    bool result = false;
    if (rssi!=0)
    {
      String topic;
      topic = generateTopic(VALUE_KEY_RSSI);  // Generate topic using config.h and data.h parameters
      // add ,MQTT_QOS_1); if problematic, remove QOS parameter
      Adafruit_MQTT_Publish rssiLevelPub = Adafruit_MQTT_Publish(&aq_mqtt, topic.c_str());
      
      mqttConnect();

      if (rssiLevelPub.publish(rssi))
      {
        debugMessage("MQTT publish: WiFi RSSI succeeded",2);
        result = true;
      }
      else
      {
        debugMessage("MQTT publish: WiFi RSSI failed",2);
      }
    }
    return(result);
  }
  
  bool mqttSensorTemperatureFUpdate(float temperatureF)
  // Publishes temperature data to MQTT broker
  {
    bool result = false;
    String topic;
    topic = generateTopic(VALUE_KEY_TEMPERATURE);  // Generate topic using config.h and data.h parameters
    // add ,MQTT_QOS_1); if problematic, remove QOS parameter
    Adafruit_MQTT_Publish tempPub = Adafruit_MQTT_Publish(&aq_mqtt, topic.c_str());
    
    mqttConnect();

    // Attempt to publish sensor data
    if(tempPub.publish(temperatureF))
    {
      debugMessage("MQTT publish: Temperature succeeded",2);
      result = true;
    }
    else {
      debugMessage("MQTT publish: Temperature failed",2);
    }
    return(result);
  }

  bool mqttSensorHumidityUpdate(float humidity)
  // Publishes humidity data to MQTT broker
  {
    bool result = false;
    String topic;
    topic = generateTopic(VALUE_KEY_HUMIDITY);  // Generate topic using config.h and data.h parameters
    // add ,MQTT_QOS_1); if problematic, remove QOS parameter
    Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&aq_mqtt, topic.c_str());
    
    mqttConnect();
    
    // Attempt to publish sensor data
    if(humidityPub.publish(humidity))
    {
      debugMessage("MQTT publish: Humidity succeeded",2);
      result = true;
    }
    else {
      debugMessage("MQTT publish: Humidity failed",2);
    }
    return(result);
  }
#endif