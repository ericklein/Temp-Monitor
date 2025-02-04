/*
  Project:      air_quality
  Description:  write sensor data to hassio_mqtt
*/

/*
 * Additional routines for use in an MQTT-enabled environment with Home Assistant, allowing
 * sensor readings to be reported to Home Assistant.   
 * 
 * Requires the following addition to configuration.yaml for Home Assistant, with the
 * state topic built from deployment parameters defined in config.h. Also, Unique IDs, 
 * which enable additional UI customization features for the sensors in Home Assistant,
 * can be generated here: https://www.uuidgenerator.net/version1.

   mqtt:
    sensor:
     - name: "Temperature"
      device_class: "temperature"
      state_topic: "{DEVICE_SITE}/{DEVICE}/{DEVICE_ID}/state"
      unit_of_measurement: "°F"
      unique_id: "-- GENERATE A UUID TO USE HERE --"
      value_template: "{{ value_json.temperature }}"
    - name: "Humidity"
      device_class: "humidity"
      state_topic: "{DEVICE_SITE}/{DEVICE}/{DEVICE_ID}/state"
      unit_of_measurement: "%"
      unique_id: "-- GENERATE A UUID TO USE HERE --"
      value_template: "{{ value_json.humidity }}"
    - name: "CO2"
      device_class: "carbon_dioxide"
      state_topic: "{DEVICE_SITE}/{DEVICE}/{DEVICE_ID}/state"
 */

#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// Shared helper function
extern void debugMessage(String messageText, int messageLevel);

#if defined MQTT && defined HASSIO_MQTT
    // MQTT setup
    #include <Adafruit_MQTT.h>
    #include <Adafruit_MQTT_Client.h>
    #include <ArduinoJson.h>
    extern Adafruit_MQTT_Client aq_mqtt;

    // The code below attempts to enable MQTT auto-discovery for Home Assistant,
    // but so far doesn't work to do that.  Instead this function isn't called from
    // post_mqtt(), and manual sensor configuration is enabled in the Home Assistant
    // configuration file.  See hassio_mqtt.cpp for more details.
    #define TCONFIG_TOPIC "homeassistant/sensor/aq-1T/config"
    #define HCONFIG_TOPIC "homeassistant/sensor/aq-1H/config"

    void hassio_mqtt_setup() {
        const int capacity = JSON_OBJECT_SIZE(5);
        StaticJsonDocument<capacity> doc;

        // Declare buffer to hold serialized object
        char output[1024];
        String topic;
        // Generate the device state topic for Home Assistant using deployment parameters from
        // config.h.  Note that this must match the state topic as specified in Home Assiatant's
        // configuration file (configuration.yaml) for this device.
        topic = String(DEVICE_SITE) + "/" + String(DEVICE) + "/" + String(DEVICE_ID) + "/state";

        // Create MQTT publish objects for the config channels
        Adafruit_MQTT_Publish tconfigPub = Adafruit_MQTT_Publish(&aq_mqtt,TCONFIG_TOPIC);
        Adafruit_MQTT_Publish hconfigPub = Adafruit_MQTT_Publish(&aq_mqtt,HCONFIG_TOPIC);

        debugMessage(String("Configuring RCO2 for Home Assistant MQTT auto-discovery"),1);
        // Create config info for temperature sensor
        doc["device_class"] = "temperature";
        doc["name"] = "Temperature";
        doc["state_topic"] = topic.c_str();
        doc["unit_of_measurement"] = "°F";
        doc["value_template"] = "{{ value_json.temperature}}";

        serializeJson(doc,output);
        // Publish temperature config to its topic (TCONFIG_TOPIC) as a retained message
        debugMessage(output,1);
        tconfigPub.publish(output,true);

        // Reset config data for humidity sensor
        doc["device_class"] = "humidity";
        doc["name"] = "Humidity";
        doc["state_topic"] = topic.c_str();
        doc["unit_of_measurement"] = "%";
        doc["value_template"] = "{{ value_json.humidity}}";

        serializeJson(doc,output);
        // Publish humidity config to its topic (HCONFIG_TOPIC) as a retained message
        debugMessage(output,1);
        hconfigPub.publish(output,true);
    }

    // Called to publish sensor readings as a JSON payload, as part of overall MQTT
    // publishing as implemented in post_mqtt().  Should only be invoked if 
    // Home Assistant MQTT integration is enabled in config.h.
    // Note that it depends on the value of the state topic matching what's in Home
    // Assistant's configuration file (configuration.yaml).
    void hassio_mqtt_publish(float temperatureF, float humidity, float batteryVoltage) {
        const int capacity = JSON_OBJECT_SIZE(4);
        StaticJsonDocument<capacity> doc;

        // Declare buffer to hold serialized object
        char output[1024];
        String topic;
        // Generate the device state topic for Home Assistant using deployment parameters from
        // config.h.  Note that this must match the state topic as specified in Home Assiatant's
        // configuration file (configuration.yaml) for this device.
        topic = String(DEVICE_SITE) + "/" + String(DEVICE) + "/" + String(DEVICE_ID) + "/state";
        Adafruit_MQTT_Publish aqStatePub = Adafruit_MQTT_Publish(&aq_mqtt,topic.c_str());

        debugMessage("Publishing values to Home Assistant via MQTT (topic below)",1);
        debugMessage(topic,1);

        doc["temperature"] = temperatureF;
        doc["humidity"] = humidity;
        doc["batteryVolts"] = batteryVoltage;

        serializeJson(doc,output);
        // Publish state info to its topic (MQTT_HASSIO_STATE)
        debugMessage(output,1);
        aqStatePub.publish(output);
    }
#endif