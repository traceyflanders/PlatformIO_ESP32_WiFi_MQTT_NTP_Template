/*
 ESP32 Platform
*/
#define DEBUG true

// Must have includes
#include "wifimqtt.h"

unsigned long previousMillis = millis();
unsigned long previousMillis2 = millis();
int update_in_seconds = 15;
int update_in_seconds2 = 5;

void setup()
{
    Serial.begin(115200);

    // Set all output pins
    // set_output_pins();

    // Wifi
    setup_wifi();

    // NTP Time
    setup_sntp();

    // MQTT
    client.setServer(mqtt_server, mqtt_server_port);
    client.setCallback(callback);
}

void loop()
{
    // MQTT loops
    if (!client.connected())
    {
        reconnect();
    }

    client.loop();

    // Get current time in ms
    unsigned long currentMillis = millis();

    // Run something every update_loop_ms seconds
    if (currentMillis - previousMillis > (1000 * update_in_seconds))
    {
        previousMillis = currentMillis;

        // Fake sensors
        String topic;
        topic = root_location + "environment";

        // Fake Temp
        char tempString[8];
        int temperature = random(66, 100);
        dtostrf(temperature, 1, 2, tempString);

        // Fake Humidity
        char humString[8];
        int humidity = random(30, 80);
        dtostrf(humidity, 1, 2, humString);

        // Create JSON document
        JsonDocument doc;
        doc["id"] = client_id;
        doc["environment"]["temperature"] = temperature;
        doc["environment"]["humidity"] = humidity;
        doc["environment"]["rand_num"] = random(100, 999);

        char buffer[256];
        serializeJson(doc, buffer);

        Serial.print("MQTT publish on topic ");
        Serial.print(topic);
        Serial.print("  msg: ");
        serializeJson(doc, Serial);
        Serial.print("  publish: ");

        if (client.publish(topic.c_str(), buffer))
        {

            Serial.print("[SUCCESS]");
        }
        else
        {
            Serial.print("[FAILED]");
        }
        Serial.println();
    }

    // Get current time in ms
    unsigned long currentMillis2 = millis();

    // Run something every update_loop_ms seconds
    if (currentMillis2 - previousMillis2 > (1000 * update_in_seconds2))
    {
        previousMillis2 = currentMillis2;
        Serial.print(get_current_date());
        Serial.println(get_current_time());
    }
}