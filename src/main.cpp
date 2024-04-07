/*
 ESP32 Platform
*/

// Must have includes
#include "wifimqtt.h"

// Loop timing
unsigned long previousMillis = millis();
unsigned long previousMillis2 = millis();
int update_in_seconds = 15;
int update_in_seconds2 = 1;

void setup()
{
    Serial.begin(115200);
    Serial.println(F("Setup Start"));

    // Set array of gpio pins to output
    // setup_output_pins();

    // Buttons
    Serial.println(F("Setup Easy Button"));
    button.begin();
    button.onPressed(button_pressed);

    // BME280 Sensor
    Serial.print(F("Setup BME280 Sensor: "));
    if (!bme.begin(bme280_address, &Wire))
    {
        Serial.println(F("[FAILED]"));
        while (1)
            delay(10);
    }
    else
    {
        Serial.println(F("[SUCCESS]"));
    }

    // Wifi
    setup_wifi();

    // NTP Time
    setup_sntp();

    // MQTT
    Serial.println(F("Setup MQTT"));
    client.setServer(mqtt_server, mqtt_server_port);
    client.setCallback(mqtt_callback);

    Serial.println(F("Setup End"));
}

void loop()
{
    // MQTT non-blocking
    if (!client.connected())
    {
        reconnect();
    }
    else
    {
        client.loop();
    }

    // Buttons
    button.read();

    // Get current time in ms
    unsigned long currentMillis = millis();

    // Run something every update_loop_ms seconds
    if (currentMillis - previousMillis > (1000 * update_in_seconds))
    {
        previousMillis = currentMillis;

        // Fake sensors
        String topic;
        topic = root_location + "/environment";

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