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
#ifdef ENABLE_SSL
    espClient.setCACert(letsencrypt_root_ca); // SSL Root CA
#endif
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

    client.loop();

    // Buttons
    button.read();

    // Get current time in ms
    unsigned long currentMillis = millis();

    // Run something every update_loop_ms seconds
    if (currentMillis - previousMillis > (1000 * update_in_seconds))
    {
        previousMillis = currentMillis;
        pub_bme280_data(root_location + "/environment");
        // pub_fake_sensor_data(root_location + "/environment");
    }

    // NTP Time Loop
    // Get current time in ms
    unsigned long currentMillis2 = millis();

    // Run something every update_loop_ms seconds
    if (currentMillis2 - previousMillis2 > (1000 * update_in_seconds2))
    {
        previousMillis2 = currentMillis2;
        // Serial.print(get_current_date());
        // Serial.print(" ");
        // Serial.println(get_current_time());
    }
}