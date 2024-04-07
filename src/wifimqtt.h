// All include libraries here
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "credentials.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include "time.h"
#include "esp_sntp.h"
#include <PubSubClient.h>
#include <EasyButton.h>
#include <Adafruit_BME280.h>

#define DEBUG false

// How we fix obtaining functions or vars set in the main.cpp
// Examples:
//  extern void function_name();
//  extern void pub_wifi_signal_strength(String topic);
//  extern const int batteryMonitorPin

/*!
 * Rounds a number to 2 decimal places example: round(3.14159) -> 3.14
 * @param value  Float value to round
 * @return  value with only two decimal places
 */
double round_json(double value)
{
  return (int)(value * 100 + 0.5) / 100.0;
}

/*!
 *   Sets array of  gpio pins a pinMode output on ESP32
 *   @param
 *   @returns void
 */
/*void setup_output_pins()
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Function: "));
    Serial.println(__FUNCTION__);
  }

  int output_pins[] = {4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 23, 25, 26, 27, 32, 33};
  int pinCount = sizeof(output_pins) / sizeof(output_pins[0]);

  if (DEBUG)
  {
    Serial.print(F("pinCount: "));
    Serial.println(pinCount);
  }

  for (int thisPin = 0; thisPin < pinCount; thisPin++)
  {
    Serial.print(F("thisPin: "));
    Serial.print(thisPin);
    Serial.print(" ");
    pinMode(output_pins[thisPin], OUTPUT);
    if (DEBUG)
    {
      Serial.print(F("PinMode: "));
      Serial.print(output_pins[thisPin]);
      Serial.print(F(" OUTPUT "));
    }

    digitalWrite(output_pins[thisPin], LOW);
    if (DEBUG)
    {
      Serial.print(F("state: "));
      Serial.println(digitalRead(output_pins[thisPin]));
    }
  }
}
*/

// Setup WiFi
String hostname;
String mac_address;
String network;
String ip_address;
String subnet_mask;
String gateway_ip;
String subnet_cidr;
String dns1_ip;
String dns2_ip;
int32_t rssi;
int32_t channel;
int wifi_connection_attempts_max = 120; // waits 60 secs before rebooting
int wifi_connection_attempts;
const int ledWiFiStatusPin = 2; // ESP32 Onboard Blue LED

/*!
 *  Connects to WiFi
 *   @param
 *   @returns void
 */
void setup_wifi()
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Function: "));
    Serial.println(__FUNCTION__);
  }

  Serial.println();
  Serial.print(F("Connecting to WiFi "));
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  pinMode(ledWiFiStatusPin, OUTPUT);
  wifi_connection_attempts = 0;

  while (WiFi.status() != WL_CONNECTED)
  {
    wifi_connection_attempts++;
    digitalWrite(ledWiFiStatusPin, HIGH);
    delay(50);
    digitalWrite(ledWiFiStatusPin, LOW);
    Serial.print("." + String(wifi_connection_attempts));

    if (wifi_connection_attempts >= wifi_connection_attempts_max)
    {
      Serial.println(F("...[FAILED] Rebooting"));
      delay(2000);
      ESP.restart(); // Reboot ESP
    }
    delay(500);
  }

  Serial.println(F("...[SUCCESS]"));
  Serial.print(F("Connection attempts "));
  Serial.print(wifi_connection_attempts);
  Serial.print(F(" of "));
  Serial.println(wifi_connection_attempts_max);
  hostname = WiFi.getHostname();
  mac_address = WiFi.macAddress();
  network = WiFi.networkID().toString();
  ip_address = WiFi.localIP().toString();
  subnet_mask = WiFi.subnetMask().toString();
  subnet_cidr = WiFi.subnetCIDR();
  gateway_ip = WiFi.gatewayIP().toString();
  dns1_ip = WiFi.dnsIP(0).toString();
  dns2_ip = WiFi.dnsIP(1).toString();
  rssi = WiFi.RSSI();
  channel = WiFi.channel();
  Serial.print(F("Hostname: "));
  Serial.println(hostname);
  Serial.print(F("MAC Address: "));
  Serial.println(mac_address);
  Serial.print(F("Network: "));
  Serial.println(network);
  Serial.print(F("IP Address: "));
  Serial.println(ip_address);
  Serial.print(F("Subnet Mask: "));
  Serial.println(subnet_mask);
  Serial.print(F("Subnet CIDR: "));
  Serial.println(subnet_cidr);
  Serial.print(F("Gateway IP: "));
  Serial.println(gateway_ip);
  Serial.print(F("DNS 1: "));
  Serial.println(dns1_ip);
  Serial.print(F("DNS 2: "));
  Serial.println(dns2_ip);
  Serial.print(F("RSSI: "));
  Serial.print(rssi);
  Serial.println(F(" dBm"));
  Serial.print(F("Channel: "));
  Serial.println(channel);
  digitalWrite(ledWiFiStatusPin, HIGH); // Signals we are connected!
}

// NTP Time
const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const char *ntpServer3 = "time-a-b.nist.gov	";
const char *time_zone = "MST7MDT,M3.2.0,M11.1.0";
String month, day, year, hour, minute, second;
String current_date, current_time, current_daytime;

/*!
 *   Retrieves current date from NTP
 *   @param
 *   @returns String
 */
String get_current_date()
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Function: "));
    Serial.println(__FUNCTION__);
  }
  /*
    Member    Type  Meaning Range
    tm_sec    int   seconds after the minute  0-61*
    tm_min    int   minutes after the hour    0-59
    tm_hour   int   hours since midnight      0-23
    tm_mday   int   day of the month          1-31
    tm_mon    int   months since January     0-11
    tm_year   int   years since 1900
    tm_wday   int   days since Sunday         0-6
    tm_yday   int   days since January 1      0-365
    tm_isdst  int   Daylight Saving Time flag
  */
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo))
  {
    if (DEBUG)
    {
      Serial.println(F("Failed to get NTP date...retrying"));
    }
    delay(1000);
  }

  int months_by_number[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  month = (months_by_number[timeinfo.tm_mon] < 10)
              ? "0" + String(months_by_number[timeinfo.tm_mon])
              : String(months_by_number[timeinfo.tm_mon]);
  day = (timeinfo.tm_mday < 10) ? "0" + String(timeinfo.tm_mday) : String(timeinfo.tm_mday);
  year = String((timeinfo.tm_year + 1900));

  current_date = (month + "/" + day + "/" + year);
  return current_date;
}

/*!
 *   Retrieves current time from NTP
 *   @param
 *   @returns String
 */
String get_current_time()
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Function: "));
    Serial.println(__FUNCTION__);
  }
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo))
  {
    if (DEBUG)
    {
      Serial.println(F("Failed to get NTP time...retrying"));
    }
    delay(1000);
  }

  hour = (timeinfo.tm_hour < 10) ? "0" + String(timeinfo.tm_hour) : String(timeinfo.tm_hour);
  minute = (timeinfo.tm_min < 10) ? "0" + String(timeinfo.tm_min) : String(timeinfo.tm_min);
  second = (timeinfo.tm_sec < 10) ? "0" + String(timeinfo.tm_sec) : String(timeinfo.tm_sec);
  year = String((timeinfo.tm_year + 1900));

  current_time = (hour + ":" + minute + ":" + second);
  return current_time;
}

/*!
 *   NTP Callback function (get's called when time adjusts via NTP)
 *   @param
 *   @returns void
 */
void timeavailable(struct timeval *t)
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Function: "));
    Serial.println(__FUNCTION__);
  }
  Serial.print(F("Received time adjustment from NTP! "));
  current_date = get_current_date();
  current_time = get_current_time();
  Serial.print(current_date);
  Serial.print(" ");
  Serial.println(current_time);
}

/*!
 *  Connects to NTP servers
 *   @param
 *   @returns void
 */
void setup_sntp()
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Function: "));
    Serial.println(__FUNCTION__);
  }

  Serial.println();
  Serial.print(F("NTP Time: "));
  sntp_set_time_sync_notification_cb(timeavailable); // set notification call-back function
  configTzTime(time_zone, ntpServer1, ntpServer2, ntpServer3);
  get_current_time(); // Request time to trigger callback above
}

// Setup MQTT
const char *mqtt_server = "10.0.1.254";
const int mqtt_server_port = 1883;
WiFiClient espClient;
PubSubClient client(espClient);
String client_id;
String root_topic = "home/devices/";    // Default for all devices, contains cmd, status sub-topics
String root_location = "home/hallway/"; // Default topic for all sensor reporting

/*!
 *  Connects to MQTT server
 *   @param
 *   @returns void
 */
void reconnect()
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Function: "));
    Serial.println(__FUNCTION__);
  }
  // Increase our buffer to send larger packet for json 256 is default
  // client.setBufferSize(2000);

  while (!client.connected())
  {
    Serial.print(F("Attempting connection to MQTT server ["));
    Serial.print(mqtt_server);
    Serial.print(F("] as user ["));
    Serial.print(mqtt_user);
    Serial.print(F("] "));
    Serial.print(F("client id "));
    Serial.print(F("["));
    String mac_name = mac_address;
    mac_name.replace(":", "");
    client_id = "esp32-";
    client_id += mac_name;
    client_id.toLowerCase();
    Serial.print(client_id);
    Serial.print(F("] "));

    String mqtt_subscribed_topics[] = {
        "" + root_topic + client_id + "/cmd/get_battery_level",
        "" + root_topic + client_id + "/cmd/get_wifi_signal_strength",
        "" + root_topic + client_id + "/cmd/get_bme280_data",
        "" + root_topic + client_id + "/cmd/set_gpio",
        "" + root_topic + client_id + "/cmd/reboot"};

    // Create json of all our info
    JsonDocument doc;
    doc["id"] = client_id;
    doc["date"] = get_current_date();
    doc["time"] = get_current_time();
    doc["system"]["model"] = String(ESP.getChipModel());
    doc["system"]["revision"] = String(ESP.getChipRevision());
    doc["system"]["cpu_mhz"] = String(getCpuFrequencyMhz());
    doc["system"]["cpu_cores"] = String(ESP.getChipCores());
    doc["system"]["xtal_crystal_mhz"] = String(getXtalFrequencyMhz());
    doc["system"]["apb_hz"] = String(getApbFrequency());
    doc["system"]["flash_mb"] = String((((ESP.getFlashChipSize() / 1024) / 1024)));
    doc["network"]["ssid"] = ssid;
    doc["network"]["hostname"] = hostname;
    doc["network"]["mac_address"] = mac_address;
    doc["network"]["ip_address"] = ip_address;
    doc["network"]["subnet_mask"] = subnet_mask;
    doc["network"]["subnet_cidr"] = subnet_cidr;
    doc["network"]["gateway_ip"] = gateway_ip;
    doc["network"]["dns1_ip"] = dns1_ip;
    doc["network"]["dns2_ip"] = dns2_ip;
    doc["network"]["channel"] = channel;
    doc["network"]["rssi"] = rssi;
    doc["mqtt"]["user"] = mqtt_user;
    doc["mqtt"]["server"] = mqtt_server;
    doc["mqtt"]["port"] = mqtt_server_port;
    doc["mqtt"]["root_publish_topic"] = root_location;

    if (client.connect(client_id.c_str(), mqtt_user, mqtt_password))
    {
      Serial.println(F("[SUCCESS]"));

      Serial.print(F("Root publishing topic for sensors: "));
      Serial.println(root_location);

      // Subscribe to all topics
      for (byte idx = 0; idx < sizeof(mqtt_subscribed_topics) / sizeof(mqtt_subscribed_topics[0]); idx++)
      {
        // Add to json
        doc["mqtt"]["subscribed_topics"][idx] = mqtt_subscribed_topics[idx];
        Serial.print(F("Subscribing to topic "));
        Serial.print("\"");
        Serial.print(mqtt_subscribed_topics[idx]);
        Serial.print("\" ");
        if (client.subscribe(mqtt_subscribed_topics[idx].c_str()))
        {
          Serial.println(F("[SUCCESS]"));
        }
        else
        {
          Serial.println(F("[FAILED]"));
        }

        Serial.print(F("Publishing to topic "));
        Serial.print("\"");
        Serial.print(mqtt_subscribed_topics[idx]);
        Serial.print("\" ");
        if (client.publish(mqtt_subscribed_topics[idx].c_str(), NULL))
        {
          Serial.println(F("[SUCCESS]"));
        }
        else
        {
          Serial.println(F("[FAILED]"));
        }
      }
    }
    else
    {
      Serial.print(F("[FAILED], rc="));
      Serial.print(client.state());
      Serial.println(F(" will try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay(5000);
    }

    // Publish our json info to MQTT Server
    char buffer[2000];
    serializeJson(doc, buffer);

    String myjson = String(buffer);
    int client_buffer_size = (myjson.length() + 100);
    client.setBufferSize(client_buffer_size);

    if (DEBUG)
    {
      Serial.printf("Client buffer size:\t%u\r\nJSON size:\t%u\r\nFree heap size:\t%u\r\n", client.getBufferSize(), myjson.length(), ESP.getFreeHeap());
    }

    String topic = root_topic + client_id;
    Serial.print("Publishing my info to topic \"" + topic + "\" ");
    if (client.publish(topic.c_str(), buffer))
    {
      Serial.print(F("[SUCCESS]"));
    }
    else
    {
      Serial.print(F("[FAILED]"));
    }

    Serial.println();

    if (DEBUG)
    {
      Serial.println(F("json"));
      serializeJsonPretty(doc, Serial);
      Serial.println();
    }
  }
}

/*!
 *  MQTT Publishes a specified msg on a specified topic with retain message option
 *   @param  topic      MQTT Topic to publish
 *   @returns void
 */
void pub_msg(String topic, String json, boolean retain_msg)
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Function: "));
    Serial.println(__FUNCTION__);
  }

  Serial.println();
  Serial.print(F("MQTT publish on topic "));
  Serial.print(topic);
  Serial.print(F("  msg: "));
  Serial.print(json);
  Serial.print(F("  publish: "));

  if (client.publish(topic.c_str(), json.c_str(), retain_msg))
  {
    Serial.println(F("[SUCCESS]"));
  }
  else
  {
    Serial.println(F("[FAILED]"));
  }
}

// Battery level monitor
const int batteryMonitorPin = 34; // Requires voltage divider circuit for 3.3V

/*!
 *  MQTT Publishes battery level
 *   @param  topic      MQTT Topic to publish
 *   @returns void
 */
void pub_battery_level(String topic)
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Function: "));
    Serial.println(__FUNCTION__);
  }

  int raw_analog = analogRead(batteryMonitorPin);
  int battery_level = map(raw_analog, 2500, 4095, 0, 100);
  battery_level = (battery_level < 0) ? 0 : battery_level;
  // float batteryVoltage = map(battery_level, 0.0f, 100.0f, 0.0f, 4.2f);

  // Create JSON document
  JsonDocument doc;
  doc["id"] = client_id;
  doc["date"] = get_current_date();
  doc["time"] = get_current_time();
  doc["battery"]["level"] = battery_level;
  doc["battery"]["raw_analog"] = raw_analog;
  // doc["battery"]["rand_num"] = random(100, 999);

  char buffer[256];
  serializeJson(doc, buffer);
  pub_msg(topic, buffer, false);
}

/*!
 *  MQTT Publishes wifi rssi aka wifi signal level
 *   @param  topic      MQTT Topic to publish
 *   @returns void
 */
void pub_wifi_signal_strength(String topic)
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Function: "));
    Serial.println(__FUNCTION__);
  }
  // Create JSON document
  JsonDocument doc;
  doc["id"] = client_id;
  doc["date"] = get_current_date();
  doc["time"] = get_current_time();
  doc["network"]["rssi"] = WiFi.RSSI();
  // doc["network"]["rand_num"] = random(100, 999);

  char buffer[256];
  serializeJson(doc, buffer);
  pub_msg(topic, buffer, false);
}

// BME280 Temp, Humd, Pressure Sensor
Adafruit_BME280 bme; // use I2C interface
uint8_t bme280_address = 0x76;
#define SEALEVELPRESSURE_HPA (1007)
float temperature_celsius, temperature, humidity, pressure, altitude_meters, altitude;

/*!
 * Gets bme280 sensor data
 * @param
 * @return
 */
void pub_bme280_data(String topic)
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Function: "));
    Serial.println(__FUNCTION__);
  }
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF);

  bme.takeForcedMeasurement();

  // BME Temperatures
  temperature_celsius = bme.readTemperature();
  temperature = (1.8 * temperature_celsius + 32);

  // BME Humidity
  humidity = bme.readHumidity();

  // BME Pressure
  pressure = (bme.readPressure() / 100.0F);

  // BME Altitude
  altitude_meters = (bme.readAltitude(SEALEVELPRESSURE_HPA));
  altitude = (altitude_meters * 3.28084);

  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Temperature = "));
    Serial.print(temperature);
    Serial.println(F(" *C"));

    Serial.print(F("Pressure = "));
    Serial.print(pressure);
    Serial.println(F(" hPa"));

    Serial.print(F("Approx. Altitude = "));
    Serial.print(altitude);
    Serial.println(" m");

    Serial.print(F("Humidity = "));
    Serial.print(humidity);
    Serial.println(F(" %"));
    Serial.println();
  }

  // Create JSON document
  JsonDocument doc;
  doc["id"] = client_id;
  doc["date"] = get_current_date();
  doc["time"] = get_current_time();
  doc["sensor"]["type"] = "BME280";
  doc["sensor"]["i2c_address"] = "0x" + String(bme280_address, HEX);
  doc["environment"]["temperature"] = round_json(temperature);
  doc["environment"]["temperature_celsius"] = round_json(temperature_celsius);
  doc["environment"]["humidity"] = round_json(humidity);
  doc["environment"]["altitude"] = round_json(altitude);
  doc["environment"]["altitude_m"] = round_json(altitude_meters);
  doc["environment"]["pressure_hPa"] = round_json(pressure);
  // doc["environment"]["rand_num"] = random(100, 999);

  char buffer[512];
  serializeJson(doc, buffer);

  Serial.println();
  pub_msg(topic, buffer, false);
}

/*!
 *  MQTT callback function if we need to recieve information from a subscribed topic
 *   @param  topic      MQTT callback topic
 *   @param  message    Received message
 *   @param  length     Message size
 *   @returns void
 */
void mqtt_callback(char *topic, byte *message, unsigned int length)
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Function: "));
    Serial.println(__FUNCTION__);
  }
  Serial.println();
  Serial.print(F("MQTT subscribe msg arrived on topic: "));
  Serial.print(topic);
  Serial.print(F("    msg: "));

  String messageTemp;
  for (int i = 0; i < length; i++)
  {
    // Serial.println((char)message[i]);
    messageTemp += (char)message[i];
  }

  if (!messageTemp.isEmpty())
  {
    Serial.print(messageTemp);
    Serial.print("  ");
  }
  else
  {
    Serial.print(F("null"));
  }

  // Check subscribed topics when message arrives

  // Check for cmd set gpio topic
  if (String(topic) == root_topic + client_id + "/cmd/set_gpio" && !messageTemp.isEmpty())
  {

    JsonDocument doc;
    deserializeJson(doc, messageTemp);
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, messageTemp);

    // Test if parsing succeeds.
    if (error)
    {
      Serial.println();
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    String host = doc["client_id"];
    String gpio_pin_mode = doc["gpio_pin_mode"];
    int gpio_pin = doc["gpio_pin"];
    int gpio_state = doc["gpio_state"];

    if (gpio_pin_mode == "OUTPUT")
    {
      pinMode(gpio_pin, OUTPUT);
    }

    if (gpio_pin_mode == "INPUT")
    {
      pinMode(gpio_pin, INPUT);
    }

    if (gpio_pin_mode == "INPUT_PULLDOWN")
    {
      pinMode(gpio_pin, INPUT_PULLDOWN);
    }

    if (gpio_pin_mode == "INPUT_PULLUP")
    {
      pinMode(gpio_pin, INPUT_PULLUP);
    }

    if (gpio_state == 1)
    {
      digitalWrite(gpio_pin, HIGH);
    }
    else if (gpio_state == 0)
    {
      digitalWrite(gpio_pin, LOW);
    }

    JsonDocument doc2;
    doc2["date"] = get_current_date();
    doc2["time"] = get_current_time();
    doc2["gpio_state"] = digitalRead(gpio_pin);
    doc2["gpio_pin_mode"] = gpio_pin_mode;

    char buffer[256];
    serializeJson(doc2, buffer);

    String reply_topic = root_topic + client_id + "/status/gpio" + gpio_pin;
    Serial.print("reply on topic: " + reply_topic);
    Serial.println();
    pub_msg(reply_topic, buffer, false);
  }

  // Reboot
  if (String(topic) == root_topic + client_id + "/cmd/reboot" && messageTemp == "1")
  {
    String msg = (get_current_date() + " " + get_current_time());
    String reply_topic = root_topic + client_id + "/status/reboot";
    Serial.print("reply on topic: " + reply_topic);
    Serial.println();
    pub_msg(reply_topic, msg, false);
    Serial.println();
    Serial.println(F("Rebooting..."));
    delay(2000);
    ESP.restart();
  }

  // BME280 Sensor
  if (String(topic) == root_topic + client_id + "/cmd/get_bme280_data" && messageTemp == "1")
  {
    String reply_topic = root_location + "environment";
    Serial.print("reply on topic: " + reply_topic);
    Serial.println();
    pub_bme280_data(reply_topic);
  }

  // Battery
  if (String(topic) == root_topic + client_id + "/cmd/get_battery_level" && messageTemp == "1")
  {
    String reply_topic = root_topic + client_id + "/status/battery_level";
    Serial.print("reply on topic: " + reply_topic);
    Serial.println();
    pub_battery_level(reply_topic);
  }

  // Wifi Signal Strength
  if (String(topic) == root_topic + client_id + "/cmd/get_wifi_signal_strength" && messageTemp == "1")
  {
    String reply_topic = root_topic + client_id + "/status/wifi_signal_strength";
    Serial.print("reply on topic: " + reply_topic);
    Serial.println();
    pub_wifi_signal_strength(reply_topic);
  }
  Serial.println(); // leave for blank messages
}

// Easy Button
EasyButton button(32); // Button 1

/*!
 * Button callback function to be called when button is pressed
 * @param
 * @return void
 */
void button_pressed() // Button 1
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print(F("Function: "));
    Serial.println(__FUNCTION__);
  }
  int gpio_pin = 16;
  digitalWrite(gpio_pin, !digitalRead(gpio_pin));
  int gpio_state = digitalRead(gpio_pin);

  // {"client_id":"button_toggle_gpio16","gpio_pin_mode":"OUTPUT","gpio_pin":16,"gpio_state":1}
  JsonDocument doc;
  doc["client_id"] = __FUNCTION__;
  doc["gpio_pin_mode"] = "OUTPUT";
  doc["gpio_pin"] = gpio_pin;
  doc["gpio_state"] = gpio_state;

  char buffer[256];
  serializeJson(doc, buffer);

  pub_msg(root_topic + client_id + "/cmd/set_gpio", buffer, true);
}
