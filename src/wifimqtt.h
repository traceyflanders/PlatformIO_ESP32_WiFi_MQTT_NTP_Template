// All include libraries here
#include <Arduino.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Wire.h>
#include "time.h"
#include "esp_sntp.h"
#include <PubSubClient.h>
#include "credentials.h"
// #include <EasyButton.h>
// #include <Adafruit_BME280.h>

// How we fix obtaining functions or vars set in the main.cpp
// Examples:
//  extern void pubbattery_level(String topic);
//  extern void pub_wifi_signal_strength(String topic);
//  extern const int batteryMonitorPin

// Battery level monitor
const int batteryMonitorPin = 34; // Requires voltage divider circuit for 3.3V

/*!
 * Rounds a number to 2 decimal places\
 * example: round(3.14159) -> 3.14
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
/*void set_output_pins()
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print("Function: ");
    Serial.println(__FUNCTION__);
  }

  int output_pins[] = {4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 23, 25, 26, 27, 32, 33};
  int pinCount = sizeof(output_pins) / sizeof(output_pins[0]);

  if (DEBUG)
  {
    Serial.print("pinCount: ");
    Serial.println(pinCount);
  }

  for (int thisPin = 0; thisPin < pinCount; thisPin++)
  {
    Serial.print("thisPin: ");
    Serial.print(thisPin);
    Serial.print(" ");
    pinMode(output_pins[thisPin], OUTPUT);
    if (DEBUG)
    {
      Serial.print("PinMode: ");
      Serial.print(output_pins[thisPin]);
      Serial.print(" OUTPUT ");
    }

    digitalWrite(output_pins[thisPin], LOW);
    if (DEBUG)
    {
      Serial.print("state: ");
      Serial.println(digitalRead(output_pins[thisPin]));
    }
  }
}
*/

// Setup WiFi
String hostname;
String mac_address;
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
    Serial.print("Function: ");
    Serial.println(__FUNCTION__);
  }

  Serial.println();
  Serial.print("Connecting to WiFi ");
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
      Serial.println("...[FAILED] Rebooting");
      delay(2000);
      ESP.restart(); // Reboot ESP
    }
    delay(500);
  }

  Serial.println("...[SUCCESS]");
  Serial.print("Connection attempts ");
  Serial.print(wifi_connection_attempts);
  Serial.print(" of ");
  Serial.println(wifi_connection_attempts_max);
  hostname = WiFi.getHostname();
  mac_address = WiFi.macAddress();
  ip_address = WiFi.localIP().toString();
  subnet_mask = WiFi.subnetMask().toString();
  subnet_cidr = WiFi.subnetCIDR();
  gateway_ip = WiFi.gatewayIP().toString();
  dns1_ip = WiFi.dnsIP(0).toString();
  dns2_ip = WiFi.dnsIP(1).toString();
  rssi = WiFi.RSSI();
  channel = WiFi.channel();
  Serial.print("Hostname: ");
  Serial.println(hostname);
  Serial.print("MAC Address: ");
  Serial.println(mac_address);
  Serial.print("Network: ");
  Serial.println(ip_address);
  Serial.print("IP Address: ");
  Serial.println(ip_address);
  Serial.print("Subnet Mask: ");
  Serial.println(subnet_mask);
  Serial.print("Subnet CIDR: ");
  Serial.println(subnet_cidr);
  Serial.print("Gateway IP: ");
  Serial.println(gateway_ip);
  Serial.print("DNS 1: ");
  Serial.println(dns1_ip);
  Serial.print("DNS 2: ");
  Serial.println(dns2_ip);
  Serial.print("RSSI: ");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.print("Channel: ");
  Serial.println(channel);
  digitalWrite(ledWiFiStatusPin, HIGH); // Signals we are connected!
}

// NTP Time
const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const char *ntpServer3 = "time.nist.gov";
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
  struct tm timeinfo;
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
  if (!getLocalTime(&timeinfo))
  {
    if (DEBUG)
    {
      Serial.println("No date available (yet)");
    }
    return "00/00/0000";
  }
  else
  {

    // Month
    int months_by_number[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    if (months_by_number[timeinfo.tm_mon] < 10)
    {
      month = "0";
      month += String(months_by_number[timeinfo.tm_mon]);
    }
    else
    {
      month = String(months_by_number[timeinfo.tm_mon]);
    }

    // Day
    if (timeinfo.tm_mday < 10)
    {
      day = "0";
      day += String(timeinfo.tm_mday);
    }
    else
    {
      day = String(timeinfo.tm_mday);
    }

    year = String((timeinfo.tm_year + 1900));

    current_date = (month + "/" + day + "/" + year);
    return current_date;
  }
}

/*!
 *   Retrieves current time from NTP
 *   @param
 *   @returns String
 */
String get_current_time()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    return "00:00:00";
  }
  else
  {

    // Hour
    if (timeinfo.tm_hour < 10)
    {
      hour = "0";
      hour += String(timeinfo.tm_hour);
    }
    else
    {
      hour = String(timeinfo.tm_hour);
    }

    // Minute
    if (timeinfo.tm_min < 10)
    {
      minute = "0";
      minute += String(timeinfo.tm_min);
    }
    else
    {
      minute = String(timeinfo.tm_min);
    }

    // Second
    if (timeinfo.tm_sec < 10)
    {
      second = "0";
      second += String(timeinfo.tm_sec);
    }
    else
    {
      second = String(timeinfo.tm_sec);
    }

    year = String((timeinfo.tm_year + 1900));

    current_time = (hour + ":" + minute + ":" + second);
    return current_time;
  }
}

/*!
 *   NTP Callback function (get's called when time adjusts via NTP)
 *   @param
 *   @returns void
 */
void timeavailable(struct timeval *t)
{
  // Serial.println(F("Got time adjustment from NTP!"));
  Serial.print(get_current_date());
  Serial.println(get_current_time());
}

/*!
 *  Connects to NTP servers
 *   @param
 *   @returns void
 */
void setup_sntp()
{
  Serial.print("Setup NTP time: ");
  sntp_set_time_sync_notification_cb(timeavailable); // set notification call-back function
  configTzTime(time_zone, ntpServer1, ntpServer2, ntpServer3);
  current_date = get_current_date();
  current_time = get_current_time();
  Serial.print(current_date);
  Serial.print(" ");
  Serial.print(current_time);
  Serial.println();
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
    Serial.print("Function: ");
    Serial.println(__FUNCTION__);
  }
  // Increase our buffer to send larger packet for json 256 is default
  // client.setBufferSize(2000);

  while (!client.connected())
  {
    Serial.print("Attempting connection to MQTT server [");
    Serial.print(mqtt_server);
    Serial.print("] as user [");
    Serial.print(mqtt_user);
    Serial.print("] ");
    Serial.print("client id ");
    Serial.print("[");
    String mac_name = mac_address;
    mac_name.replace(":", "");
    client_id = "esp32-";
    client_id += mac_name;
    client_id.toLowerCase();
    Serial.print(client_id);
    Serial.print("] ");

    String mqtt_subscribed_topics[] = {
        "" + root_topic + client_id + "/cmd/get_battery_level",
        "" + root_topic + client_id + "/cmd/get_wifi_signal_strength",
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
      Serial.println("[SUCCESS]");

      Serial.print("Root publishing topic for sensors: ");
      Serial.println(root_location);

      // Subscribe to all topics
      for (byte idx = 0; idx < sizeof(mqtt_subscribed_topics) / sizeof(mqtt_subscribed_topics[0]); idx++)
      {
        // Add to json
        doc["mqtt"]["subscribed_topics"][idx] = mqtt_subscribed_topics[idx];
        Serial.print("Subscribing to topic ");
        Serial.print("\"");
        Serial.print(mqtt_subscribed_topics[idx]);
        Serial.print("\" ");
        if (client.subscribe(mqtt_subscribed_topics[idx].c_str()))
        {
          Serial.println("[SUCCESS]");
        }
        else
        {
          Serial.println("[FAILED]");
        }

        Serial.print("Publishing to topic ");
        Serial.print("\"");
        Serial.print(mqtt_subscribed_topics[idx]);
        Serial.print("\" ");
        if (client.publish(mqtt_subscribed_topics[idx].c_str(), NULL))
        {
          Serial.println("[SUCCESS]");
        }
        else
        {
          Serial.println("[FAILED]");
        }
      }
    }
    else
    {
      Serial.print("[FAILED], rc=");
      Serial.print(client.state());
      Serial.println(" will try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }

    // Publish our json info to MQTT Server
    char buffer[2000];
    serializeJson(doc, buffer);

    String myjson = String(buffer);
    int client_buffer_size = (myjson.length() + 100);
    client.setBufferSize(client_buffer_size);

    Serial.printf("Client buffer size:\t%u\r\nJSON size:\t%u\r\nFree heap size:\t%u\r\n", client.getBufferSize(), myjson.length(), ESP.getFreeHeap());

    String topic = root_topic + client_id;
    Serial.print("Publishing my info to topic \"" + topic + "\" ");
    if (client.publish(topic.c_str(), buffer))
    {
      Serial.print("[SUCCESS]");
    }
    else
    {
      Serial.print("[FAILED]");
    }

    Serial.println();

    if (DEBUG)
    {
      Serial.println("json");
      serializeJsonPretty(doc, Serial);
      Serial.println();
    }
  }
}

/*!
 *  MQTT Publishes a json msg
 *   @param  topic      MQTT Topic to publish
 *   @returns void
 */
void pub_json_msg(String topic, String json)
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print("Function: ");
    Serial.println(__FUNCTION__);
  }

  Serial.print("MQTT publish on topic ");
  Serial.print(topic);
  Serial.print("  msg: ");
  Serial.print(json);
  Serial.print("  publish: ");

  if (client.publish(topic.c_str(), json.c_str()))
  {
    Serial.print("[SUCCESS]");
  }
  else
  {
    Serial.print("[FAILED]");
  }
  Serial.println();
}

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
    Serial.print("Function: ");
    Serial.println(__FUNCTION__);
  }

  int raw_analog = analogRead(batteryMonitorPin);
  int battery_level = map(raw_analog, 2500, 4095, 0, 100);
  // float batteryVoltage = map(battery_level, 0.0f, 100.0f, 0.0f, 4.2f);
  Serial.print("MQTT publish on topic ");
  Serial.print(topic);
  Serial.print("  msg: ");
  if (battery_level < 0)
  {
    battery_level = 0;
  }
  Serial.print(battery_level);
  // Serial.print("  raw_analog: ");
  // Serial.print(raw_analog);
  Serial.print("  publish: ");

  // Create JSON document
  JsonDocument doc;
  doc["id"] = client_id;
  doc["date"] = get_current_date();
  doc["time"] = get_current_time();
  doc["system"]["battery_level"] = battery_level;
  doc["system"]["raw_analog"] = raw_analog;
  // doc["system"]["rand_num"] = random(100, 999);

  char buffer[256];
  serializeJson(doc, buffer);
  pub_json_msg(topic, buffer);
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
    Serial.print("Function: ");
    Serial.println(__FUNCTION__);
  }
  rssi = WiFi.RSSI();
  Serial.print("MQTT publish on topic: ");
  Serial.print(topic);
  Serial.print("  msg: ");
  Serial.print(rssi);
  Serial.print("  publish: ");

  // Create JSON document
  JsonDocument doc;
  doc["id"] = client_id;
  doc["date"] = get_current_date();
  doc["time"] = get_current_time();
  doc["system"]["rssi"] = rssi;
  // doc["system"]["rand_num"] = random(100, 999);

  char buffer[256];
  serializeJson(doc, buffer);
  pub_json_msg(topic, buffer);
}

/*!
 *  MQTT callback function if we need to recieve information from a subscribed topic
 *   @param  topic      MQTT callback topic
 *   @param  message    Received message
 *   @param  length     Message size
 *   @returns void
 */
void callback(char *topic, byte *message, unsigned int length)
{
  if (DEBUG)
  {
    Serial.println();
    Serial.print("Function: ");
    Serial.println(__FUNCTION__);
  }

  Serial.print("MQTT subscribe msg arrived on topic: ");
  Serial.print(topic);
  Serial.print("    msg: ");
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
    Serial.print("null");
  }

  // Check subscribed topics when message arrives

  if (String(topic) == root_topic + client_id + "/cmd/set_gpio" && !messageTemp.isEmpty())
  {

    JsonDocument doc;
    deserializeJson(doc, messageTemp);
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, messageTemp);

    // Test if parsing succeeds.
    if (error)
    {
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

    pub_json_msg(root_topic + client_id + "/status/gpio" + gpio_pin, buffer);
  }

  // Reboot
  if (String(topic) == root_topic + client_id + "/cmd/reboot" && messageTemp == "1")
  {
    pub_json_msg(root_topic + client_id + "/status/reboot", messageTemp);
    Serial.println("Rebooting...");
    delay(2000);
    ESP.restart();
  }

  // Battery
  if (String(topic) == root_topic + client_id + "/cmd/get_battery_level" && messageTemp == "1")
  {
    Serial.print("reply on topic: " + root_topic + client_id + "/status/battery_level");
    Serial.println();
    // Serial.print(messageTemp);
    pub_battery_level(root_topic + client_id + "/status/battery_level");
  }

  // Wifi Signal Strength
  if (String(topic) == root_topic + client_id + "/cmd/get_wifi_signal_strength" && messageTemp == "1")
  {

    Serial.print("reply on topic: " + root_topic + client_id + "/status/wifi_signal_strength");
    Serial.println();
    pub_wifi_signal_strength(root_topic + client_id + "/status/wifi_signal_strength");
  }
  Serial.println();
}