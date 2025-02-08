# PlatformIO_ESP32_WiFi_MQTT_NTP_Template
Platform IO Project for ESP32 using WiFI, MQTT, NTP

### Steps
Rename `src/default_credentials.h` to `credentials.h` with your wifi and mqtt usernames and passwords.
```
// WiFi
const char *ssid = "your-wifi-ssid";
const char *password = "your-wifi-password";

// MQTT
const char *mqtt_user = "your-username";
const char *mqtt_password = "super-secret-password";
```

Inside `src/wifimqtt.h` config file replace the following to suit your needs.
Decide if you want encrypted MQTT communication (off by default). Enable it for remote sensors that connect to your mqtt server over the internet. You'll also need to configure your mosquitto server for SSL. I use free certificates from [letsencrypt.org](https://letsencrypt.org/)
```
// #define ENABLE_SSL // Uncomment to enable secure MQTT config

// Setup MQTT
#ifdef ENABLE_SSL // MQTT SSL config
const char *mqtt_server = "broker.xxxxxx.com";
const int mqtt_server_port = 8883;
WiFiClientSecure espClient;
#endif

#ifndef ENABLE_SSL // Default MQTT config
const char *mqtt_server = "10.0.1.254";
const int mqtt_server_port = 1883;
WiFiClient espClient;
#endif

// MQTT root pub topics
PubSubClient client(espClient);
String client_id;
String root_topic = "home/devices"; // Default for all devices, contains cmd, status sub-topics
String root_location = "home/bedroom"; // Default topic for this sensor's reporting

// NTP Time
const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const char *ntpServer3 = "time.nist.gov";
const char *time_zone = "MST7MDT,M3.2.0,M11.1.0"; // Denver Colorado
```

Then enjoy life!
