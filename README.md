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
```
// MQTT
const char *mqtt_server = "10.0.1.254";
const int mqtt_server_port = 1883;
String root_topic = "home/devices/";    // Default for all devices, contains cmd, status sub-topics
String root_location = "home/hallway/"; // Default topic for all sensor reporting

// NTP Time
const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const char *ntpServer3 = "time.nist.gov";
const char *time_zone = "MST7MDT,M3.2.0,M11.1.0"; // Denver Colorado
```

Then enjoy life!
