#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include <BH1750.h>
#include <DHT.h>
#include <Adafruit_BME280.h>

#include "device_id.h"
#include "mqtt.h"


const String TOPIC_PREFIX = "higrow_plant_monitor/";
const String TOPIC_SUFFIX = "/state";
const int DEEP_SLEEP_MINUTES = 20; // CHANGE ME TO 20
const int DEEP_SLEEP_SECONDS = 60 * DEEP_SLEEP_MINUTES;

const int WATER_MEASUREMENTS = 10;
const int WATER_MEASURE_INTERVAL_MS = 200;

// The following values were calibrated using a glass of water
const int NO_WATER = 3323;
const int FULL_WATER = 1389;

String device_id;
String client_id;
String topic;

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);

const int WIFI_CONNECT_TIMEOUT_SECONDS = 10;


// Simple ds18b20 class
class DS18B20
{
public:
    DS18B20(int gpio)
    {
        pin = gpio;
    }

    float temp()
    {
        uint8_t arr[2] = {0};
        if (reset()) {
            wByte(0xCC);
            wByte(0x44);
            delay(750);
            reset();
            wByte(0xCC);
            wByte(0xBE);
            arr[0] = rByte();
            arr[1] = rByte();
            reset();
            return (float)(arr[0] + (arr[1] * 256)) / 16;
        }
        return 0;
    }
private:
    int pin;

    void write(uint8_t bit)
    {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        delayMicroseconds(5);
        if (bit)digitalWrite(pin, HIGH);
        delayMicroseconds(80);
        digitalWrite(pin, HIGH);
    }

    uint8_t read()
    {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        delayMicroseconds(2);
        digitalWrite(pin, HIGH);
        delayMicroseconds(15);
        pinMode(pin, INPUT);
        return digitalRead(pin);
    }

    void wByte(uint8_t bytes)
    {
        for (int i = 0; i < 8; ++i) {
            write((bytes >> i) & 1);
        }
        delayMicroseconds(100);
    }

    uint8_t rByte()
    {
        uint8_t r = 0;
        for (int i = 0; i < 8; ++i) {
            if (read()) r |= 1 << i;
            delayMicroseconds(15);
        }
        return r;
    }

    bool reset()
    {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        delayMicroseconds(500);
        digitalWrite(pin, HIGH);
        pinMode(pin, INPUT);
        delayMicroseconds(500);
        return digitalRead(pin);
    }
};

#define I2C_SDA             25
#define I2C_SCL             26
#define DHT_PIN             16 //16?
#define BAT_ADC             33
#define SALT_PIN            34
#define SOIL_PIN            32
#define BOOT_PIN            0
#define POWER_CTRL          4
#define USER_BUTTON         35

BH1750 lightMeter(0x23);      //0x23 
Adafruit_BME280 bmp;      //0x77
DHT dht(DHT_PIN, DHT12);

bool bme_found = false;

boolean connect_wifi() {
  Serial.print(F("Connecting to wifi "));
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int i=0; i<WIFI_CONNECT_TIMEOUT_SECONDS && WiFi.status() != WL_CONNECTED; i++) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("Connected to "));
    Serial.println(WIFI_SSID);
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println(F("Failed to connect to WiFi"));
    return false;
  }
}


void print_info() {
  Serial.println(F("------------------------------------------------------------"));
  Serial.print(F("Device id:      "));
  Serial.println(device_id);
  Serial.print(F("Mqtt broker:    "));
  Serial.print(MQTT_BROKER_HOST);
  Serial.print(F(":"));
  Serial.println(MQTT_BROKER_PORT);
  Serial.print(F("Mqtt topic:     "));
  Serial.println(topic);
  Serial.print(F("Mqtt client id: "));
  Serial.println(client_id);
  Serial.println(F("------------------------------------------------------------"));
}

void setup() {
  Serial.begin(115200);
  Serial.println("");

  //device setup
  Wire.begin(I2C_SDA, I2C_SCL);

  dht.begin();

  pinMode(POWER_CTRL, OUTPUT);
  digitalWrite(POWER_CTRL, 1);
  delay(1000);

  if (!bmp.begin()) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
      bme_found = false;
  } else {
      bme_found = true;
  }

  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
      Serial.println(F("BH1750 Advanced begin"));
  } else {
      Serial.println(F("Error initialising BH1750"));
  }

  while(isnan(dht.readTemperature(true))){
    Serial.println(dht.read());
    Serial.println(dht.readTemperature()); /// rage...
    Serial.println(dht.readHumidity());
    Serial.println(bmp.readTemperature());
    sleep(1);
  }

  Serial.println(F("Hello :)\nHigrow MQTT sender\nhttps://github.com/tom-mi/higrow-mqtt-sender"));

  device_id = get_device_id();
  client_id = "higrow_" + device_id;
  topic = TOPIC_PREFIX + device_id + TOPIC_SUFFIX;

  print_info();

  connect_wifi();
  mqtt_client.setServer(MQTT_BROKER_HOST, String(MQTT_BROKER_PORT).toInt());

}

uint32_t readSalt()
{
    uint8_t samples = 120;
    uint32_t humi = 0;
    uint16_t array[120];

    for (int i = 0; i < samples; i++) {
        array[i] = analogRead(SALT_PIN);
        delay(2);
    }
    std::sort(array, array + samples);
    for (int i = 0; i < samples; i++) {
        if (i == 0 || i == samples - 1)continue;
        humi += array[i];
    }
    humi /= samples - 2;
    return humi;
}

uint16_t readSoil()
{
    uint16_t soil = analogRead(SOIL_PIN);
    return map(soil, 0, 4095, 100, 0);
}

float readBattery()
{
    int vref = 1100;
    uint16_t volt = analogRead(BAT_ADC);
    float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
    return battery_voltage;
}

void read_and_send_data() {
  DynamicJsonDocument root(1024);
  root["device_id"] = device_id;
  root["temperature_dht_celsius"] = dht.readTemperature();
  root["humidity_dht_percent"] = dht.readHumidity();
  if (bme_found){
    root["temperature_bme_celsius"] = bmp.readTemperature();
    root["humidity_bme_percent"] = bmp.readHumidity();
    root["pressure_bme_pascal"] = bmp.readPressure();
  }
  root["water"] = readSoil();
  root["salt"] = readSalt();
  root["light"] = lightMeter.readLightLevel();
  root["battery"] = readBattery();

  Serial.print(F("Sending payload: "));
  serializeJson(root, Serial);
  Serial.println("");
  if(reconnect_mqtt(mqtt_client, client_id.c_str())) {
    String payload;
    serializeJson(root, payload);
    if (mqtt_client.beginPublish(topic.c_str(), payload.length(), true)) {
      mqtt_client.print(payload);
      mqtt_client.endPublish();
      mqtt_client.disconnect();
      Serial.println("Success sending message");
    } else {
        Serial.println("Error sending message");
    }
  }
}

void loop() {
  read_and_send_data();
  Serial.println(F("Flushing wifi client"));
  wifi_client.flush();
  Serial.println(F("Disconnecting wifi"));
  WiFi.disconnect(true, false);

  delay(500);  // Wait a bit to ensure message sending is complete
  Serial.print(F("Going to sleep after "));
  Serial.print(millis());
  Serial.println(F("ms"));
  Serial.flush();
  //pinMode(I2C_SDA,INPUT); // needed because Wire.end() enables pullups, power Saving
  //pinMode(I2C_SCL,INPUT);
  ESP.deepSleep(DEEP_SLEEP_SECONDS * 1000000);
}
