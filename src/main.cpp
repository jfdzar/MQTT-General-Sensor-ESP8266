/***************************************************
  General Sensor based on ESP8266

  The sensor will send a MQTT Message with the Sensors Values
 ****************************************************/
#include <Arduino.h>
#include <ESP8266WiFi.h>

#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

#include <DHT.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include "WiFiCredentials.h"

/************************* Adafruit.io Setup *********************************/
#define MQTT_FEED "living"

#define LEAK_SENSOR_PIN 5
#define LED_STATUS_PIN LED_BUILTIN
#define DHT_SENSOR_PIN 0

#define SENSOR_AVERAGE_SAMPLE 5
#define SENSOR_AVERAGE_WAIT_TIME 500

#define DEEP_SLEEP_TIME 300

#define ONE_WIRE_BUS 14

#define VOLTAGE_PIN 0

#define BUZZER_PIN 13

#define READ_SENSORS_PERIOD 3 * 60 * 1000

#define ALARM_MQTT_TIMEOUT 10000

#define ALARM_REPETITIONS 3
#define ALARM_DUTY_CYCLE 100
#define ALARM_WAITING 200

/************ Global Variables ******************/
boolean debug = true;

int i = 0;
bool status_leak = false;
bool previous_status_leak = false;

bool alarm = false;

DHT dht;
double T = 0;
double h = 0;
double v = 0;

double earth_T = 0;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds(&oneWire);

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish leak_status = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/" MQTT_FEED "/leak");
Adafruit_MQTT_Publish dallas_T = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/" MQTT_FEED "/earth_temperature");
Adafruit_MQTT_Publish dh_t = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/" MQTT_FEED "/temperature");
Adafruit_MQTT_Publish dh_h = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/" MQTT_FEED "/humidity");

// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe alarmsignal = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/" MQTT_FEED "/alarm");
Adafruit_MQTT_Subscribe alarmpwm = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/" MQTT_FEED "/alarmpwm");
Adafruit_MQTT_Subscribe alarmtimeout = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/" MQTT_FEED "/alarmtimeout");
Adafruit_MQTT_Subscribe alarmwait = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/" MQTT_FEED "/alarmwait");

/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void publishMsg(Adafruit_MQTT_Publish topic, const char *Msg)
{
  if (debug)
  {
    Serial.print(F("\nSending Topic Value "));
    Serial.print(Msg);
    Serial.print("...");
  }
  if (!topic.publish(Msg))
  {
    if (debug)
    {
      Serial.println(F("Failed"));
    }
  }
  else
  {
    if (debug)
    {
      Serial.println(F("OK!"));
    }
  }
}

void check_status_leak()
{
  status_leak = digitalRead(LEAK_SENSOR_PIN);
}

void readSensors()
{
  //bool local_debug = debug;
  bool local_debug = false;

  int h_count = 0;
  int T_count = 0;
  int v_count = 0;
  int earth_T_count = 0;
  float value_read = 0;

  dht.setup(DHT_SENSOR_PIN);
  h = 0;
  T = 0;

  ds.begin();
  ds.requestTemperatures();
  earth_T = 0;

  v = 0;

  for (int i = 1; i <= SENSOR_AVERAGE_SAMPLE; i++)
  {

    value_read = dht.getHumidity();
    if (!isnan(value_read))
    {
      h = h + value_read;
      h_count = h_count + 1;
    }
    if (local_debug)
    {
      Serial.println(h);
    }

    value_read = dht.getTemperature();
    if (!isnan(value_read))
    {
      T = T + value_read;
      T_count = T_count + 1;
    }
    if (local_debug)
    {
      Serial.println(T);
    }

    value_read = ds.getTempCByIndex(0);
    if (!isnan(value_read))
    {
      earth_T = earth_T + value_read;
      earth_T_count = earth_T_count + 1;
    }
    if (local_debug)
    {
      Serial.println(earth_T);
    }

    value_read = analogRead(VOLTAGE_PIN);
    if (!isnan(value_read))
    {
      v = v + value_read;
      v_count = v_count + 1;
    }
    if (local_debug)
    {
      Serial.println(v);
    }

    delay(SENSOR_AVERAGE_WAIT_TIME);
  }

  h = h / h_count;
  T = T / T_count;

  earth_T = earth_T / earth_T_count;
  v = v / v_count;

  if (local_debug)
  {
    Serial.println(h);
  }
  if (local_debug)
  {
    Serial.println(T);
  }
  if (local_debug)
  {
    Serial.println(earth_T_count);
  }
  if (local_debug)
  {
    Serial.println(v_count);
  }
}

void MQTT_connect()
{
  // Function to connect and reconnect as necessary to the MQTT server.
  // Should be called in the loop function and it will take care if connecting.
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected())
  {
    return;
  }

  if (debug)
  {
    Serial.print("Connecting to MQTT... ");
  }

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0)
  { // connect will return 0 for connected
    if (debug)
    {
      Serial.println(mqtt.connectErrorString(ret));
    }
    if (debug)
    {
      Serial.println("Retrying MQTT connection in 5 seconds...");
    }
    mqtt.disconnect();
    delay(5000); // wait 5 seconds
    retries--;
    if (retries == 0)
    {
      // changed to restart the micro
      ESP.restart();
      while (1)
        ;
    }
  }
  if (debug)
  {
    Serial.println("MQTT Connected!");
  }
}

void setup()
{

  Serial.begin(9600);

  //Set Pin Modes
  pinMode(LEAK_SENSOR_PIN, INPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_STATUS_PIN, HIGH);
  digitalWrite(LEAK_SENSOR_PIN, HIGH);

  //Short Buzzer Test on Reset
  /*
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  */

  delay(1000);

  if (debug)
  {
    Serial.println("Connecting to WiFi");
  }
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  int wifi_retries = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(250);
    digitalWrite(LED_STATUS_PIN, LOW);
    delay(250);
    wifi_retries = wifi_retries + 1;

    if (wifi_retries > 50)
    {
      if (debug)
      {
        Serial.println("Error Connecting - Restarting...");
      }
      ESP.restart();
    }
  }

  // Setup MQTT subscription for onoff & slider feed.
  mqtt.subscribe(&alarmsignal);
  mqtt.subscribe(&alarmpwm);
  mqtt.subscribe(&alarmtimeout);
  mqtt.subscribe(&alarmwait);

  digitalWrite(LED_STATUS_PIN, LOW);

  delay(1000);
}

void loop()
{

  unsigned long currentMillis = 0;
  unsigned long previousMillis = 0;

  unsigned long ReadSensors = READ_SENSORS_PERIOD;

  uint16_t AlarmDutyCycle = ALARM_DUTY_CYCLE;
  unsigned long AlarmTimeout = ALARM_MQTT_TIMEOUT;
  unsigned long AlarmWaiting = ALARM_WAITING;

  while (1)
  {

    currentMillis = millis();

    MQTT_connect();

    Serial.println(F("Reading Subscription.."));
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(AlarmTimeout)))
    {
      // Check if its the onoff button feed
      if (subscription == &alarmsignal)
      {
        Serial.print(F("Alarm State: "));
        Serial.println((char *)alarmsignal.lastread);

        if (strcmp((char *)alarmsignal.lastread, "ON") == 0)
        {
          digitalWrite(LED_STATUS_PIN, LOW);
          alarm = true;
        }
        if (strcmp((char *)alarmsignal.lastread, "OFF") == 0)
        {
          digitalWrite(LED_STATUS_PIN, HIGH);
          alarm = false;
        }
      }
      if (subscription == &alarmpwm)
      {
        Serial.print(F("PWM Duty Cycle: "));
        Serial.println((char *)alarmpwm.lastread);
        uint16_t alarmpwmval = atoi((char *)alarmpwm.lastread); // convert to a number
        AlarmDutyCycle = alarmpwmval;
      }
      if (subscription == &alarmwait)
      {
        Serial.print(F("Wait: "));
        Serial.println((char *)alarmwait.lastread);
        unsigned long alarmwaitval = atoi((char *)alarmwait.lastread); // convert to a number
        AlarmWaiting = alarmwaitval;
      }

      if (subscription == &alarmtimeout)
      {
        Serial.print(F("Timeout: "));
        Serial.println((char *)alarmtimeout.lastread);
        unsigned long alarmtimeoutval = atoi((char *)alarmtimeout.lastread); // convert to a number
        AlarmTimeout = alarmtimeoutval;
      }
    }

    if (alarm)
    {
      for (int i = 1; i <= ALARM_REPETITIONS; i++)
      {
        //digitalWrite(BUZZER_PIN, HIGH);
        analogWrite(BUZZER_PIN, AlarmDutyCycle);
        delay(AlarmWaiting);
        digitalWrite(BUZZER_PIN, LOW);
        delay(AlarmWaiting);
      }
    }

    if ((currentMillis - previousMillis >= ReadSensors))
    {
      previousMillis = currentMillis;
      if (debug)
      {
        Serial.println("Checking Sensors");
      }
      readSensors();
      char data[100];
      sprintf(data, "%f", T);
      if (debug)
      {
        Serial.print("Temperature: ");
      }
      if (debug)
      {
        Serial.println(T);
      }
      publishMsg(dh_t, data);

      sprintf(data, "%f", h);
      if (debug)
      {
        Serial.print("Humidity: ");
      }
      if (debug)
      {
        Serial.println(h);
      }
      publishMsg(dh_h, data);
      sprintf(data, "%f", earth_T);
      if (debug)
      {
        Serial.print("Earth Temperature: ");
      }
      if (debug)
      {
        Serial.println(earth_T);
      }
      publishMsg(dallas_T, data);
    }
  }
}