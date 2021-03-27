/*
  W. Hoogervorstm feb 2021 
  WiFi Smart plug firmware
  ESP8266 or ESP8285 MQTT Sonoff program
  Suitable for Sonoffs, BSD33, Sinilink
  Power measuring for BSD34 type modules
  version 17 published IP address and energy consumption when Relay is on for standby killing via Home Automation
  version 18 publish power when relay is switched off
  version 19: second LED off
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <TimeLib.h>
#include <credentials.h>	//mySSID, myPASSWORD, mqtt_server

extern "C" {
#include "user_interface.h"
}

uint32_t lastReconnectAttempt = 0, lastBlink = 0;
int resetcounter = 0;

// base values
const char* software_version = "version 22";
const char* devicename = "BSD34_2"; // 7 characters, then the length of the MQTT topic fits nicely

// for HTTPupdate
char host[20];
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

/*credentials & definitions */
//MQTT
char mqtt_id[20];
char* status_topic_a = "/status";
char* LED_topic_a = "/LED_sw";
char* LED_st_topic_a = "/LED_st";
char* RELAY_sw_topic_a = "/Relay_sw";
char* RELAY_st_topic_a = "/Relay_st";

char status_topic[20], LED_topic[20], LED_st_topic[20], RELAY_sw_topic[20], RELAY_st_topic[20];


#define SECOND 1       // in s
#define MINUTE 60*SECOND  // in s
#define STATUSSENDDELAY 30*MINUTE     // send status every ##

#define SONOFF  1
#define BSD33   2
#define BSD34   3 // Girier, power monitoring
#define SINILINK 4
#define NEOCOOLCAM 5  // small plug with power monitoring, NEO Coolcam NAS-WR01W Plug
#define SONOFF_MINI 6
#define BRAND BSD34 //SONOFF, BSD33, BSD34, SINILINK, NEOCOOLCAM, SONOFF_MINI

#define DEVICE_TYPE1 1    // Sonoff separate LEDs for status and relay, SonoffBasic R3, Sonoff S20, S26, Sinilink
#define DEVICE_TYPE2 2    // Sonoff only one LED, not connected to relay, but displays relay status, Sonoff-Basic 1, BSD33
#define DEVICE_TYPE3 3    // Sonoff separate LEDs for status and relay, Sonoff-Touch
#define DEVICE_TYPE4 4    // 2 Separate LEDs for status LED not connected to relay BSD34/Girier/NeoCoolcam

#define DEVICE DEVICE_TYPE4

#if (BRAND == BSD33)
#define BUTTONPIN 13
#define RELAYPIN 15
#define LEDPIN 2
#define LEDPIN2 LEDPIN
#define LEDON LOW
#define LEDOFF HIGH
#define CF_PIN 10

#elif (BRAND == BSD34)  // switch with power monitoring function
#define BUTTONPIN 13
#define RELAYPIN 12
#define LEDPIN 14
#define LEDPIN2 1
#define LEDON LOW
#define LEDOFF HIGH
//#define SEL_PIN 3
//#define CF1_PIN 5
#define CF_PIN 4

#elif (BRAND == NEOCOOLCAM)  // switch with power monitoring function
#define BUTTONPIN 0
#define RELAYPIN 14
#define LEDPIN 13
#define LEDPIN2 LEDPIN
#define LEDON LOW
#define LEDOFF HIGH
//#define SEL_PIN 12
//#define CF1_PIN 5
#define CF_PIN 4
#define MEASUREINTERVAL ( 5*1000 ) //in ms
#define POWERSENDDELAY 1*MINUTE     // send power data every ##

#elif (BRAND == SONOFF_MINI)
#define BUTTONPIN 4
#define RELAYPIN 12
#define LEDPIN 13
#define LEDPIN2 LEDPIN
#define CF_PIN 10
#define LEDON LOW
#define LEDOFF HIGH

#elif (BRAND == SONOFF)
#define BUTTONPIN 0
#define RELAYPIN 12
#define LEDPIN 13
#define LEDPIN2 LEDPIN
#define LEDON LOW
#define LEDOFF HIGH
#define CF_PIN 10

#elif (BRAND == SINILINK)
#define BUTTONPIN 4
#define RELAYPIN 5
//#define RELAYLEDPIN 14
#define LEDPIN 16
#define LEDPIN2 LEDPIN
#define LEDON HIGH
#define LEDOFF LOW
#define CF_PIN 10
#endif

//for power monitoring
#define MEASUREINTERVAL ( 5*1000 ) //in ms
#define POWERSENDDELAY 1*MINUTE     // send power data every ##
char* powerW_topic_a = "/powerW";
char* kWh_topic_a = "/kWh";
//char current_topic[20], voltage_topic[20];
char powerW_topic[20], kWh_topic[20];
// variables for measuring voltage, current and power
//long CF1_counts = 0;
long CF_counts = 0;
uint32_t power_starttime, lastCFpulse; // , volt_curr_starttime, lastCF1pulse;
boolean power_starttime_known = false, measurement = false; //, volt_curr_starttime_known = false, ;
long prev_CF_counts; //prev_CF1_counts,
float power_frequency; //, voltage_frequency, current_frequency;
float powerW; //, currentA;
#define POWER_MULTIPLIER 1.47 // transforms the measured frequency to power in W
float energy_consumption = 0;
//boolean CF_pulse, CF1_pulse;
uint32_t powertime;

#define SERIALDEBUG 0

#define RECONNECTDELAY_S 60
#define WIFI_CONNECT_TIMEOUT_S 25
//#define MAXRETRY 10

uint32_t statustime;

WiFiClient espClient;
PubSubClient client(espClient);

boolean messagespool;
boolean startup = true;


// RTC-MEM Adresses
#define RTC_ADDRESS 66

#define ON_VALUE    1 // value to write in RTC memory
#define OFF_VALUE   2 // value to write in RTC memory
#define RELAYVALUE 0 // position in byte of RTC memory
#define LEDVALUE   1 // position in byte of RTC memory
byte buf[2];          // bytes to read and write the values to

void ICACHE_RAM_ATTR CF_impulse();
//void ICACHE_RAM_ATTR CF1_impulse();

void setup() {
  pinMode(BUTTONPIN, INPUT_PULLUP);      // button switch, sinilink needs a internal pullup
  pinMode(LEDPIN, OUTPUT);     // Initialize the LED pin as an output
  if (DEVICE == DEVICE_TYPE4)   // set LED2PIN
    pinMode(LEDPIN2, OUTPUT);     // Initialize the LED pin as an output
  pinMode(RELAYPIN, OUTPUT);     // Initialize the RELAY pin as an output
  digitalWrite(RELAYPIN, LOW);
  digitalWrite(LEDPIN, LEDOFF);

  if (BRAND == BSD34 || BRAND == NEOCOOLCAM)
  {
    //pinMode(CF1_PIN, INPUT);                       // Set pin to input for capturing CF1_PIN pulses
    pinMode(CF_PIN, INPUT);                       // Set pin to input for capturing CF_PIN pulses
    interrupts();                                 // Enable interrupts (in case they were previously disabled)
    //attachInterrupt(digitalPinToInterrupt(CF1_PIN), CF1_impulse, RISING); // Define external interrupts
    attachInterrupt(digitalPinToInterrupt(CF_PIN), CF_impulse, RISING); // Define external interrupts
  }
  strcpy(mqtt_id, devicename);
  strcpy(host, devicename);

  strcpy(status_topic, devicename);
  strcpy(LED_topic, devicename);
  strcpy(LED_st_topic, devicename);
  strcpy(RELAY_sw_topic, devicename);
  strcpy(RELAY_st_topic, devicename);

  strcat (status_topic, status_topic_a);
  strcat (LED_topic, LED_topic_a);
  strcat (LED_st_topic, LED_st_topic_a);
  strcat (RELAY_sw_topic, RELAY_sw_topic_a);
  strcat (RELAY_st_topic, RELAY_st_topic_a);
  if (BRAND == BSD34 || BRAND == NEOCOOLCAM)
  {
    //strcpy(voltage_topic, devicename);
    //strcpy(current_topic, devicename);
    strcpy(powerW_topic, devicename);
    strcpy(kWh_topic, devicename);
    //strcat (voltage_topic, voltage_topic_a);
    //strcat (current_topic, current_topic_a);
    strcat (powerW_topic, powerW_topic_a);
    strcat (kWh_topic, kWh_topic_a);
  }

  if (SERIALDEBUG)  // initialize the serial port if needed for debug
  {
    Serial.begin(115200);
    Serial.println();
    Serial.println();
  }


  // check if relay state was stored in RTC memory
  system_rtc_mem_read(RTC_ADDRESS, buf, 2); // read 2 bytes from RTC-MEMORY
  Serial.println("");
  Serial.println("");
  Serial.print("check values:\t");
  Serial.print(ON_VALUE);
  Serial.print("\t");
  Serial.println(OFF_VALUE);

  Serial.print("read values:\t");

  Serial.print(buf[RELAYVALUE]);
  Serial.print("\t");
  Serial.println(buf[LEDVALUE]);

  if (((buf[0] == ON_VALUE) || (buf[0] == OFF_VALUE) && (buf[1] == ON_VALUE) || (buf[1] == OFF_VALUE)))  // values were stored
  {
    messagespool = true;
    Serial.println("values were stored, use them");
    if (buf[RELAYVALUE] == ON_VALUE)
      digitalWrite(RELAYPIN, HIGH);
    if (buf[RELAYVALUE] == OFF_VALUE)
      digitalWrite(RELAYPIN, LOW);
    if (buf[LEDVALUE] == ON_VALUE)
      digitalWrite(LEDPIN, LEDON);
    if (buf[LEDVALUE] == OFF_VALUE)
      digitalWrite(LEDPIN, LEDOFF);
  }
  else
    Serial.println("values were NOT stored, do not use them");

  WiFi.mode(WIFI_STA);
  connect_wifi();
  Serial.println("setup started");

  // for HTTPudate
  MDNS.begin(host);
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  httpServer.on("/", handleRoot);
  //httpServer.on("/", handle_OnConnect);
  httpServer.on("/ledon", handle_ledon);
  httpServer.on("/ledoff", handle_ledoff);
  httpServer.on("/relayon", handle_relayon);
  httpServer.on("/relayoff", handle_relayoff);
  httpServer.onNotFound(handle_NotFound);
  Serial.println("\n\nHTTP server started");

  // MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  if (!client.connected()) {
    reconnect();
  }
  //client.publish(LED_topic, "ON");
  //client.publish(RELAY_sw_topic, "OFF");
  statustime = now();

  Serial.println("setup ended");
}

void loop() {
  // first handle power, voltage and current measurement
  if (BRAND == BSD34 || BRAND == NEOCOOLCAM)
  {

    if (digitalRead(RELAYPIN) == HIGH && measurement != true)	//relay on so measure the power
    {
      measurement = true;
      client.publish(status_topic, "Relay switched on, measurement started");
      //CF_pulse = false;
    }
    if (digitalRead(RELAYPIN) == LOW && measurement == true)  //relay off so stop measuring the power
    {
      measurement = false;
      powerW = 0;
      publish_power();
      //client.publish(powerW_topic, "0");
      power_starttime_known = false;
    }
    if (measurement)
    {
      /*
         measure pulses without interrupt, but this is to slow
        if (digitalRead(CF_PIN) == HIGH && !CF_pulse) // CF pin is high, so pulse is received
        {
        CF_pulse = true;
        Serial.println("CF pulse = true");
        }
        if (digitalRead(CF_PIN) == LOW && CF_pulse)	// CF pin is LOW, pulse has ended
        {
        CF_pulse = false;
        Serial.println("CF pulse = false");
        CF_counts++;
        }
      */
      //power measurement
      if (CF_counts > 0 && power_starttime_known == false) // a pulse was registrated, so now start the measurement
      {
        power_starttime = millis();
        Serial.print("power starttime: ");
        Serial.println(power_starttime);
        power_starttime_known = true;
        CF_counts = 0;
        prev_CF_counts = 0;
      }
      else if (power_starttime_known)
      {
        if (CF_counts > prev_CF_counts) // a pulse was registrated
        {
          prev_CF_counts = CF_counts;
          lastCFpulse = millis();
        }
        if (millis() > MEASUREINTERVAL + power_starttime)	//measurement period has ended, calculate frequency
        {
          Serial.print("CF counts = ");
          Serial.println(CF_counts);
          if (CF_counts > 1)
          {
            uint32_t measurementtime = lastCFpulse - power_starttime;
            long powerpulses = CF_counts - 1;	//between first and last pulse are counts - 1 times the period between pulses
            power_frequency = (float) powerpulses * 1000 / (float) MEASUREINTERVAL;		// 1000 since time is measured in ms

            powerW = power_frequency * POWER_MULTIPLIER;  // transform measured frequency to power in W
            //consumption in kWh      (power *     elapsed time        ) / (W/kW * sec/min * min/hr * 1000 us/s
            energy_consumption = energy_consumption + (powerW * (float) MEASUREINTERVAL) / ((float) 1000 * 60 * 60 * 1000);

          }
          if (lastCFpulse > millis() - MEASUREINTERVAL) // lastpulse was in measurement period
          {
            power_starttime = lastCFpulse;
            Serial.println("power_starttime = lastCFpulse");
          }
          else
          {
            power_starttime_known = false;              // no recent pulse, start measurement again
            powerW = 0;
          }
          CF_counts = 0;
          prev_CF_counts = 0;
        }

        if (millis() < power_starttime)	// start again
        {
          power_starttime_known = false;
        }
      }
      // if desired, add voltage or current measuring here, take account for the needed switch time for stabilisation of the pulses
      // als SEL_MODE HIGH measuring voltage SEL_MODE LOW current
    }
  }
  if (client.connected())
  {
    // Client connected
    client.loop();
    httpServer.handleClient();    // for HTTPupdate
    if (messagespool)             // switch was switched while not connected, sent message now connected
    {
      if (digitalRead(RELAYPIN) == HIGH)
      {
        client.publish(RELAY_sw_topic, "ON");
      }
      else if (digitalRead(RELAYPIN) == LOW)
      {
        client.publish(RELAY_sw_topic, "OFF");
      }
      messagespool = false;
    }
    setLEDstate();
    if (digitalRead(BUTTONPIN) == LOW) // manual control of switch, communicate via mqtt
    {
      if (digitalRead(RELAYPIN)) {
        client.publish(RELAY_sw_topic, "OFF");
      }
      else
      {
        client.publish(RELAY_sw_topic, "ON");
      }
      while (digitalRead(BUTTONPIN) == LOW) // button is still pressed
      {
        delay(50);
        yield();
      }
    }

    // publish status value
    if (startup || now() > statustime + STATUSSENDDELAY)
    {
      publish_status();
      statustime = now();
      startup = false;
    }
    // publish power values when measuring
    if (measurement && now() > powertime + POWERSENDDELAY)
    {
      publish_power();
      powertime = now();
    }
  }
  else
    // Client is not connected
  {
    //long now = millis();
    //long now2 = millis();
    if (digitalRead(BUTTONPIN) == LOW) // manual control of switch when not connected
    {
      digitalWrite(RELAYPIN, !digitalRead(RELAYPIN));
      Serial.println("Relay is switched");
      messagespool = true;
    }

    while (digitalRead(BUTTONPIN) == LOW) // button is still pressed
    {
      delay(50);
      yield();
    }

    if (millis() - lastBlink > 500) {          // blink the LED while not connected to MQTT
      if (DEVICE == DEVICE_TYPE4)   //blink blueLED
        digitalWrite(LEDPIN2, !digitalRead(LEDPIN2));
      else
        digitalWrite(LEDPIN, !digitalRead(LEDPIN));
      Serial.println("blink loop");
      //Serial.println(digitalRead(BUTTONPIN));
      lastBlink = millis();
    }

    if (millis() - lastReconnectAttempt > RECONNECTDELAY_S * 1000) {
      lastReconnectAttempt = millis();
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
}


void connect_wifi() // start connection, but do not wait for connection
{
  uint32_t time1;
  delay(10);
  resetcounter++;
  //if (resetcounter > MAXRETRY)
  //  ESP.restart();
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(mySSID);
  WiFi.begin(mySSID, myPASSWORD);
  time1 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (digitalRead(BUTTONPIN) == LOW) // manual control of switch when not connected
    {
      digitalWrite(RELAYPIN, !digitalRead(RELAYPIN));
      messagespool = true;
      setLEDstate();
    }
    while (digitalRead(BUTTONPIN) == LOW) // button is still pressed
    {
      delay(50);
      yield();
    }
    yield();
    if (millis() - lastBlink > 500) {          // blink the LED while not connected
      if (DEVICE == DEVICE_TYPE4)   //blink blueLED
        digitalWrite(LEDPIN2, !digitalRead(LEDPIN2));
      else
        digitalWrite(LEDPIN, !digitalRead(LEDPIN));
      lastBlink = millis();
    }
    if (millis() > time1 + (WIFI_CONNECT_TIMEOUT_S * 1000))
    {
      /*
        system_rtc_mem_write(RTC_CHECK, buf, 2);
        // initialise values
        countbuf[0] = 0;
        system_rtc_mem_write(RTC_COUNT, countbuf, 1);     // set counter to 0
      */
      Serial.println("No connection in time, write the current values and restart");
      if (digitalRead(RELAYPIN) == HIGH)
        buf[RELAYVALUE] = ON_VALUE;
      if (digitalRead(RELAYPIN) == LOW)
        buf[RELAYVALUE] = OFF_VALUE;
      if (digitalRead(LEDPIN) == LEDON)
        buf[LEDVALUE] = ON_VALUE;
      if (digitalRead(LEDPIN) == LEDOFF)
        buf[LEDVALUE] = OFF_VALUE;

      system_rtc_mem_write(RTC_ADDRESS, buf, 2);

      delay(10);
      ESP.restart();
    }
  }
  if (DEVICE == DEVICE_TYPE4)
  {
    if (WiFi.status() == WL_CONNECTED)
      digitalWrite(LEDPIN2, LEDOFF);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(LEDPIN, LEDOFF);
}


boolean reconnect()
{
  if (WiFi.status() != WL_CONNECTED) {    // check if WiFi connection is present
    connect_wifi();
    return false;
  }
  //LED off
  if (DEVICE == DEVICE_TYPE4)   
    digitalWrite(LEDPIN2, LEDOFF);
  else
    digitalWrite(LEDPIN, LEDOFF);

  Serial.println("Attempting MQTT connection...");
  //if (client.connect(mqtt_id)) {
  if (client.connect(mqtt_id, RELAY_st_topic, 0, 0, "OFF")) {    // when disconnected switch will be off. connect(const char *id, const char* willTopic, uint8_t willQos, boolean willRetain, const char* willMessage)
    Serial.println("connected");
    // ... and resubscribe
    client.subscribe(LED_topic);
    client.subscribe(RELAY_sw_topic);
    client.publish(status_topic, "connected");
  }
  Serial.println(client.connected());
  return client.connected();
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // Switch on the LED if an 1 was received as first character
  if ((char)topic[8] == 'L')      // check for the "L" of sonoff#/LED_sw
  {
    if ((char)payload[1] == 'N' || (char)payload[0] == '1') // check for 'N' in ON or for 1
    {
      digitalWrite(LEDPIN, LEDON);   // Turn the LED on
      if (DEVICE == DEVICE_TYPE4)
        digitalWrite(LEDPIN2, LEDON);   // Turn the 2nd LED on
      // client.publish(LED_st_topic, "ON");
    }
    else if ((char)payload[1] == 'F' || (char)payload[0] == '0')  // check for 'F' in OFF or for 0
    {
      digitalWrite(LEDPIN, LEDOFF);  // Turn the LED off
      if (DEVICE == DEVICE_TYPE4)
        digitalWrite(LEDPIN2, LEDOFF);   // Turn the 2nd LED off
      // client.publish(LED_st_topic, "OFF");
    }
  }
  if ((char)topic[8] == 'R')    // check for the "R" of sonoff#/Relay_sw
  {
    if ((char)payload[1] == 'N' || (char)payload[0] == '1')  // check for 'N' in ON or for 1
    {
      digitalWrite(RELAYPIN, HIGH);   // Turn the RELAY on
      client.publish(RELAY_st_topic, "ON");
    }
    else if ((char)payload[1] == 'F' || (char)payload[0] == '0') // check for 'F' in OFF or for 0
    {
      digitalWrite(RELAYPIN, LOW) ;  // Turn the RELAY off
      client.publish(RELAY_st_topic, "OFF");
    }
    else if ((char)payload[0] == 'T' || (char)payload[0] == '2') // check for T in TOGGLE or for 2
    {
      digitalWrite(RELAYPIN, !digitalRead(RELAYPIN));  // Togle the relay
      if (digitalRead(RELAYPIN) == HIGH)
      {
        client.publish(RELAY_st_topic, "ON");
      }
      else if (digitalRead(RELAYPIN) == LOW)
      {
        client.publish(RELAY_st_topic, "OFF");
      }
    }
  }
}

void handleRoot() {
  Serial.println("Connected to client");
  httpServer.send(200, "text/html", SendHTML());
  /*
    String message = "Sonoff WimIOT\nDevice: ";
    message += mqtt_id;
    message += "\nSoftware version: ";
    message += software_version;
    message += "\nUpdatepath at http://[IP]/update";
    httpServer.send(200, "text/plain", message);
  */
}
void handle_OnConnect() {
  Serial.println("Connected to client");
  httpServer.send(200, "text/html", SendHTML());
}

void handle_ledon() {
  digitalWrite(LEDPIN, LEDON);   // Turn the LED on by smaking the voltage LOW (inversed for this LED)
  client.publish(LED_st_topic, "ON");
  Serial.println("LED Status: ON");
  setLEDstate();
  httpServer.send(200, "text/html", SendHTML());
}

void handle_ledoff() {
  digitalWrite(LEDPIN, LEDOFF);   // Turn the LED on by smaking the voltage LOW (inversed for this LED)
  client.publish(LED_st_topic, "OFF");
  Serial.println("LED Status: OFF");
  setLEDstate();
  httpServer.send(200, "text/html", SendHTML());
}

void handle_relayon() {
  digitalWrite(RELAYPIN, HIGH);   // Turn the RELAY on
  client.publish(RELAY_st_topic, "ON");
  Serial.println("RELAY Status: ON");
  setLEDstate();
  httpServer.send(200, "text/html", SendHTML());
}

void handle_relayoff() {
  digitalWrite(RELAYPIN, LOW);   // Turn the RELAY on
  client.publish(RELAY_sw_topic, "OFF");
  Serial.println("RELAY Status: OFF");
  setLEDstate();
  httpServer.send(200, "text/html", SendHTML());
}

void handle_NotFound() {
  httpServer.send(404, "text/plain", "Not found");
}

String SendHTML() {
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr += "<title>";
  ptr += mqtt_id;
  ptr += "</title>\n";
  ptr += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr += "body{margin-top: 50px;} h1 {color: #444444;margin: 25px auto 30px;} h3 {color: #444444;margin-bottom: 30px;}\n";
  ptr += ".button {display: block;width: 150px;background-color: #1abc9c;border: none;color: white;padding: 13px 10px;text-decoration: none;font-size: 20px;margin: 0px auto 15px;cursor: pointer;border-radius: 4px;}\n";
  ptr += ".button-on {background-color: #1abc9c;}\n";
  ptr += ".button-on:active {background-color: #16a085;}\n";
  ptr += ".button-off {background-color: #34495e;}\n";
  ptr += ".button-off:active {background-color: #2c3e50;}\n";
  ptr += ".button-update {background-color: #a32267;}\n";
  ptr += ".button-update:active {background-color: #961f5f;}\n";
  ptr += "p {font-size: 18px;color: #383535;margin-bottom: 15px;}\n";
  ptr += "</style>\n";
  ptr += "</head>\n";
  ptr += "<body>\n";
  ptr += "<h1>Smart plug Web Control</h1>\n";
  ptr += "<h3>Control of Relay, LED and link to HTTPWebUpdate</h3>\n";
  ptr += "<p>Sonoff WimIOT\nDevice: ";
  ptr += mqtt_id;
  ptr += "<br>Software version: ";
  ptr += software_version;
  if (BRAND == BSD34 || BRAND == NEOCOOLCAM)
  {
    ptr += "<br><br>Power: ";
    ptr += powerW;
    ptr += " W";
    ptr += "<br>Energy consumption: ";
    ptr += energy_consumption;
    ptr += " kWh";
  }
  ptr += "<br></p>";
  //if (ledstatus)
  if (digitalRead(LEDPIN) == LEDON) // LED is on (reversed logic)
  {
    ptr += "<p>LED Status: ON</p><a class=\"button button-on\" href=\"/ledoff\">Turn LED OFF</a>\n";
  }
  else
  {
    ptr += "<p>LED Status: OFF</p><a class=\"button button-off\" href=\"/ledon\">Turn LED ON</a>\n";
  }

  if (digitalRead(RELAYPIN) == HIGH)
  {
    ptr += "<p>Relay Status: ON</p><a class=\"button button-on\" href=\"/relayoff\">Turn Relay OFF</a>\n";
  }
  else
  {
    ptr += "<p>Relay Status: OFF</p><a class=\"button button-off\" href=\"/relayon\">Turn Relay ON</a>\n";
  }

  ptr += "<p>Click for update page</p><a class=\"button button-update\" href=\"/update\">Update</a>\n";


  ptr += "</body>\n";
  ptr += "</html>\n";
  return ptr;
}

void publish_power(void)
{
  String tmp_str; // String for publishing the data as a string to MQTT
  char buf5[20];

  tmp_str = String(powerW);
  tmp_str.toCharArray(buf5, tmp_str.length() + 1);
  client.publish(powerW_topic, buf5);

  tmp_str = String(energy_consumption);
  tmp_str.toCharArray(buf5, tmp_str.length() + 1);
  client.publish(kWh_topic, buf5);
}

void publish_status(void)
{
  client.publish(status_topic, WiFi.localIP().toString().c_str());
  char buf[80] = "Status OK. IP: ";
  char buf2[20];
  sprintf(buf2, "%s", WiFi.localIP().toString().c_str());
  strcat (buf, buf2);
  sprintf(buf2, ". RSSI: ");
  strcat (buf, buf2);
  sprintf(buf2, "%ld", WiFi.RSSI());
  strcat (buf, buf2);

  sprintf(buf2, ". Relay = ");
  strcat (buf, buf2);
  if (digitalRead(RELAYPIN) == HIGH)
    sprintf(buf2, "ON");
  else
    sprintf(buf2, "OFF");
  strcat (buf, buf2);

  client.publish(status_topic, buf);
}

void setLEDstate(void)
{
  if (DEVICE == DEVICE_TYPE2 || DEVICE == DEVICE_TYPE4)   // set LED to Relay state
  {
    if (digitalRead(LEDPIN) == digitalRead(RELAYPIN))
      digitalWrite(LEDPIN, !digitalRead(RELAYPIN)); // pin is inverted
  }
}
/*
  void CF1_impulse() { // Captures count of pulses from CF1_PIN
  CF1_counts++;
  }
*/
void CF_impulse() { // Captures count of pulses from CF_PIN
  CF_counts++;
}
