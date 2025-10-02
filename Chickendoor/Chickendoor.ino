/* ------------------------------------------------- */

// TODO: https://github.com/JoaoLopesF/RemoteDebug?tab=readme-ov-file

#include "ESPTelnet.h"          
//#include <Servo.h>
#include <ESP32Servo.h>

void(* resetFunc) (void) = 0; // jump to 0 to cause a sofware reboot

/* ------------------------------------------------- */

#define SERIAL_SPEED    115200
#include "wifi_secrets.h"

#define SERVO_PIN	15
#define OPEN_PIN	23
#define CLOSE_PIN	22
#define WATER_PIN	18
// Only power the water testing board once an hour
#define WATER_PWR_PIN	19

/* ------------------------------------------------- */

// 1 => tell server we got open
// -1 => tell server we got close
int8_t posswitch; 
uint32_t water_board_on;


int16_t openpos=20;
int16_t closepos=150;

// Assume water is good by default until it is read
bool water_read = 1;

ESPTelnet telnet;
IPAddress ip;
uint16_t  port = 23;

Servo myservo;  // create servo object to control a servo
int16_t pos=90; // slightly opened
int16_t newpos=pos;

/* ------------------------------------------------- */

void prime_water_level_read() {
    digitalWrite(WATER_PWR_PIN, HIGH);
    water_board_on = millis();
    Serial.print("Powering Water Board On for subsequent read at millis: ");
    Serial.println(water_board_on);
}

/* ------------------------------------------------- */

void setservo()
{
    if (newpos == pos) {
	Serial.print("Ignoring position move to ");
	Serial.println(pos);
	return;
    }   
    Serial.printf("Changing servo to %d\n\r", newpos);
    while (newpos != pos) {
      if (newpos > pos) { pos++; }
      if (newpos < pos) { pos--; }
      delay(40);
      myservo.write(pos);
    }
}


/* ------------------------------------------------- */

void setupSerial(long speed, String msg = "") {
  Serial.begin(speed);
  while (!Serial) {
  }
  delay(200);  
  Serial.println();
  Serial.println();
  if (msg != "") Serial.println(msg);
}

/* ------------------------------------------------- */

bool isConnected() {
  return (WiFi.status() == WL_CONNECTED);
}

/* ------------------------------------------------- */

bool connectToWiFi(const char* ssid, const char* password, int max_tries = 20, int pause = 500) {
  int i = 0;
  WiFi.mode(WIFI_STA);
  #if defined(ARDUINO_ARCH_ESP8266)
    WiFi.forceSleepWake();
    delay(200);
  #endif
  WiFi.begin(ssid, password);
  do {
    delay(pause);
    Serial.print(".");
  } while (!isConnected() || i++ < max_tries);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  // https://github.com/espressif/arduino-esp32/issues/1921
  WiFi.setSleep(true);
  return isConnected();
}

/* ------------------------------------------------- */

void errorMsg(String error, bool restart = true) {
  Serial.println(error);
  if (restart) {
    Serial.println("Rebooting now...");
    delay(2000);
    ESP.restart();
    delay(2000);
  }
}

/* ------------------------------------------------- */

void setupTelnet() {  
    // passing on functions for various telnet events
    telnet.onConnect(onTelnetConnect);
    telnet.onConnectionAttempt(onTelnetConnectionAttempt);
    telnet.onReconnect(onTelnetReconnect);
    telnet.onDisconnect(onTelnetDisconnect);

    // passing a lambda function
    telnet.onInputReceived([](String str) {
      // checks for a certain command
      if ((newpos = str.toInt())) {
          setservo();
          telnet.print("Reset posswitch (and NEW: flag) and move servo angle to ");
          telnet.println(String(newpos));
          posswitch = 0; 
      } else if (str == "open") {
          posswitch = 2;
          telnet.println("> Simulate local switch open");
          Serial.println("> Simulate local switch open");
          newpos = openpos;
          setservo();
      } else if (str == "close") {
          posswitch = -2;
          telnet.println("> Simulate local switch close");
          Serial.println("> Simulate local switch close");
          newpos = closepos;
          setservo();
      } else if (str == "water") {
          telnet.println("> Trigger water level read, wait 15 sec to get result");
          Serial.println("> Trigger water level read, wait 15 sec to get result");
          prime_water_level_read();
      } else if (str == "ping") {
          telnet.println("> pong");
          Serial.println("- Telnet: pong");
      } else if (str == "reboot") {
          telnet.println("> reboot");
          Serial.println("- reboot");
          resetFunc();
      } else if (str == "bye") {
          char posstr[5];
          telnet.print("Pos SW: ");
          sprintf(posstr, "%d", posswitch);
          telnet.print(posstr);
          telnet.print(", Open SW: ");
          telnet.print((char) (!digitalRead(OPEN_PIN)+48));
          telnet.print(" Close SW: ");
          telnet.print((char) (!digitalRead(CLOSE_PIN)+48));
          telnet.print(" Water: ");
          telnet.println((char) (water_read+48));
          telnet.println("> disconnecting you... Current servo angle is");
          Serial.print("Disconnecting and sending servo angle ");
          // We send new to the other other side until they reset it by sending a new
          // angle to turn that off (adding +1)
          if (posswitch) { telnet.print("NEW: "); Serial.print("NEW: "); };
          telnet.println(String(pos));
          Serial.println(pos);
          telnet.disconnectClient();
      }
    });

    Serial.print("- Telnet: ");
    if (telnet.begin(port)) {
      Serial.println("running");
    } else {
      Serial.println("error.");
      errorMsg("Will reboot...");
    }
}

/* ------------------------------------------------- */

// (optional) callback functions for telnet events
void onTelnetConnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" connected");
  telnet.println("\nWelcome " + telnet.getIP());
  telnet.println("Commands: [angle:0-255]/open/close/water/ping/bye");
}

void onTelnetDisconnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" disconnected");
}

void onTelnetReconnect(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" reconnected");
}

void onTelnetConnectionAttempt(String ip) {
  Serial.print("- Telnet: ");
  Serial.print(ip);
  Serial.println(" tried to connect");
}

/* ------------------------------------------------- */

void setup() {
  pinMode(OPEN_PIN, INPUT_PULLUP);
  pinMode(CLOSE_PIN, INPUT_PULLUP);
  pinMode(WATER_PIN, INPUT_PULLUP);
  pinMode(WATER_PWR_PIN, OUTPUT);

  setupSerial(SERIAL_SPEED, "Serial Init " __DATE__ __TIME__);

  Serial.println("Priming water board for first read");
  prime_water_level_read();

  Serial.println("Servo Init, opening to 90");
  myservo.attach(SERVO_PIN);  // attaches the servo on GIO2 to the servo object
  myservo.write(pos);
#if 0

  int pos;

  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
#endif

  Serial.println("Start");
  Serial.print("Wifi: ");
  connectToWiFi(WIFI_SSID, WIFI_PASSWORD);
  
    if (isConnected()) {
      Serial.println("Wifi connected, opening 1");
      newpos = 80;
      setservo();
      ip = WiFi.localIP();
      Serial.println();
      Serial.print("- Telnet: "); Serial.print(ip); Serial.print(" "); Serial.print(port);
      setupTelnet();
      Serial.println("telnet setup, opening 2");
      newpos = 85;
      setservo();
      Serial.println("setservo2, opening 3");
      newpos = 70;
      setservo();
    } else {
      Serial.println();    
      errorMsg("Error connecting to WiFi");
    }
    Serial.println("setservo3, opening 4");
    newpos = 75;
    setservo();
    Serial.print("Setup done, opening to ");
    newpos = openpos;
    Serial.println(newpos);
    setservo();
}

/* ------------------------------------------------- */

void loop() {
    static uint32_t nextprint = 0;

    if (millis() > nextprint) {
      // Every 5 seconds
      nextprint = millis() + 5000;

      uint32_t diff = millis() - water_board_on;
      // Wait at least 5 sec before trying to read from board after power on
      if (millis() > water_board_on  && diff > 4500) {
        Serial.printf("mil:%u, last: %u, diff: %u. Reading and Powering Water Board Back Off. Water: ", millis(), water_board_on, diff);
        water_read = !digitalRead(WATER_PIN);
        Serial.println(water_read);
        digitalWrite(WATER_PWR_PIN, LOW);
        // do not read from water board again until it's re-enabled
        water_board_on = 4294967295;
      } else if (millis() % 3600000 <  5005)  {
          // prime for sample taken above (within 15 sec)
          prime_water_level_read();
      }
      Serial.printf("Pos SW: %d, Open: %d, Close: %d, Water: %d\n\r", posswitch, !digitalRead(OPEN_PIN), !digitalRead(CLOSE_PIN), water_read);
    }

    if (!digitalRead(OPEN_PIN) && posswitch != 1) {
      prime_water_level_read();
      Serial.println("Switch to Open and re-poll water level");
      posswitch = 1;
      newpos = openpos;
      setservo();
    }
    if (!digitalRead(CLOSE_PIN) && posswitch != -1) {
      prime_water_level_read();
      Serial.println("Switch to Close and re-poll water level");
      posswitch = -1;
      newpos = closepos;
      setservo();
    }

    // Wifi still on. Before Telnet...
    // [D][WiFiClient.cpp:509] connected(): Disconnected: RES: 0, ERR: 128
    if (millis() % 1000 == 0) {
      if (isConnected()) { 
        Serial.printf("Wifi still on. Before Telnet...\n\r");
        telnet.loop();
        Serial.printf("After Telnet...\n\r");
      } else {
        Serial.printf("Wifi connection lost, reconnecting...\n\r");
        connectToWiFi(WIFI_SSID, WIFI_PASSWORD);
        Serial.printf("Wifi reconnected\n\r");
      } 
      delay(1);
    }
}
//* ------------------------------------------------- */
