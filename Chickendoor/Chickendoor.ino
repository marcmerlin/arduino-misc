/* ------------------------------------------------- */

// TODO: https://github.com/JoaoLopesF/RemoteDebug?tab=readme-ov-file

#include "ESPTelnet.h"          
//#include <Servo.h>
#include <ESP32Servo.h>

void(* resetFunc) (void) = 0; // jump to 0 to cause a sofware reboot

/* ------------------------------------------------- */

#define SERIAL_SPEED    115200

#define SERVO_PIN	15
#define OPEN_PIN	23
// was 22, but seems to be floating or undefined even with pull up
#define CLOSE_PIN	21
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

#include "wifi_secrets.h"
#include <ArduinoOTA.h>

template <typename T>
class GenericSerialSplitter : public Stream {
private:
  T* hw;
  WiFiServer* server;
  WiFiClient client;
  bool serverStarted = false; 
  NetworkUDP udpServer;
  bool udpStarted = false;
  uint16_t udpPort;
  
  // Cooldown timer to prevent UDP polling from starving the network stack
  unsigned long lastUdpCheck = 0;
  const unsigned long UDP_COOLDOWN_MS = 100; // 10Hz limit

public:
  GenericSerialSplitter(T& hardwareSerial, uint16_t port = 23) {
    hw = &hardwareSerial;
    server = new WiFiServer(port);
    udpPort = port;
  }

  // Pass the initialization through to the underlying serial interface
  void begin(unsigned long baud) {
    hw->begin(baud);
  }

    // Boolean operator so "if (Serial)" and "while (!Serial)" evaluate cleanly
  operator bool() const {
    return hw ? (bool)(*hw) : false;
  }

  void restart() {
    Serial.println("Inside SerialSplitter.restart()");
    if (serverStarted) {
        server->end();
        serverStarted = false;
        Serial.println("SerialSplitter.restart(): end Splitter");
    }
    if (client) {
        client.stop();
        Serial.println("SerialSplitter.restart(): stop client");
    }
    if (udpStarted) {
        udpServer.stop(); 
        udpStarted = false;
    }
  }

  void handle() {
    if (WiFi.status() != WL_CONNECTED && WiFi.getMode() != WIFI_AP && WiFi.getMode() != WIFI_AP_STA) {
        return; 
    }

    if (!serverStarted) {
        Serial.println("SerialSplitter: (re)start");
        server->begin();
        serverStarted = true;
    }

    // Enabling this in the past caused it to eat UDP broadcast from the master
    // hopefully UDP_COOLDOWN_MS will fix this and allow enabling safely
    if (!udpStarted) {
        Serial.println("SerialSplitter: (re)start UDP");
        if (udpServer.begin(udpPort)) {
            udpStarted = true;
        }
    }

    if (server->hasClient()) {
      if (!client || !client.connected()) {
        if (client) client.stop();
        // Fixed Core 3.x deprecation: converted available() to accept()
        client = server->accept(); 
        client.println("\n--- Connected to ESP32 Serial Splitter ---");
      } else {
        // Fixed Core 3.x deprecation: converted available() to accept()
        WiFiClient reject = server->accept();
        reject.stop(); 
      }
    }
  }

  // --- STREAM/PRINT OUTPUT OVERRIDES ---
  
  size_t write(uint8_t c) override {
    size_t n = hw->write(c);         
    if (client && client.connected()) {
      client.write(c);               
    }
    return n;
  }

  size_t write(const uint8_t *buffer, size_t size) override {
    size_t n = hw->write(buffer, size);
    if (client && client.connected()) {
      client.write(buffer, size);
    }
    return n;
  }

  // --- STREAM INPUT OVERRIDES (TWO-WAY COMMS) ---

  int available() override { 
    int total = hw->available();
    if (client) {
      total += client.available();
    }
    if (udpStarted) {
        // RATE LIMITING: Only check for new UDP packets every 100ms
        if (millis() - lastUdpCheck > UDP_COOLDOWN_MS) {
            lastUdpCheck = millis();
            if (udpServer.available() == 0) {
                udpServer.parsePacket();
            }
        }
        total += udpServer.available();
    }
    return total;
  }

  int read() override { 
    if (client && client.available()) {
      return client.read();
    }
    if (udpStarted && udpServer.available()) {
      return udpServer.read();
    }
    return hw->read(); 
  }

  int peek() override { 
    if (client && client.available()) {
      return client.peek();
    }
    if (udpStarted && udpServer.available()) {
      return udpServer.peek();
    }
    return hw->peek(); 
  }

  void flush() override { 
    hw->flush(); 
    // Fixed Core 3.x deprecation: converted flush() to clear()
    if(client) client.clear(); 
  }
};

// 2. Instantiate our splitter using decltype(Serial) to automatically match the architecture
GenericSerialSplitter<decltype(Serial)> Splitter(Serial, 24);

// 3. THE HIJACK: Replace the word "Serial" with our Splitter for the rest of this file
#undef Serial
#define Serial Splitter


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
  telnet.println("\nWelcome " + telnet.getIP() + ". Build Time: " + __DATE__ + " " +  __TIME__);
  telnet.println("Commands: [angle:0-255]/open/close/water/ping/bye/reboot");
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

  setupSerial(SERIAL_SPEED, "Serial Init ");

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

  Serial.println("Setup ArduinoOTA. Remote Wifi Updates Supported!");
  ArduinoOTA.setHostname("Marc Chickendoor");
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  
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
  Serial.println("Done with init. Build time: " __DATE__ " " __TIME__);
}

/* ------------------------------------------------- */

void loop() {
    static uint32_t nextprint = 0;

    ArduinoOTA.handle();
    Splitter.handle();

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

    // bug: for some reason the first read can return yes and the 2nd one no.
    uint16_t op  = !digitalRead(OPEN_PIN);
    uint16_t op2 = !digitalRead(OPEN_PIN);
    if (op2 && posswitch != 1) {
      prime_water_level_read();
      Serial.printf("Open invert: %d / %d (pos %d)\n\r", op, op2, posswitch);
      Serial.println("Switch to Open and re-poll water level");
      posswitch = 1;
      newpos = openpos;
      setservo();
    }
    // bug: for some reason the first read can return yes and the 2nd one no.
    uint16_t cp  = !digitalRead(CLOSE_PIN);
    uint16_t cp2 = !digitalRead(CLOSE_PIN);
    if (cp2 && posswitch != -1) {
      prime_water_level_read();
      Serial.printf("Close invert: %d / %d (pos %d)\n\r", cp, cp2, posswitch);
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
        Serial.printf("Pos SW: %d, Open: %d, Close: %d, Water: %d\n\r", posswitch, !digitalRead(OPEN_PIN), !digitalRead(CLOSE_PIN), water_read);
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
