#include <IRremote.hpp>

#include <WiFi.h>
#include <WiFiClient.h>

WiFiClient client;

#define RECEIVER_PIN 26    // Pin connected to the IR receiver
#define TRANSMITTER_PIN 2  // Pin connected to the IR transmitter
#define SHOOT_PIN 9        // Pin connected to the shoot button
#define RELOAD_PIN 27      // Pin connected to the reload button

const char* ssid = "Xandra-01";
const char* password = "Auenag-01";
const char* host = "192.168.68.110";  // IP address of your Java server
const int gunID = 0;

const bool debug = true;

int DELAY_BETWEEN_REPEAT = 50;
int maxAmmo = 12;
int ammo = 12;

void setup() {
  Serial.begin(115200);
  if (debug)
    Serial.println("Connecting to WiFi");
  WiFi.begin(ssid, password);

  IrSender.begin(TRANSMITTER_PIN);
  IrReceiver.begin(RECEIVER_PIN);
  IrReceiver.start();
  pinMode(SHOOT_PIN, INPUT_PULLUP);
  pinMode(RELOAD_PIN, INPUT_PULLUP);
}

bool reloadStateLast, shootStateLast;

void readInputs() {
  bool shootState = (digitalRead(SHOOT_PIN) == LOW);    // Button pin is active LOW
  bool reloadState = (digitalRead(RELOAD_PIN) == LOW);  // Button pin is active LOW
  if (reloadState != reloadStateLast) {                 //check is button state changed
    reloadStateLast = reloadState;
    if (ammo < maxAmmo) {
      reload();
    }
  }
  if (shootState && !shootStateLast) {
    Serial.println(F("Stop receiving"));
    IrReceiver.stop();
  } else if (!shootState && shootStateLast) {
    // Restart receiver
    Serial.println(F("Button released -> start receiving"));
    IrReceiver.start();
  } else if (IrReceiver.decode()) {
    /*
         * Button is not pressed and data available -> send received data and resume
         */
    int protocol = IrReceiver.decodedIRData.protocol;
    int address = IrReceiver.decodedIRData.address;
    int command = IrReceiver.decodedIRData.command;
    if (debug) {
      IrReceiver.printIRResultShort(&Serial);
    }
    if (protocol == 8 && command == 0x01) {  //check for correct protcol and correct command
      hit(address);
    }
    delay(10);
    IrReceiver.resume();  // resume receiver
  }
  if (shootState != shootStateLast) {  //check is button state changed
    shootStateLast = shootState;
    if (shootState) {
      shoot();
    }
  }
}

void reload() {
  if (ammo != -1)
    ammo = maxAmmo;
}

void shoot() {
  if (ammo > 0) {
    ammo--;
    IrSender.sendNEC(gunID, 0x01, 0);
  }
  Serial.println(ammo);
}

void hit(int address) {
  if (client.connected()) {
    client.println("hitby:" + String(address));
  }
}

void loop() {
  if (!client.connect(host, 8888)) {
    if (debug)
      Serial.println("Connection failed.");
    delay(5000);
    return;
  } else {
    client.println("ID:" + String(gunID));  // send id to GUI server
  }
  while (client.connected()) {  // if we are connected run all logic
    readInputs();
    if (client.available()) {
      String line = client.readStringUntil('\n');
      if (line != "") {
        if (debug)
          Serial.println("Response from server: " + line);
        // String txt = line;
        // String command[16];
        // if (txt.indexOf(':') != -1) {
        //   for (int i = 0; !txt.isEmpty(); i++) {
        //     command[i] = txt.substring(0, txt.indexOf(':'));
        //     txt.remove(0, txt.indexOf(':') + 1);
        //     if (debug) {
        //       Serial.println(String(i) + ":" + command[i]);
        //     }
        //   }
        // }
      }
    }
    delay(10);
  }
}
