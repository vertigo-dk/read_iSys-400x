
//#include <PulsePosition.h>
#include "iSYS_TargetDecoding.h"

// for Network / OSC
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCBundle.h>
#include "TeensyMAC.h"

#include "Config.h"

// DEFINES
#define VERSION_HI 0
#define VERSION_LO 1

#define DEBUG true

#define INPUT_1_1_PIN 19
#define INPUT_1_2_PIN 18
#define INPUT_1_3_PIN 17

#define INPUT_2_1_PIN 16
#define INPUT_2_2_PIN 15
#define INPUT_2_3_PIN 14

#define PIN_RESET 9


// Names to switch state


const int S_SENSOR_READ = 0;
const int S_SENSOR_CONFIG_A = 1;
const int S_SENSOR_CONFIG_B = 2;
int state = S_SENSOR_READ;



/*
  #define VEL_MIN 0
  #define VEL_MAX 20
  #define RANGE_MIN 0
  #define RANGE_MAX 4
*/

/*
  // PWM readout
  PulsePositionInput input_1(RISING);
  int count = 0;
*/


// SERIAL readout


unsigned char serialBuffer[256];

iSYSTargetList_t targetList[2];

unsigned long millisLastRead = 0;

// OSC
EthernetUDP udp;
EthernetServer server(9008);
String ipStr;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CONFIG
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
T_Config config = {
  {0xDE, 0xAD, 0xBE, 0x00, 0x00, 0x00}, // MAC - last 3 bytes set by Teensy
  {2, 0, 0, 1},                         // IP
  {255, 0, 0, 0},                       // Subnet mask
  false,                                // DHCP
  9002,                               // oscLocalPort
  9001,                               // oscTargetPort
  {2, 0, 0, 1},                         // oscTargetIP
  0,                                    // id
  // These fields get overwritten by loadConfig:
  "RadarReadout",                           // Short name
  "iSys-4001_readout",                     // Long name
  VERSION_HI,
  VERSION_LO
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SETUP
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  //saveConfig(&config); //<-- uncomment to force the EEPROM config to default on each reboot


  T_Config defaultConfig = config;
  loadConfig(&config);

  config.verHi = defaultConfig.verHi;
  config.verLo = defaultConfig.verLo;

  mac_addr mac;
  for (int i = 3; i < 6; i++) {
    config.mac[i] = mac.m[i];
  }

  // Calculate IP address
  config.ip[0] = 2;
  config.ip[1] = config.mac[3];
  config.ip[2] = config.mac[4];
  config.ip[3] = config.mac[5];

  ipStr = String(config.ip[0]) + String(".") + String(config.ip[1]) + String(".") + String(config.ip[2]) + String(".") + String(config.ip[3])  ;


  // first time, create id as checksum of the ip
  if (config.id == 0) {
    int check_sum = 0;              //checksum
    for (int i = 0; i < 4; i++) {
      check_sum += (int)config.ip[i];     //calculate the checksum
    }
    if (check_sum > 255) {           //if greater than 8 bits then encode bits
      int lcheck_sum = check_sum;
      lcheck_sum = lcheck_sum >> 8;    //shift 8 bits to the right
      int rcheck_sum = check_sum & 0x00FF;
      check_sum = lcheck_sum + rcheck_sum;
    }
    config.id = check_sum;
  }

  saveConfig(&config);

  //initialise WIZ820io
#ifdef PIN_RESET
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_RESET, HIGH);
  delay(150);
#endif

  //pinMode(INPUT_PIN_1, INPUT);
  //input_1.begin(INPUT_PIN_1);



  // Open Ethernet connection
  IPAddress dns(2, 0, 0, 1);
  IPAddress gateway(2, 0, 0, 1);
  IPAddress subnet(255, 0, 0, 0);

  Ethernet.begin(config.mac, config.ip, dns, gateway, subnet);
  udp.begin(config.oscLocalPort);
  server.begin();

  // Serial
  if (DEBUG) Serial.begin(115200);

  Serial1.begin(115200);
  Serial3.begin(115200);

  Serial1.write(iSYS_startAcquisiton, sizeof(iSYS_startAcquisiton));
  Serial1.write(iSYS_requestMeasurement, sizeof(iSYS_requestMeasurement));

  Serial3.write(iSYS_startAcquisiton, sizeof(iSYS_startAcquisiton));
  Serial3.write(iSYS_requestMeasurement, sizeof(iSYS_requestMeasurement));

  if (DEBUG) Serial.println("Setup finished");


}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// LOOP
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  switch (state)
  {
    case S_SENSOR_READ :
      {
        // READ SENSOR DATA - SERIAL
        unsigned long currentMillis = millis();
        if (currentMillis - millisLastRead > 75) { // wait for 75ms to make sure the sensors had enought time to make the measure.
          millisLastRead = currentMillis;


          // readData from sensor A
          while (Serial1.available()) {
            for (unsigned int i = 0; i < sizeof(serialBuffer); i++) {
              serialBuffer[i] = Serial1.read();
              if (!Serial1.available() || serialBuffer[i] == 0x16) {
                iSYSResultValue_t result = decodeFrame(&serialBuffer[0], i + 1, 4001, 32, &targetList[0]);
                //if (DEBUG) Serial.print("A: "); Serial.println( result );
                Serial1.write(iSYS_requestMeasurement, sizeof(iSYS_requestMeasurement));

                if (targetList[0].nrOfTargets > 0) sendTargetsOSC(0);
                if (result.iSYSResult == ACK_STARTSTOP_ACQUISITION) sendAckAcqOSC(0);
                if (result.iSYSResult == ACK_SENSOR_WRITE) sendAckSensWriteOSC(0);
                if (result.iSYSResult == ACK_SENSOR_WRITE_EEPROM) sendAckWritePromOSC(0);
                if (result.iSYSResult == READ_CHANNEL) sendChannelOSC(0, result.value);
                if (result.iSYSResult == READ_POTI) sendPotiOSC(0, result.value);

                break;
              }
            }
          }

          // readData fron sensor B
          while (Serial3.available()) {
            for (unsigned int i = 0; i < sizeof(serialBuffer); i++) {
              serialBuffer[i] = Serial3.read();
              if (!Serial3.available() || serialBuffer[i] == 0x16) {
                iSYSResultValue_t result = decodeFrame(&serialBuffer[0], i + 1, 4001, 32, &targetList[1]);
                //if (DEBUG) Serial.print("B: "); Serial.println( result );
                Serial3.write(iSYS_requestMeasurement, sizeof(iSYS_requestMeasurement));
                if (targetList[1].nrOfTargets > 0) sendTargetsOSC(1);
                if (result.iSYSResult == ACK_STARTSTOP_ACQUISITION) sendAckAcqOSC(1);
                if (result.iSYSResult == ACK_SENSOR_WRITE) sendAckSensWriteOSC(1);
                if (result.iSYSResult == ACK_SENSOR_WRITE_EEPROM) sendAckWritePromOSC(1);
                if (result.iSYSResult == READ_CHANNEL) sendChannelOSC(1, result.value);
                if (result.iSYSResult == READ_POTI) sendPotiOSC(1, result.value);
                break;
              }
            }
          }
        }
      }
      break;
    // ----- END SENSOR_READ -----

    case S_SENSOR_CONFIG_A :
      {
        EthernetClient client = server.available();
        if (client) {
          while (client.connected()) {
            // transmit
            while (client.available()) {
              char c = client.read();
              Serial1.print(c); // then send the message through serial

              if (DEBUG) {
                Serial.print(c, HEX);
                if (c == 0x16) {
                  Serial.println(" received");
                }
              }
            }

            // receive
            while (Serial1.available() > 0) { // if data has been received from the serial connection
              char s = Serial1.read();
              client.print(s);

              if (DEBUG) {
                Serial.print(s, HEX);
                if (s == 0x16) {
                  Serial.println(" send");
                }
              }
            }
          }
        }
      }
      break;

    case S_SENSOR_CONFIG_B :
      {
        EthernetClient client = server.available();
        if (client) {
          while (client.connected()) {
            // transmit
            while (client.available()) {
              char c = client.read();
              Serial3.print(c); // then send the message through serial

              if (DEBUG) {
                Serial.print(c, HEX);
                if (c == 0x16) {
                  Serial.println(" received");
                }
              }
            }

            // receive
            while (Serial3.available() > 0) { // if data has been received from the serial connection
              char s = Serial3.read();
              client.print(s);

              if (DEBUG) {
                Serial.print(s, HEX);
                if (s == 0x16) {
                  Serial.println(" send");
                }
              }
            }
          }
        }
      }
      break;
      // ----- END SENSOR_CONFIG -----
  }


  OSCMsgReceive();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// OSC functions
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// routed response functions

void OSCMsgReceive() {

  OSCMessage msgIN;
  int size;
  if ((size = udp.parsePacket()) > 0) {
    while (size--) {
      msgIN.fill(udp.read());
    }
    if (!msgIN.hasError()) {
      msgIN.route("/setMode", setMode);
      msgIN.route("/getMode", getMode);
      msgIN.route("/setSensorChannel", setSensorChannel);
      msgIN.route("/getSensorChannel", getSensorChannel);
      msgIN.route("/setSensorPoti", setSensorPoti);
      msgIN.route("/getSensorPoti", getSensorPoti);
      msgIN.route("/setId", setId);
      msgIN.route("/getConfig", getConfig);
      msgIN.route("/saveAllSetting", saveAllSetting);
    }
  }
}

void setMode(OSCMessage & msg, int addrOffset) {
  if (msg.getInt(0) == config.id) {
    switch (msg.getInt(1)) {
      case 0 :
        {
          state = S_SENSOR_READ;
          Serial1.write(iSYS_startAcquisiton, sizeof(iSYS_startAcquisiton));
          Serial1.write(iSYS_requestMeasurement, sizeof(iSYS_requestMeasurement));
          Serial3.write(iSYS_startAcquisiton, sizeof(iSYS_startAcquisiton));
          Serial3.write(iSYS_requestMeasurement, sizeof(iSYS_requestMeasurement));
          millisLastRead = millis();

          if (DEBUG) Serial.println("MODE: SENSOR READ");
        }
        break;

      case 1 :
        {
          state = S_SENSOR_CONFIG_A;
          if (DEBUG) Serial.println("MODE: SENSOR CONFIG A");
        }
        break;

      case 2 :
        {
          state = S_SENSOR_CONFIG_B;
          if (DEBUG) Serial.println("MODE: SENSOR CONFIG B");
        }
        break;
    }
  }
};

void getMode(OSCMessage & msg, int addrOffset) {
  OSCMessage newMsg("/mode");
  newMsg.add((int) config.id);
  newMsg.add((int) state);
  udp.beginPacket(config.oscTargetIp, config.oscTargetPort);
  newMsg.send(udp);
  udp.endPacket();
  newMsg.empty();
};

void setSensorChannel(OSCMessage & msg, int addrOffset) {
  if (msg.getInt(0) == config.id) {
    uint8_t iSys_setChannelNew[13];
    memcpy(iSys_setChannelNew, iSYS_setChannelX, sizeof(iSys_setChannelNew));
    iSys_setChannelNew[10] += (uint8_t)msg.getInt(2);
    iSys_setChannelNew[11] += (uint8_t)msg.getInt(2);

    switch (msg.getInt(1)) {
      case 0 :
        {
          Serial1.write(iSYS_stopAcquisiton, sizeof(iSYS_stopAcquisiton));
          delay(10);
          Serial1.write(iSys_setChannelNew, sizeof(iSys_setChannelNew));
          delay(10);
          Serial1.write(iSYS_startAcquisiton, sizeof(iSYS_startAcquisiton));
        }
        break;
      case 1 :
        {
          Serial3.write(iSYS_stopAcquisiton, sizeof(iSYS_stopAcquisiton));
          delay(10);
          Serial3.write(iSys_setChannelNew, sizeof(iSys_setChannelNew));
          delay(10);
          Serial3.write(iSYS_startAcquisiton, sizeof(iSYS_startAcquisiton));
        }
        break;
      case 2 :
        {
          Serial1.write(iSYS_stopAcquisiton, sizeof(iSYS_stopAcquisiton));
          delay(10);
          Serial1.write(iSys_setChannelNew, sizeof(iSys_setChannelNew));
          delay(10);
          Serial1.write(iSYS_startAcquisiton, sizeof(iSYS_startAcquisiton));

          Serial3.write(iSYS_stopAcquisiton, sizeof(iSYS_stopAcquisiton));
          delay(10);
          Serial3.write(iSys_setChannelNew, sizeof(iSys_setChannelNew));
          delay(10);
          Serial3.write(iSYS_startAcquisiton, sizeof(iSYS_startAcquisiton));
        }
        break;
    }
  }
};

void getSensorChannel(OSCMessage & msg, int addrOffset) {
  if (msg.getInt(0) == config.id) {
    switch (msg.getInt(1)) {
      case 0 :
        {
          Serial1.write(iSYS_getChannel, sizeof(iSYS_getChannel));
        }
        break;

      case 1 :
        {
          Serial3.write(iSYS_getChannel, sizeof(iSYS_getChannel));
        }
        break;

      case 2 :
        {
          Serial1.write(iSYS_getChannel, sizeof(iSYS_getChannel));
          Serial3.write(iSYS_getChannel, sizeof(iSYS_getChannel));
        }
        break;
    }
    if (DEBUG) Serial.print("read channel from sensor: "); Serial.println(msg.getInt(1));
  }
};

void setSensorPoti(OSCMessage & msg, int addrOffset) {
  if (msg.getInt(0) == config.id) {
    uint8_t iSys_setPotiNew[13];
    memcpy(iSys_setPotiNew, iSYS_setPotiX, sizeof(iSys_setPotiNew));
    iSys_setPotiNew[10] += (uint8_t)msg.getInt(2);
    iSys_setPotiNew[11] += (uint8_t)msg.getInt(2);

    switch (msg.getInt(1)) {
      case 0 :
        {
          Serial1.write(iSys_setPotiNew, sizeof(iSys_setPotiNew));
        }
        break;
      case 1 :
        {
          Serial3.write(iSys_setPotiNew, sizeof(iSys_setPotiNew));
        }
        break;
      case 2 :
        {
          Serial1.write(iSys_setPotiNew, sizeof(iSys_setPotiNew));
          Serial3.write(iSys_setPotiNew, sizeof(iSys_setPotiNew));
        }
        break;
    }
  }
};

void getSensorPoti(OSCMessage & msg, int addrOffset) {
  if (msg.getInt(0) == config.id) {
    switch (msg.getInt(1)) {
      case 0 :
        {
          Serial1.write(iSYS_getPoti, sizeof(iSYS_getPoti));
        }
        break;

      case 1 :
        {
          Serial3.write(iSYS_getPoti, sizeof(iSYS_getPoti));
        }
        break;

      case 2 :
        {
          Serial1.write(iSYS_getPoti, sizeof(iSYS_getPoti));
          Serial3.write(iSYS_getPoti, sizeof(iSYS_getPoti));
        }
        break;
    }
    if (DEBUG) Serial.print("read poti from sensor: "); Serial.println(msg.getInt(1));
  }
};

void setId(OSCMessage & msg, int addrOffset) {
  // compare if received IP string is the same as the local
  char ipStrReceived[15];
  msg.getString(0, ipStrReceived, 15);
  bool isSame = true;
  for (unsigned int i = 0; i < ipStr.length(); i++) {
    if (ipStrReceived[i] != ipStr[i]) isSame = false;
  }

  if (isSame) {
    if (msg.isInt(1)) {
      config.id = msg.getInt(1);
      saveConfig(&config);

      if (DEBUG) {
        Serial.print("new ID: ");
        Serial.println(config.id);
      }
    }
  }
};

void getConfig(OSCMessage & msg, int addrOffset) {
  OSCMessage newMsg("/config");
  newMsg.add((int) config.id);
  newMsg.add( ipStr.c_str());
  udp.beginPacket(config.oscTargetIp, config.oscTargetPort);
  newMsg.send(udp);
  udp.endPacket();
  newMsg.empty();

  if (DEBUG) {
    Serial.println("CONFIG INFO");
    Serial.print("IP:");
    Serial.println(ipStr);
  }
};

void saveAllSetting(OSCMessage & msg, int addrOffset) {
  Serial1.write(iSYS_saveAllToEEPROM, sizeof(iSYS_saveAllToEEPROM));
  Serial3.write(iSYS_saveAllToEEPROM, sizeof(iSYS_saveAllToEEPROM));

  if (DEBUG) {
    Serial.println("Save all setting");
  }
};

void sendTargetsOSC(int indx) {
  // sending OSC
  OSCMessage msg("/sensorRead");
  msg.add((int) config.id);
  if (indx == 0)   msg.add("A");
  if (indx == 1)   msg.add("B");

  msg.add(targetList[indx].nrOfTargets);

  for (int i = 0; i < targetList[indx].nrOfTargets; i++) {
    msg.add((float)targetList[indx].targets[i].velocity);
    Serial.println(targetList[indx].targets[i].velocity);
    msg.add((float)targetList[indx].targets[i].range);
    msg.add((float)targetList[indx].targets[i].signal);
  }
  udp.beginPacket(config.oscTargetIp, config.oscTargetPort);
  msg.send(udp); // send the bytes to the SLIP stream
  udp.endPacket(); // mark the end of the OSC Packet
  msg.empty();
};

void sendAckAcqOSC(int indx) {
  // sending OSC
  OSCMessage msg("/sensorResponse/acknowledge");
  msg.add((int) config.id);
  if (indx == 0)   msg.add("A");
  if (indx == 1)   msg.add("B");

  msg.add("acquisition changed");
  udp.beginPacket(config.oscTargetIp, config.oscTargetPort);
  msg.send(udp); // send the bytes to the SLIP stream
  udp.endPacket(); // mark the end of the OSC Packet
  msg.empty();
};

void sendAckSensWriteOSC(int indx) {
  // sending OSC
  OSCMessage msg("/sensorResponse/acknowledge");
  msg.add((int) config.id);
  if (indx == 0)   msg.add("A");
  if (indx == 1)   msg.add("B");

  msg.add("sensor wrote change");
  udp.beginPacket(config.oscTargetIp, config.oscTargetPort);
  msg.send(udp); // send the bytes to the SLIP stream
  udp.endPacket(); // mark the end of the OSC Packet
  msg.empty();
};

void sendAckWritePromOSC(int indx) {
  // sending OSC
  OSCMessage msg("/sensorResponse/acknowledge");
  msg.add((int) config.id);
  if (indx == 0)   msg.add("A");
  if (indx == 1)   msg.add("B");

  msg.add("sensor wrote to EEPROM");
  udp.beginPacket(config.oscTargetIp, config.oscTargetPort);
  msg.send(udp); // send the bytes to the SLIP stream
  udp.endPacket(); // mark the end of the OSC Packet
  msg.empty();
};

void sendChannelOSC(int indx, uint8_t value) {
  // sending OSC
  OSCMessage msg("/sensorResponse/channel");
  msg.add((int) config.id);
  if (indx == 0)   msg.add("A");
  if (indx == 1)   msg.add("B");

  msg.add((int)value);
  udp.beginPacket(config.oscTargetIp, config.oscTargetPort);
  msg.send(udp); // send the bytes to the SLIP stream
  udp.endPacket(); // mark the end of the OSC Packet
  msg.empty();
};

void sendPotiOSC(int indx, uint8_t value) {
  // sending OSC
  OSCMessage msg("/sensorResponse/poti");
  msg.add((int) config.id);
  if (indx == 0)   msg.add("A");
  if (indx == 1)   msg.add("B");

  msg.add((int)value);
  udp.beginPacket(config.oscTargetIp, config.oscTargetPort);
  msg.send(udp); // send the bytes to the SLIP stream
  udp.endPacket(); // mark the end of the OSC Packet
  msg.empty();
};


