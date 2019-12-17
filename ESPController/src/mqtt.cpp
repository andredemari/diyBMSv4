#include <Arduino.h>
#include "mqtt.h"
#include "ArduinoJson.h"
#include "defines.h"
#include <AsyncMqttClient.h>
#include "PacketRequestGenerator.h"
#include "settings.h"



void mqttProc::begin() {    //Initialise relays
  for(uint8_t i=0; i< RELAY_TOTAL; i++) mqttRelay[i] = LOW;
  mqttRelayControl = false;
  active = false;
  pinMode(ESP32_RELAY1, OUTPUT);
  pinMode(ESP32_RELAY2, OUTPUT);
  pinMode(ESP32_RELAY3, OUTPUT);
  ledcSetup(0, 2048, 8);        //Setup PWM on inverter control pin
  ledcAttachPin(INVERTER_PWM, 0);
  bst900_init();
  return;
}


void mqttProc::sendModuleStatus(uint8_t bank, uint8_t module) {
  //char value[127];
  uint8_t address = (bank<<4)||module;
  sprintf(value, "{\"address\":%d,\"volts\":%d,\"temp\":%d,\"exttemp\":%d,\"bypass\":%d}", address, cmi[bank][module].voltagemV,
    cmi[bank][module].internalTemp,cmi[bank][module].externalTemp,cmi[bank][module].inBypass);
  mqttClient.publish(MQTTSUBJECT, 0, true, value);
}

void mqttProc::lostcomms() {          //Timer has expired without a valid MQTT command
  mqttRelayControl=false;
  active=false;
  return;
}

void mqttProc::processCommand(char* payload) {    //MQTT packet received
  Serial.print("MQTT Command received : ");
  Serial.println(payload);
  StaticJsonDocument<256> mqtt_json;
  // Deserialize the JSON document
  DeserializationError error = deserializeJson(mqtt_json, payload);
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  uint8_t command = mqtt_json["command"];
  uint8_t address = mqtt_json["address"];
  uint8_t bank = address>>4;
  uint8_t module = address && 0x0F;

  if(command) {
    mqttRelayControl=true;    //MQTT is controlling
    //reset the timeout timer
    active=true;              //Signal that valid mqtt command received
  }
  //Decode MQTT Command
  switch(command) {
    case Mqtt_restart:                                     // Command 1 - resetESP32
      Serial.println("Restarting Controller");
      delay(1000);
      ESP.restart();
      break;

    case  Mqtt_cell_config:                                     // Command 40 - Set Cell Module Parameters
      Calibration = mqtt_json["calib"];
      Internal_BCoefficient = mqtt_json["intB"];
      External_BCoefficient = mqtt_json["extB"];
      mVPerADC = mqtt_json["mvADC"];
      LoadResistance = mqtt_json["load"];
      BypassOverTempShutdown = mqtt_json["bpT"];
      BypassThresholdmV = mqtt_json["bpmV"];
      prg.sendSaveSetting(bank, module,BypassThresholdmV,BypassOverTempShutdown,LoadResistance,Calibration,mVPerADC,Internal_BCoefficient,External_BCoefficient);
      prg.sendGetSettingsRequest(bank, module);  //Now immediately read settings back again
      break;

    case Mqtt_GlobalCellSettings:
      BypassThresholdmV = mqtt_json["bpmV"];
      BypassOverTempShutdown = mqtt_json["bpT"];
      prg.sendSaveGlobalSetting(BypassThresholdmV, BypassOverTempShutdown);
    break;

    case Mqtt_GlobalSettings:

    break;

    case Mqtt_ReportConfiguration:                                 //Command 8 - Report configuration
    //Loop through cells
    for (int8_t b = 0; b < mysettings.totalNumberOfBanks; b++)
    {
      for (int8_t i = 0; i < numberOfModules[b]; i++) {
        //address=((b<<4)||i);
        if (cmi[b][i].settingsCached == false) {  //Get settings if not already cached
          prg.sendGetSettingsRequest(b, i);           //Must try again later
          // Send failure message to node red
          sprintf(value, "{\"address\":%d,\"command\":%d,\"text\":\"failed\"}", ((b<<4)||i), command);
          mqttClient.publish(MQTTSUBJECT, 0, true, value);
        } else {
          //send mqtt packet with config
          sprintf(value, "{\"address\":%d,\"calib\":%5.4f,\"intB\":%d,\"extB\":%d,\"mvADC\":%3.2f,\"load\":%3.2f,\"bpT\":%d,\"bpmV\":%d}",
            ((b<<4)||i), cmi[b][i].Calibration, cmi[b][i].Internal_BCoefficient,cmi[b][i].External_BCoefficient,cmi[b][i].mVPerADC,
            cmi[b][i].LoadResistance,cmi[b][i].BypassOverTempShutdown,cmi[b][i].BypassThresholdmV);
          mqttClient.publish(MQTTSUBJECT, 0, true, value);
        }
      }
    }
    break;

    case Mqtt_EnableCharger:                                        //Command 10 - Start Charging
      mqttRelay[0]=HIGH;    //Enable charging
      mqttRelay[1]=LOW;     //Disable inverter
      //Relays will switch at next processrules timer expiry
      //TODO set up watchdog timer to auto switch off if loss of mqtt communication
      bstcurrent = mqtt_json["bstcurrent"];
      if (bstcurrent > MAXCHARGERATE) bstcurrent = MAXCHARGERATE;
      setcurrent_bst900();                                             //Set charger current
      Serial.print("Charging rate = "); Serial.println(bstcurrent);
      break;

      case Mqtt_DisableCharger:                                        //Command 11 - Stop Charging
      mqttRelay[0]=LOW;     //Disable charging
      mqttRelay[1]=LOW;     //Disable inverter
      //TODO Tell BST900
      Serial.println("Stop Charging");
      break;
    case Mqtt_EnableInverter:                                        //Command 12 - Start onWifiDisconnectharging
      mqttRelay[0]=LOW;    //Disable charging
      mqttRelay[1]=HIGH;     //Enable inverter
      //TODO set up watchdog timer to auto switch off if loss of mqtt communication
      dischargepower = mqtt_json["dischargepower"];
      dischargepower = (dischargepower < MAXDISCHARGE) ? dischargepower : MAXDISCHARGE;    //Do not permit power output > MAXDISCHARGE
      ledcWrite(0, dischargepower);           //Set discharge rate on PWM pin
      Serial.print("Discharging rate = "); Serial.println(dischargepower);
      break;

      case Mqtt_DisableInverter:                                        //Command 13 - Stop Discharging
      mqttRelay[0]=LOW;    //Disable charging
      mqttRelay[1]=LOW;     //Disable inverter
      ledcWrite(0, 0);
      Serial.println("Stop Discharging");
      break;


    default:
      break;
  }
}


//Functions to manage BST900 boost converter connected to Serial1
// Empty input buffer

void mqttProc::clear()  {       //clear bst buffer
  for(uint8_t i=0 ; i<64; i++) bstbuf[i]=0;
  indx=0;
  return;
}

void mqttProc::serialFlush(){
  while(Serial1.available() > 0) {
    Serial1.read();
  }
  inputready = false;
  clear();
  return;
}

//Process data arriving on serial interface
void mqttProc::bst_process() {
  while(Serial1.available()) {
    char character = Serial1.read();
    if(character == '\r') continue;
    if(character == '\n') {
      if ( (bstbuf[indx-2]=='O' &&  bstbuf[indx-1]=='K' )|| (bstbuf[indx-2]=='E' &&  bstbuf[indx-1]=='!')) {
        inputready = true;
        Serial.println(bstbuf);
        //Todo: send mqtt confirmation
        clear();
        return;
      }
      character = ' ';    //Replace LF with spaces
    }
    bstbuf[indx] = character;
    indx +=1;
    if (indx>63) clear();   //buffer overflow
  }
}


// Establish contact with BST900
boolean mqttProc::bst900_init() {
  if (!Serial1) return false;
  clear();
  serialFlush();
  Serial1.write('\n');
  return true;
}

/* could be issues if bst fails to respond to a command
 *  suggest putting sequence number on each message to assist with ensuring response.
 *  Or else reflect command back with OK.
 */

//Disable boost converter output
void mqttProc::stop_bst900() {
  //if (bst_output_enabled == true) Serial.print("OUTPUT 0\n");
  Serial1.print("OUTPUT 0\n");
  Serial1.flush();
  bst_output_enabled = false;
}


//Set charger current in Amps
boolean mqttProc::setcurrent_bst900 () {
  if ( bstcurrent == 0.0 ) {
    stop_bst900();
    return true;
  }
  char bstoutbuf[32];
  sprintf(bstoutbuf, "CURRENT %3.2f OUTPUT 1\n", bstcurrent);
  Serial.print("BST: "); Serial.println(bstoutbuf);   //debug
  Serial1.print(bstoutbuf);
  Serial1.flush();
  bst_output_enabled = true;
  return true;
}

//Send text string to BST900
void mqttProc::bst_send_text(String text) {
  Serial1.println(text);
}
