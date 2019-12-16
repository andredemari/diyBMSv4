#include "mqtt.h"
#include "ArduinoJson.h"
#include "defines.h"
#include <AsyncMqttClient.h>
#include "PacketRequestGenerator.h"
#include "settings.h"


void mqttProc::processCommand(char* payload) {
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
  float Calibration=0.0, mVPerADC=0.0, LoadResistance=0.0;
  uint8_t BypassOverTempShutdown=0;
  uint16_t Internal_BCoefficient=0, External_BCoefficient=0, BypassThresholdmV=0;
  char value[256];        //Buffer for mqtt messages
  //Decode MQTT Command
  switch(command) {
    case Mqtt_restart:                                     // Command 1 - resetESP32
      Serial.println("Restarting Controller");
      delay(1000);
      ESP.restart();
      break;

    case  Mqtt_cell_config:                                     // Command 40 - Set Cell Module Parameters
      if( mqtt_json.containsKey("calib")) Calibration = mqtt_json["calib"];
      if( mqtt_json.containsKey("intB")) Internal_BCoefficient = mqtt_json["intB"];
      if( mqtt_json.containsKey("extB")) External_BCoefficient = mqtt_json["extB"];
      if( mqtt_json.containsKey("mvADC")) mVPerADC = mqtt_json["mvADC"];
      if( mqtt_json.containsKey("load")) LoadResistance = mqtt_json["load"];
      if( mqtt_json.containsKey("bpT")) BypassOverTempShutdown = mqtt_json["bpT"];
      if( mqtt_json.containsKey("bpmV")) BypassThresholdmV = mqtt_json["bpmV"];
      prg.sendSaveSetting(bank, module,BypassThresholdmV,BypassOverTempShutdown,LoadResistance,Calibration,mVPerADC,Internal_BCoefficient,External_BCoefficient);
      prg.sendGetSettingsRequest(bank, module);  //Now immediately read settings back again
      break;

    case Mqtt_GlobalCellSettings:
      if( mqtt_json.containsKey("bpmV")) BypassThresholdmV = mqtt_json["bpmV"];
      if( mqtt_json.containsKey("bpT")) BypassOverTempShutdown = mqtt_json["bpT"];
      prg.sendSaveGlobalSetting(BypassThresholdmV, BypassOverTempShutdown);
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
          sprintf(value, "{\"address\":%d,\"calib\":%3.2f,\"intB\":%d,\"extB\":%d,\"mvADC\":%3.2f,\"load\":%3.2f,\"bpT\":%d,\"bpmV\":%d}",
            ((b<<4)||i), cmi[b][i].Calibration, cmi[b][i].Internal_BCoefficient,cmi[b][i].External_BCoefficient,cmi[b][i].mVPerADC,
            cmi[b][i].LoadResistance,cmi[b][i].BypassOverTempShutdown,cmi[b][i].BypassThresholdmV);
          mqttClient.publish(MQTTSUBJECT, 0, true, value);
        }
      }
    }
    break;


    default:
      break;
  }
}
