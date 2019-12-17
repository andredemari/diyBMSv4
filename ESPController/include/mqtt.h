
#ifndef mqtt_H_
#define mqtt_H_

#include <Arduino.h>
#include "defines.h"
#include "ArduinoJson.h"
#include "defines.h"
#include <AsyncMqttClient.h>
#include "PacketRequestGenerator.h"
#include "settings.h"


class mqttProc {
   public:
      void begin();
      void processCommand(char* payload);
      void sendModuleStatus(uint8_t bank, uint8_t module);
      void lostcomms();                    //timeout has expired
      bool mqttRelayControl;              //flag to signal relays are being controlled via mqtt
      bool active;                        //flag to signal mqtt command received
      uint8_t mqttRelay[RELAY_TOTAL];     //Relay status determined by mqtt commands
  private:
      uint8_t BypassOverTempShutdown;
      uint8_t dischargepower;
      uint16_t Internal_BCoefficient;
      uint16_t External_BCoefficient;
      uint16_t BypassThresholdmV;
      float Calibration;
      float mVPerADC;
      float bstcurrent;       //Current to set external charger to
      float LoadResistance;
      char value[256];        //Buffer for mqtt messages



};

enum MQTT_COMMAND: uint8_t
{
    Mqtt_restart=1,
    Mqtt_balance=2,
    Mqtt_balance_cancel=3,
    Mqtt_cell_config=40,          //cell configuration parameters
    Mqtt_GlobalSettings=6,
    Mqtt_GlobalCellSettings=41,
    Mqtt_ReportConfiguration=8,
    Mqtt_EnableCharger=10,
    Mqtt_DisableCharger=11,
    Mqtt_EnableInverter=12,
    Mqtt_DisableInverter=13


};


//TODO: Mixing of classes, static and extern is not great
extern AsyncMqttClient mqttClient;
extern PacketRequestGenerator prg;
extern diybms_eeprom_settings mysettings;
extern uint16_t ConfigHasChanged;
extern bool rule_outcome[RELAY_RULES];
#endif
