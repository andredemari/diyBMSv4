
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
      void processCommand(char* payload);
  private:



};

enum MQTT_COMMAND: uint8_t
{
    Mqtt_restart=1,
    Mqtt_balance=2,
    Mqtt_balance_cancel=3,
    Mqtt_cell_config=40,          //cell configuration parameters
    Mqtt_IntTempBCoef=5,
    Mqtt_GlobalSettings=6,
    Mqtt_GlobalCellSettings=41,
    Mqtt_ReportConfiguration=8



};


//TODO: Mixing of classes, static and extern is not great
extern AsyncMqttClient mqttClient;
extern PacketRequestGenerator prg;
extern diybms_eeprom_settings mysettings;
extern uint16_t ConfigHasChanged;
extern bool rule_outcome[RELAY_RULES];
#endif
