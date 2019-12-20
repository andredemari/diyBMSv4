
#ifndef mqtt_H_
#define mqtt_H_

#include <Arduino.h>
#include "defines.h"
#include "ArduinoJson.h"
//#include "defines.h"
#include <AsyncMqttClient.h>
#include "PacketRequestGenerator.h"
#include "settings.h"
#include <INA.h>
#include <Wire.h>

#define FAN_ADDRESS 19     //Address of PWM FanController
//If we send a cmdByte with BIT 6 set its a command byte which instructs the cell to do something (not for reading)
#define COMMAND_BIT 6

//Variables for AC Power monitor
#define PMON_ADDRESS 18    //I2C address of AC power monitor

#define READ_vrms 10    //Read Voltage AC rms
#define READ_irms 11    //Read Current AC rms
#define READ_tempA 12
#define READ_power 13   // Read power in Watts
#define READ_powerfactor 14
#define READ_ACvoltage_calibration 15
#define READ_ACcurrent_calibration 16
#define COMMAND_set_ACvoltage_calibration 4
#define COMMAND_set_ACcurrent_calibration 5

//Defines for PWM fan Controller
#define FAN_ADDRESS 19     //Address of PWM FanController
#define PWM_COMMAND_save_config 2                   //Save configuration to EEPROM
#define PWM_COMMAND_factory_default 3
#define PWM_COMMAND_set_temperature_threshold 4     //Fan threshold
#define PWM_COMMAND_set_override_fan 5              //Sets duty cycle manually
#define PWM_COMMAND_set_Kp  6                        // Sets PID Kp parameter
#define PWM_COMMAND_set_Ki  7                        // Sets PID Ki parameter
#define PWM_COMMAND_set_Kd  8                        // Sets PID Kd parameter

#define PWM_READ_fan_alarm 10
#define PWM_READ_fan_speed 11
#define PWM_READ_temperature 12


class mqttProc {
   public:
      void begin();
      void processCommand(char* payload);
      void sendModuleStatus(uint8_t bank, uint8_t module);
      void lostcomms();                    //timeout has expired
      bool initINA();
      uint8_t INAdetected;                  //Number of INA decices detected on I2C
      void updatebus();
      INA_Class INA;      //INA226 Current Voltage Sense
      bool mqttRelayControl;              //flag to signal relays are being controlled via mqtt
      bool active;                        //flag to signal mqtt command received
      uint8_t mqttRelay[RELAY_TOTAL];     //Relay status determined by mqtt commands

      //Functions to manage BST900 boost converter
      void bst_send_text(String text);
      boolean setcurrent_bst900 ();
      boolean bst900_init();
      void bst_process();
      void serialFlush();
      bool inputready;      //BST900 ready

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
      uint8_t indx;
      char value[256];        //Buffer for mqtt messages

      //BST900
      void stop_bst900();
      void clear();
      char bstbuf[64];
      bool bst_output_enabled;

      //INA226
      const float ina_correction = 2.13;   //Correction factor for voltage divider
      float bus_voltage;
      float bus_amps;
      float bus_watts;

      //I2C Functions
      uint8_t send_command(uint8_t i2c_address, uint8_t cmd);
      uint8_t send_command(uint8_t i2c_address, uint8_t cmd, uint8_t byteValue);
      uint8_t send_command(uint8_t i2c_address, uint8_t cmd, float floatValue);
      uint8_t send_command(uint8_t i2c_address, uint8_t cmd, uint16_t Value);
      uint8_t cmdByte(uint8_t cmd);
      uint16_t read_uint16_from_i2c(uint8_t i2c_address, uint8_t cmd);
      uint8_t read_uint8_t_from_i2c(uint8_t i2c_address, uint8_t cmd);
      float read_float_from_i2c(uint8_t i2c_address, uint8_t cmd);
      uint8_t  i2cstatus;



      //Fan controller

      void pwm_command_save_config();           //Save power mon config
      void pwm_command_factory_reset();         //Return to default settings
      float read_fan_temperature(uint8_t fan);  //Read fan temperature
      uint16_t read_fan_speed(uint8_t fan);     //Read fan speed
      uint8_t command_set_override_fan(uint8_t fan, uint16_t  value);   //Manually set fan speed
      uint8_t command_set_temperature_threshold(uint8_t fan, float value);    //Set target temperature
      bool get_fan_data();         //Read all fan data
      uint16_t fan_speed[3];
      float fan_temperature[3];
      uint8_t fan_alarm;

      //AC Power Monitor
      uint8_t SetACVoltCalib(float value);
      uint8_t SetACCurrentCalib(float value);
      float read_powmon_voltcalib();
      float read_powmon_currcalib();

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



union {
  float val;
  uint8_t buffer[4];
} float_to_bytes;

union {
  uint16_t val;
  uint8_t buffer[2];
} uint16_t_to_bytes;

//TODO: Mixing of classes, static and extern is not great
extern AsyncMqttClient mqttClient;
extern PacketRequestGenerator prg;
extern diybms_eeprom_settings mysettings;
extern uint16_t ConfigHasChanged;
extern bool rule_outcome[RELAY_RULES];
#endif
