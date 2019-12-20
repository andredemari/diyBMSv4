#include <Arduino.h>
#include "mqtt.h"
#include "ArduinoJson.h"
//#include "defines.h"
#include <AsyncMqttClient.h>
#include "PacketRequestGenerator.h"
#include "settings.h"
#include <INA.h>
#include <Wire.h>



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
  StaticJsonDocument<512> mqtt_json;
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



//Functions to manage INA226 current/voltage sensor

bool mqttProc::initINA() {
  //Setup INA current sensor
  INAdetected = INA.begin(20,SHUNT);                               // Set expected 20 Amp max and resistor 1.5 mohm   //
  if (INAdetected == 0) return false;
  Serial.print(INAdetected); Serial.println(" INA devices detected");
  INA.setBusConversion(10000);                                                 // Maximum conversion time 8.244ms  //
  INA.setShuntConversion(10000);                                               // Maximum conversion time 8.244ms  //
  INA.setAveraging(1024);                                                     // Average each reading n-times     //
  INA.setMode(INA_MODE_CONTINUOUS_BOTH);                                      // Bus/shunt measured continuously
  String bus_readings= "";
  bus_readings += String(INA.getDeviceName()) + "  ";
  bus_readings += String((float)INA.getBusMilliVolts()/1000.0) + " V  ";    // convert mV to Volts
  bus_readings += String((float)INA.getBusMicroAmps()/1000.0) + " mA  ";     // convert uA to Milliamps
  bus_readings += String((float)INA.getBusMicroWatts() / 1000.0) + " mW  "; // convert uA to Milliwatts
  Serial.println(bus_readings);
  return true;
}


//Send bus voltage/current to MQTT
void mqttProc::updatebus() {
  if(INAdetected == 0) return;

  StaticJsonDocument<512> buffer;

  //Read INA226 sensor
  bus_voltage = ina_correction * INA.getBusMilliVolts()/1000.0;
  bus_amps = (float)INA.getBusMicroAmps()/1000000.0;
  bus_watts = bus_amps*bus_voltage;
  char bus_readings[64];
  bus_watts = (int)(bus_watts*100)/100.0;
  bus_voltage = (int)(bus_voltage*100)/100.0;
  bus_amps = (int)(bus_amps*100)/100.0;
  sprintf(bus_readings, "Bus %3.2fV, %3.2fA, %3.2fW", bus_voltage, bus_amps, bus_watts);
  Serial.println(bus_readings);

  buffer["busvolts"] = bus_voltage;
  buffer["busamps"] = bus_amps;
  buffer["buswatts"] = bus_watts;
  buffer["charging"] = digitalRead(CHARGER);      //Read the relay state
  buffer["discharging"] = digitalRead(INVERTER);

  //Reset INA after each reading to avoid lockups
  //Is this needed any longer?
  //INA.reset();
  //INA.setBusConversion(10000);                                                 // Maximum conversion time 8.244ms  //
  //INA.setShuntConversion(10000);                                               // Maximum conversion time 8.244ms  //
  //INA.setAveraging(1024);                                                   // Average each reading n-times     //
  //INA.setMode(INA_MODE_CONTINUOUS_BOTH);

  //Now read AC Power Monitor readings
  float powerMonitorTemp= read_float_from_i2c(PMON_ADDRESS, READ_tempA);
  float powerfactor= read_float_from_i2c(PMON_ADDRESS, READ_powerfactor);
  float power= read_float_from_i2c(PMON_ADDRESS, READ_power);
  float vrms= read_float_from_i2c(PMON_ADDRESS, READ_vrms);
  float irms= read_float_from_i2c(PMON_ADDRESS, READ_irms);

  if(get_fan_data()) {              //Read data from fan controller. If data sane include in mqtt message
    buffer["f_alarm"] = fan_alarm;
    buffer["f1_spd"] = fan_speed[0];
    buffer["f2_spd"] = fan_speed[1];
    buffer["f3_spd"] = fan_speed[2];
    if(isnan(fan_temperature[0])) buffer["f1_temp"] = fan_temperature[0];
    if(isnan(fan_temperature[1])) buffer["f2_temp"] = fan_temperature[1];
    if(isnan(fan_temperature[2])) buffer["f3_temp"] = fan_temperature[2];
    buffer["pmon_temp"] = powerMonitorTemp;
    buffer["power"] = power;
    buffer["vrms"] = vrms;
    buffer["irms"] = irms;
    buffer["pfact"] = powerfactor;
  }

  char output[512];
  serializeJson(buffer, output);
  //mqttClient.publish(MQTTSUBJECT, 0, true, output);
  mqttClient.publish("diybms_temp", 0, true, output);        //Todo: Replace with correct Topic
  return;
}

//Functions to manage I2C bus
//(From Stuart Pittaways's diyBMS ver3)

uint8_t  mqttProc::send_command(uint8_t i2c_address, uint8_t cmd) {
  Wire.beginTransmission(i2c_address); // transmit to device
  Wire.write(cmd);  //Command configure device address
  uint8_t ret = Wire.endTransmission();  // stop transmitting
  return ret;
}

uint8_t  mqttProc::send_command(uint8_t i2c_address, uint8_t cmd, uint8_t byteValue) {
  Wire.beginTransmission(i2c_address); // transmit to device
  Wire.write(cmd);  //Command configure device address
  Wire.write(byteValue);  //Value
  uint8_t ret = Wire.endTransmission();  // stop transmitting
  return ret;
}

uint8_t  mqttProc::send_command(uint8_t i2c_address, uint8_t cmd, float floatValue) {
  float_to_bytes.val = floatValue;
  Wire.beginTransmission(i2c_address); // transmit to device
  Wire.write(cmd);  //Command configure device address
  Wire.write(float_to_bytes.buffer[0]);
  Wire.write(float_to_bytes.buffer[1]);
  Wire.write(float_to_bytes.buffer[2]);
  Wire.write(float_to_bytes.buffer[3]);
  uint8_t ret = Wire.endTransmission();  // stop transmitting
  return ret;
}

uint8_t mqttProc::send_command(uint8_t i2c_address, uint8_t cmd, uint16_t Value) {
  uint16_t_to_bytes.val = Value;
  Wire.beginTransmission(i2c_address); // transmit to device
  Wire.write(cmd);  //Command configure device address
  Wire.write(uint16_t_to_bytes.buffer[0]);
  Wire.write(uint16_t_to_bytes.buffer[1]);
  uint8_t ret = Wire.endTransmission();  // stop transmitting
  return ret;
}

uint8_t mqttProc::cmdByte(uint8_t cmd) {
  bitSet(cmd, COMMAND_BIT);
  return cmd;
}


uint16_t mqttProc::read_uint16_from_i2c(uint8_t i2c_address, uint8_t cmd) {
  send_command(i2c_address, cmd);
  i2cstatus = Wire.requestFrom((uint8_t)i2c_address, (uint8_t)2);
  return (word((uint8_t)Wire.read(), (uint8_t)Wire.read()));
}

uint8_t mqttProc::read_uint8_t_from_i2c(uint8_t i2c_address, uint8_t cmd) {
  send_command(i2c_address, cmd);
  i2cstatus = Wire.requestFrom((uint8_t)i2c_address, (uint8_t)1);
  return (uint8_t)Wire.read();
}

float mqttProc::read_float_from_i2c(uint8_t i2c_address, uint8_t cmd) {
  send_command(i2c_address, cmd);
  i2cstatus = Wire.requestFrom((uint8_t)i2c_address, (uint8_t)4);
  float_to_bytes.buffer[0] = (uint8_t)Wire.read();
  float_to_bytes.buffer[1] = (uint8_t)Wire.read();
  float_to_bytes.buffer[2] = (uint8_t)Wire.read();
  float_to_bytes.buffer[3] = (uint8_t)Wire.read();
  return float_to_bytes.val;
}


//Functions to manage I2C fan controller
//======================================
// The I2C fan controller is on address 19. It has an STM32 with which it can control three fans
//Fan1 is for the Cisco 1300W Power Supply at 40V
//Fan2 is for the BST900 boost converter MOSFET/Diodes
//Fan 3 is for the BST900 coil
//Each fan has a temperature sensor associated with it.
//The STM32 runs PID algorithms  for each fan circuit.

//PWM fan controller commands

bool mqttProc::get_fan_data() {
  fan_alarm = read_uint8_t_from_i2c(FAN_ADDRESS, PWM_READ_fan_alarm);
  if(fan_alarm == 0xff) return false;     //Not sane data
  for(uint8_t i=0; i< 3; i++ ) {
    read_fan_speed(i);
    read_fan_temperature(i);
  }

  return true;
}

void mqttProc::pwm_command_save_config() {
  send_command(FAN_ADDRESS, cmdByte( PWM_COMMAND_save_config ));
  return;
}

void mqttProc::pwm_command_factory_reset() {
  send_command(FAN_ADDRESS, cmdByte( PWM_COMMAND_factory_default ));
  return;
}

float mqttProc::read_fan_temperature(uint8_t fan) {
  if (fan >2 ) return -1;
  fan_temperature[fan] = read_float_from_i2c(FAN_ADDRESS, PWM_READ_temperature | (fan << 4)); //bits 4 and 5 contain device
  if(isnan(fan_temperature[fan] )) return false;
  return fan_temperature[fan];
}

uint16_t mqttProc::read_fan_speed(uint8_t fan) {
  if (fan > 2 ) return 65535;
  fan_speed[fan] = read_uint16_from_i2c(FAN_ADDRESS, PWM_READ_fan_speed | (fan << 4));
  return fan_speed[fan];
}

uint8_t mqttProc::command_set_override_fan(uint8_t fan, uint16_t  value) {
  return send_command(FAN_ADDRESS, cmdByte(PWM_COMMAND_set_override_fan | (fan << 4)), value);
}

uint8_t mqttProc::command_set_temperature_threshold(uint8_t fan, float value) {
  if (fan > 2 ) return 255;
  return send_command(FAN_ADDRESS, cmdByte(PWM_COMMAND_set_temperature_threshold | (fan << 4) ), value);
}

//Functions for AC power monitor
//==============================
//The AC Power Monitor is an STM32 which has a current sense transformer with which it can measure
//AC Voltage/Current/Power, and Powerfactor
//of the Power Supply Unit and the Inverter

uint8_t mqttProc::SetACVoltCalib(float value) {
  return send_command(PMON_ADDRESS, cmdByte(COMMAND_set_ACvoltage_calibration), value);
}

uint8_t mqttProc::SetACCurrentCalib(float value){
  return send_command(PMON_ADDRESS, cmdByte(COMMAND_set_ACcurrent_calibration), value);
}

float mqttProc::read_powmon_voltcalib(){
  return read_float_from_i2c(PMON_ADDRESS, READ_ACvoltage_calibration);
}

float mqttProc::read_powmon_currcalib(){
  return read_float_from_i2c(PMON_ADDRESS, READ_ACcurrent_calibration);
}
