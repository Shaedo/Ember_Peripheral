/* LORA CONTROLLER - DATA REQUEST */

/******* SUMMARY *****
  *PIN-OUT*
    https://resource.heltec.cn/download/Wireless_Stick_Lite_V3/HTIT-WSL_V3.png

Pin  37 = Bat ADC control
Pin  1 = Bat Volt measure
Pin  35 = LED
*/
/***BATTERY******
 -  Reads the battery voltage (pin 1) for the Heltec Wireless Stick Lite(V3)
 -  The Analog-to-Digital Converter (ADC) reader (pin 37) is controlled by pin 1 to prevent current leak
 -  Conversion to voltage = rawVoltageReading * 5.11 *(3.3/1024);
 -  5.11 is a correction for the voltage divider and may change from unit to unit. 
    It should be 4.9 according to the specs. 
    Some users have found it needs to be higher
 -  If the board is plugged into USB/Power then it will take this as the battery voltage
 -  Rx should convert back to uint16_t by: 
        uint16_t rawVoltageReading  = ((uint16_t)loraData[5] << 8) | (uint16_t)loraData[4];
*/

/******* INCLUDES ***********/
  #include <Arduino.h>
  #include "lora_config.h"
  #include <RadioLib.h>
  #include <EEPROM.h>

/******* FUNCTION PROTOTYPES ***********/
  void setup();
  void initLoRa();
  void VBAT_Init();
  bool checkRegistered();

  void loop();

  void loraRx();

  void updateID(uint8_t new_id);

  uint16_t readBattVoltage();
  void compileReport();

  void requestNewID();

  void loraTx();

  void deepSleep(unsigned long &sleepDuration);

/******* GLOBAL VARS ***********/
  /*Prod/Dev/Test*/
  #define REPORTING_MODE 2 //0 = prod mode, 1 = include comments, 2 = dev (resests all, eg wipes EEPROM records etc)
  #define RESET_EEPROM_REGISTRATION 1 // 0 = prod, 1 = always set as unregistered (wipes EEPROM registration entry)
  #define SERIAL_BAUD_RATE 115200

  /**/
  #define EEPROM_SIZE 1 // bytes assigned to eeprom - stores ID_g
  #define CPU_FREQUENCY 80 // max 240. min 40. min 80 for WiFi
  #define PERIPHERAL_TYPE 0
  #define SLEEP_MIN_MS 3000 

  /*LORA*/
  #define LoRa_BUFFER 12 // bytes tx/rx each LoRa transmission
  uint8_t loraData[LoRa_BUFFER]; // array of all tx/rx LoRa data
  // Flag for current LoRa cycle - loops around from MIN_CURRENT_CYCLE to 255 
  // Prevents relays from doubling up on requests etc. This step is not neccesarry for simple systems (no relays, no cross transmissions), but has low overhead
  uint8_t currentLoRaCycle_g = 50; // 1 to 5 are reserved for specific requests eg initial registration requests
  uint8_t PeripheralID_g = 0;

  /*BATTERY*/
  #define VBAT_PIN 1
  #define BAT_ADC_CONTROL 37
  #define BATTERY_SAMPLES 20
  #define ANALOG_READ_RESOLUTION 10

/******* INSTANTIATED CLASS OBJECTS ***********/
  SX1262 radio = new Module(LoRa_NSS, LoRa_DIO1, LoRa_NRST, LoRa_BUSY); // see lora_config.h

/******* INIT - SETUP ***********/
  void setup(){
    #if REPORTING_MODE > 0
        Serial.begin(SERIAL_BAUD_RATE);
        Serial.println("Initializing ... ");
        delay(500);
    #endif 

    #if CPU_FREQUENCY != 0
      setCpuFrequencyMhz(CPU_FREQUENCY);
    #endif

    EEPROM.begin(EEPROM_SIZE);

    initLoRa();
    delay(250);
    VBAT_Init();
    delay(250);

    // If this Peripheral doesn't have a registered ID, request one via LoRa
    if(!checkRegistered()) requestNewID();

    #if REPORTING_MODE > 0
    else Serial.println("Setup Complete.");
    #endif
//status = IDLE;
  }
  void initLoRa(){
    #if REPORTING_MODE > 0
      Serial.print(F("[SX1262] Initializing ... "));
    #endif
    int state = radio.begin(LoRa_CARRIER_FREQUENCY,
                            LoRa_BANDWIDTH,
                            LoRa_SPREADING_FACTOR,
                            LoRa_CODING_RATE,
                            LoRa_SYNC_WORD,
                            LoRa_OUTPUT_POWER,
                            LoRa_PREAMBLE_LENGTH );

    if (state == RADIOLIB_ERR_NONE) {
      #if REPORTING_MODE > 0
        Serial.println(F("LoRa Init success!"));
      #endif
    } else {
      #if REPORTING_MODE > 0
        Serial.print(F("LoRa Init failed, code "));
        Serial.println(state);
      #endif
      while (true);
    }
  }
  void VBAT_Init() {
    pinMode(VBAT_PIN, INPUT);
    adcAttachPin(VBAT_PIN);
    analogReadResolution(ANALOG_READ_RESOLUTION);
    pinMode(BAT_ADC_CONTROL, OUTPUT); // ADC_Ctrl
  }
  bool checkRegistered(){
    #if RESET_EEPROM_REGISTRATION > 0
        Serial.println("Resetting EEPROM Registration");
        EEPROM.write(0, PeripheralID_g);//EEPROM.put(address, param);
        EEPROM.commit();
    #endif

    //false = unregistered, -> registration tx to controller
    byte i = EEPROM.read(0);
    #if REPORTING_MODE > 0
      Serial.println("EEPROM 0 ID is: "+String(i));
    #endif
   
    if(i!=255 && i!=0){
      PeripheralID_g = i;
      return true;
    }
    else return false;
  }


/******* LOOP ***********/
  void loop() {
      loraRx();
  }
 
  void loraRx(){
    //loraData[x]:  
    //cycle[0] | message-type[1] | peripheral_id[2] |

    int SX1262state = radio.receive(loraData, LoRa_BUFFER);

    if (SX1262state == RADIOLIB_ERR_NONE) {// packet was successfully received

      #if REPORTING_MODE > 0
        Serial.println("LoRa packet recieved");
      #endif 

      if(currentLoRaCycle_g != (int)loraData[0]){
        currentLoRaCycle_g = (int)loraData[0];

        #if REPORTING_MODE > 0 
            Serial.print("PROCESSING Rx: ");
            for(byte i=0;i<LoRa_BUFFER;i++){
              Serial.print(int(loraData[i]));
              Serial.print(",");
            }
            Serial.println(" ");
        #endif

        switch(loraData[1])  {
          case REQUESTTYPE_UPDATE_ID:{
            updateID(loraData[3]);
          }
          break;
          case REQUESTTYPE_REPORT:{
            compileReport();
          }
          break;
          case REQUESTTYPE_SLEEP:{
            #if REPORTING_MODE > 0
              Serial.println("REQUESTTYPE_SLEEP");
            #endif 
            //duration: convert four bytes to a single unsigned long
            unsigned long sleepDuration = ((int)loraData[6] << 24) | ((int)loraData[5] << 16) | ((int)loraData[4] << 8) | (int)loraData[3];
            deepSleep(sleepDuration);
            break;
          }
          default:
            #if REPORTING_MODE > 0
              Serial.println("UNKOWN REQUEST TYPE : "+String(loraData[1]));
            #endif 
          break;
        }
      }
      #if REPORTING_MODE > 0
      else Serial.println("matching LoRa cycle numbers ");
      #endif
    }
    #if REPORTING_MODE > 0
    else{  
      if (SX1262state == RADIOLIB_ERR_RX_TIMEOUT) {// timeout occurred while waiting for a packet
      //     Serial.println(F("timeout!"));
      } 
      else if (SX1262state == RADIOLIB_ERR_CRC_MISMATCH) {// packet was received, but is malformed
        Serial.println(F("CRC error!"));
      } 
      else {// some other error occurred
          Serial.print(F("failed, code "));
          Serial.println(SX1262state);
      }
    }
    #endif
  }

  void updateID(uint8_t new_id){

    #if REPORTING_MODE > 0
      Serial.println("UPDATEING ID.... ");
      Serial.println("OLD ID.... " + String(PeripheralID_g));
      Serial.println("NEW ID.... " + String(new_id));
    #endif

    PeripheralID_g = new_id;
    EEPROM.write(0, PeripheralID_g);//EEPROM.put(address, param);
    EEPROM.commit();
  }

  uint16_t readBattVoltage() {
    digitalWrite(BAT_ADC_CONTROL, LOW); // ADC_Ctrl

    uint32_t raw=0;
    for (byte i=0; i<BATTERY_SAMPLES; i++) {
    raw += analogRead(VBAT_PIN);
    }
    raw=raw/BATTERY_SAMPLES;
    digitalWrite(BAT_ADC_CONTROL, HIGH); // ADC_Ctrl
    return raw;
  }

  void compileReport(){

    #if REPORTING_MODE > 0
       Serial.println("Compiling report...");
    #endif

    //REPORT:        cycle[0] | report-type[1] | peripheral_id[2] | rssi recieved from controller[3] | Battery (Raw 1024) [4,5] | sensor data[6...LoRa_BUFFER]
    currentLoRaCycle_g ++;
    if(currentLoRaCycle_g <6) currentLoRaCycle_g = 6; // 1 to 5 are reserved for specific requests eg initial registration requests

    loraData[0]=currentLoRaCycle_g;
    loraData[1]=RETURNTYPE_REPORT; // see LORA MESSAGE TYPE in config.h
    loraData[2]=PeripheralID_g;
    loraData[3]=abs(radio.getRSSI()); //Last RSSI reading (from Controller to Peripheral)

    //format battery
    uint16_t vbat = readBattVoltage(); 
    byte byte1 = (vbat & 0x000000ff);
    byte byte2 = (vbat & 0x0000ff00) >> 8;
    loraData[4]=byte1;
    loraData[5]=byte2;
    //Rx should convert back to uint16_t by: uint16_t rawVoltageReading  = ((uint16_t)loraData[1] << 8) | (uint16_t)loraData[0];

    loraData[6]=5; // Report Length: length so 0 if no report, 5 if using maximum report length
    
    //Replace with function call to eg sensor-type-specific module
    loraData[7]='a';
    loraData[8]='b';
    loraData[9]='c';
    loraData[10]='d';
    loraData[11]='e';

    loraTx();
  }

  void requestNewID(){
    #if REPORTING_MODE > 0
       Serial.println("Requesting registration/id...");
    #endif
    //REPORT:        cycle[0] | report-type[1] | peripheral_id[2] | rssi recieved from controller[3] | Battery (Raw 1024) [4,5] | sensor data[6...LoRa_BUFFER]
    loraData[0]=1;
    loraData[1]=RETURNTYPE_REGISTRATION; // see LORA MESSAGE TYPE in config.h
    loraData[2]=PeripheralID_g;
    loraTx();
  }

  void loraTx(){

    // wait for clear airways before attempting tx
    while(radio.getRSSI(false) > RSSI_INIT_TX_THRESHOLD){
      #if REPORTING_MODE > 0
        Serial.println("RSSI: "+ String(radio.getRSSI(false)));
      #endif
      delay(100);
    }

    // transmit data
    int SX1262state = radio.transmit(loraData, LoRa_BUFFER);

    // check for errors
    #if REPORTING_MODE > 0
      if (SX1262state == RADIOLIB_ERR_NONE) {// the packet was successfully transmitted
        Serial.println("TX success. Datarate: "+ String(radio.getDataRate()) + "bps");
      } else if (SX1262state == RADIOLIB_ERR_PACKET_TOO_LONG) {// the supplied packet was longer than 256 bytes
        Serial.println(F("too long!"));
      } else if (SX1262state == RADIOLIB_ERR_TX_TIMEOUT) {// timeout occured while transmitting packet
        Serial.println(F("timeout!"));
      } else {// some other error occurred
        Serial.println("TX Fail code: "+String(SX1262state));
      }
      Serial.print("RECIEVING: ");
    #endif 
  }
 
  void deepSleep(unsigned long &sleepDuration){
   if(sleepDuration<SLEEP_MIN_MS) sleepDuration = SLEEP_MIN_MS;
   // Short sleep durations can result in attempting to send when should be sleeping

    #if REPORTING_MODE > 0
      Serial.println ("Sleep for: " + String(sleepDuration));
    #endif
    radio.sleep();
    SPI.end();

    pinMode(LoRa_DIO1,ANALOG);
    pinMode(LoRa_NSS,ANALOG);
    pinMode(LoRa_NRST,ANALOG);
    pinMode(LoRa_BUSY,ANALOG);
    //    pinMode(LORA_CLK,ANALOG);
    //    pinMode(LORA_MISO,ANALOG);
    //    pinMode(LORA_MOSI,ANALOG);
    esp_sleep_enable_timer_wakeup(sleepDuration*(uint64_t)1000);
    esp_deep_sleep_start();
  }
/*END*/
