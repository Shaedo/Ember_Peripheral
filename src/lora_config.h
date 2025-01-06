 /*** LORA CONFIG***/ 


    #define RSSI_INIT_TX_THRESHOLD -50 
    // attempt to prevent simultaneous transmissions from multipe devices
    // the effectiveness is debatable and this step may be removed
    // maximum amount of detected signal noise (eg from another device) that will prevent attempted transmission.
    // ie Received Signal Strength Indicator (RSSI) must be < RSSI_INIT_TX_THRESHOLD before a transmission is attempted
    /* 
        while(radio.getRSSI(false) > RSSI_INIT_TX_THRESHOLD) {delay(100);} 
    */

    #define LoRa_NSS 8
    #define LoRa_DIO1 14
    #define LoRa_NRST 12
    #define LoRa_BUSY 13

    #define LoRa_CARRIER_FREQUENCY      915.0 // 434 MHz // float // 
        //Allowed Ranges: Radio_lib:150 - 960khz (software); sx1262:8-1000khz (hardware);  sx1272 125-1000khz (hardware)
        //Frequency (fq) must be within 30% of tx/rx units. Maybe affected by heat. ↑fq → ↑transmission rate & ↓ in link budget.
    #define LoRa_BANDWIDTH              125.0  // 125.0 kHz // float
        // Band Width(bw) ↑bw → ↑transmission distance + ↑ transmission time - doubling bandwidth ~ correlates 3dB ↓ in link budget.
    #define LoRa_SPREADING_FACTOR       9     // 9 // unit8_t // range: 7*-12  *spreading factors < 9 consistently fail
        // Spreading Factor(sf) ↑sf → ↑transmission distance + ↑ transmission time - every increase in sf doubles transmission time
    #define LoRa_CODING_RATE            7   // 7 // unit8_t 
        // Coding Rate(cr) '7' == cr of 4/7 , ↑cr → ↑interferance tolerance + ↑ transmission time
    #define LoRa_SYNC_WORD              RADIOLIB_SX126X_SYNC_WORD_PRIVATE     // RADIOLIB_SX126X_SYNC_WORD_PRIVATE // unit8_t
    #define LoRa_OUTPUT_POWER           10    // 10dBm // range -9 to 22dBm
    #define LoRa_PREAMBLE_LENGTH        8     // 8 symbols
    /*
    Other RadioLib defaults set in radio.begin()
    //TCXO reference voltage: 1.6 V (SX126x module with TCXO) // float
    //LDO regulator mode: disabled (SX126x module with DC-DC power supply) // bool
    //Over-current protection: 60.0 mA
    //DIO2 as RF switch control: enabled
    //LoRa header mode: explicit
    //LoRa CRC: enabled, 2 bytes
  */

 /*** LORA MESSAGE TYPE***/
  #define REQUESTTYPE_REPORT 10 //controller -> peripheral
  #define RETURNTYPE_REPORT 101 // peripheral -> controller