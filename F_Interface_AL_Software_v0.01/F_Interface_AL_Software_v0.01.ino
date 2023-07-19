#include "pins_arduino.h" 
#include "F_interface_AL.h" 
#include "pins.h" 
#include "config_wheel.h" 
#include <avr/pgmspace.h>


#define dataLength 33
#define PRINTBIN(Num) for (uint32_t t = (1UL << (sizeof(Num) * 8) - 1); t; t >>= 1) //Serial.write(Num& t ? '1' : '0'); // Prints a binary number with leading zeros (Automatic Handling)

fanatec_data_in_t data_in;
fanatec_data_out_t data_out;

uint8_t encoder_plus = 0b00000001;
uint8_t encoder_less = 0b11111111;

void serialOutput();                      // Prints I/O state to Serial Output
void printmosibuf();                      // Prints MOSI buffer to Serial Output
void printmisobuf();                      // Prints MISO buffer to Serial Output
void printHex(int num, int precision);
void printButtonByteToSerial();           // Prints the selected button byte to Serial, if it was changed. The button byte can be selected by sending either A,B or C through the serial monitor. The bits in the selected button byte can either be changed by the serial monitor (sending 1 to 7 raises / lowers a specific bit,) or by pressing one of the physical buttons, if you have any attached.
void cableselect();                       // When CS line goes high - the rim should get ready to transmit the next returnData buffer, right from the begining. (so that the first byte (0xA5) will be sent out on the first Clock cycle (and after CS line went Low)
void calcOutgoingCrc();
bool checkIncomingCrc();
ISR(SPI_STC_vect);           // SPI interrupt routine
uint8_t crc8(const uint8_t* buf, uint8_t length);     // return CRC8 from buf

void buttonBit_buffer_handling();     // handling of button presses buffer, mostly required by impulsion based interfaces like encoders & rotary switches
unsigned long lastButtonPress;

int read_CD74HC4067(int pin);         // read multiplexed analog value through CD74HC4067 16-MUX breakout board connected through A1 analog pin
void inputs_management();             // handles button presses, rotary switches,....

// Current values variables

    // Rotary switches management
    int rs_val[4];                                  // Rotary switch current position, ranging from 0 to 7
    int old_rs_val[4] = {1023, 1023, 1023, 1023};   // Rotary switch previous position, ranging from 0 to 7
    int rs_pos [4] = {0, 0, 0, 0};                  // integrated position of rotary switches
    int current_analog_RS_val[4];                   // Current analog Raw position
    int val_eval, val_eval2, val_eval3, val_eval4;  // Processed value to determine if rotary switch analog value is usable
    
    // Encoders management variables, Left and Right
    volatile bool A_1_set = false;   
    volatile bool B_1_set = false;
    volatile bool A_2_set = false;   
    volatile bool B_2_set = false;
    
    volatile int encoderPos_1 = 0;
    volatile int encoderPos_2 = 0;
    int old_encoderPos_1 = 0;
    int old_encoderPos_2 = 0;

    // Buffer management
    unsigned long buffer_timer;
    int buffer_index = -1;
    bool flag_buffer = LOW;
    uint8_t buttonsBits[] = {1,  2,  3,  4,  5, 6, 7, 8, 9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};   // what bits do you want each button to affect? use the last comment in this file as reference.
    int buttonBits_buffer[24] =      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};       // Button press buffer handling
                
void test_output();   // test inputs of the wheel
int n = 0;
unsigned long timeStamp1;

void buttonBitChange(uint8_t buttonBit, bool bitOn);

volatile unsigned long lastPrintMillis = 0;
int selectedButtonByte = 2;   // button bytes are 3rd to 5th. initialize to 1st relevant byte.
uint8_t incByte, prevPrintedByte, prevAlphaDisp[3];
volatile uint8_t isrIndex = 0;
unsigned long delayMillis = 400; // wait at least the delay time since last spi communication before printing out whatever came in.

// Uni hub packet (change to other steering wheels in setup() )
/*uint8_t returnData[dataLength] = { 0xA5, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc };*/

void setup()
{
    pinMode(MISO_PIN, OUTPUT);    // usually when using SPI, arduino will automatically define MISO as output, but not when doing SPI slave.
    pinMode(CLK_PIN, INPUT_PULLUP);
    pinMode(MOSI_PIN, INPUT_PULLUP);
    pinMode(INT_CS_SPI, INPUT_PULLUP);   // connecting CS_ISR pin (D2) to the SS pin (D10) is crucial. See the schematics.
    pinMode(CS_PIN, INPUT_PULLUP);     

    pinMode(MUX_S0_PIN, OUTPUT);
    pinMode(MUX_S1_PIN, OUTPUT);
    pinMode(MUX_S2_PIN, OUTPUT);
    pinMode(MUX_S3_PIN, OUTPUT);
  
    pinMode(ENC_1_A_PIN, INPUT_PULLUP);
    pinMode(ENC_1_B_PIN, INPUT_PULLUP);
    pinMode(ENC_2_A_PIN, INPUT_PULLUP);
    pinMode(ENC_2_B_PIN, INPUT_PULLUP);

    cableselect();

    // Wheel configuration
    data_out.header = 0xa5;
    data_out.id = 0x03;

    // SPI initerrupts configuration
    attachInterrupt(digitalPinToInterrupt(INT_CS_SPI), cableselect, RISING);
    // SPCR BYTE should be: 11000100   note to self: by raw_capture.ino and fanatec.cpp spi settings, of btClubSportWheel by Darknao, SPI settings are SPI_Mode0 & MSBFIRST. but with logic scope I see that CPHA 1 (falling!) is used by wheel base, which means SPI_MODE1. (and MSBFIRST)
    // (Mode 1 - clock is normally low (CPOL = 0), and the data is sampled on the transition from high to low (trailing edge) (CPHA = 1))
    SPCR |= _BV(SPE);   // turn on SPI
    SPCR |= _BV(SPIE);    // turn on interrupts
    SPCR |= _BV(CPHA);    //turns on CPHA. as I found the CSW wheelbase to use it, with logic scope.
  
    Serial.begin(500000);
    Serial.println("F_Interface_AL");
}

void loop() {

    calcOutgoingCrc();
    bool crc8Stat = checkIncomingCrc();

    //printmosibuf();
    //printmisobuf();

    inputs_management();    // handles button presses, rotary switches,....
    //test_output();
    buttonBit_buffer_handling();   

    serialOutput();          // Prints I/O state to Serial Output
    
}

/////////////////////////////////////////////
// test inputs of the wheel
void test_output() {    
    // TEst Outputs
    if ((millis()-timeStamp1) > 1000) {
        timeStamp1 = millis();
        n++;
        int i = n%24;     // to modulate between 0 and 23
    
        // change the button bit
        buttonBitChange(i, true);
        buttonBitChange(i-1, false);
        
        data_out.axisX = i*10;
        data_out.axisY = i*10;
    }
}

/////////////////////////////////////////////
// Prints I/O state to Serial Output
void serialOutput() {
   Serial.print("B1 : ");
   printHex(data_out.buttons[0], 2);
   Serial.print(" | B2 : ");
   printHex(data_out.buttons[1], 2);
   Serial.print(" | B3 : ");
   printHex(data_out.buttons[3], 2);
   Serial.print(" | Enc : ");
   printHex(data_out.encoder, 2);
   Serial.print(" | X : ");
   printHex(data_out.axisX, 2);
   Serial.print(" | Y : ");
   printHex(data_out.axisY, 2);
   Serial.print(" | Disp : ");
   printHex(data_in.disp[0], 2);
   Serial.print(".");
   printHex(data_in.disp[1], 2);
   Serial.print(".");
   printHex(data_in.disp[2], 2);
   Serial.print(" | Leds : ");
   printHex(data_in.leds, 4);
   Serial.print(" | Rumble : ");
   printHex(data_in.rumble[0], 2);
   Serial.print(".");
   printHex(data_in.rumble[1], 2);
   Serial.println("");
}

/////////////////////////////////////////////
void buttonBitChange(uint8_t buttonBit, bool bitOn) {
   
   //if ((millis() - lastButtonPress) > DEBOUNCE_DELAY) {      // Debounce
      if (bitOn) {    // changes a selected bit in one of the 3 button bytes to either on (1) or off (0). Valid values for buttonBit is 1 to 24. See buttons list in last comment.
          //Serial.println("bit change");
          lastButtonPress = millis();
          if (buttonBit < 23) data_out.raw[2 + ((buttonBit - 1) / 8)] |= (1 << (((buttonBit - 1) % 8)));
          if (buttonBit == 23) data_out.encoder = encoder_plus;
          if (buttonBit == 24) data_out.encoder = encoder_less;
          
      }
      else {
          if (buttonBit < 23) data_out.raw[2 + ((buttonBit - 1) / 8)] &= ~(1 << (((buttonBit - 1) % 8)));
          else {
              data_out.encoder = 0b00000000;
          }
      } 
   //}
}

/////////////////////////////////////////////
void calcOutgoingCrc() {
  data_out.crc = crc8(data_out.raw, dataLength - 1);
  //returnData[dataLength - 1] = crc8(returnData, dataLength - 1);    // calculates crc8 for outgoing packet
}

/////////////////////////////////////////////
bool checkIncomingCrc() {   // crc check for incoming data. returns true if correct crc8 is found in the incoming data
  uint8_t crc = crc8(data_in.raw, dataLength - 1);    ////Serial.print("calc crc");     ////Serial.println(crc,HEX);
  if (crc != data_in.raw[dataLength - 1]) {

  return false;
  }
  else
  return true;
}

/////////////////////////////////////////////
void printmosibuf() {
  Serial.print("MOSI:");
  for (int i = 0; i < dataLength; i++) {
    printHex(data_in.raw[i], 2);
    Serial.print(" ");
  }
  Serial.println();
}

/////////////////////////////////////////////
void printmisobuf() {
  Serial.print("MISO:");
  for (int i = 0; i < dataLength; i++) {
    printHex(data_out.raw[i], 2);
    Serial.print(" ");
  }
  Serial.println();
}

/////////////////////////////////////////////
void printHex(int num, int precision) {
  char tmp[16];
  char format[128];
  sprintf(format, "0x%%.%dX", precision);
  sprintf(tmp, format, num);
  Serial.print(tmp);
}

/////////////////////////////////////////////
void printButtonByteToSerial() {  // Prints the selected button byte to Serial, if it was changed. The button byte can be selected by sending either A,B or C through the serial monitor. The bits in the selected button byte can either be changed by the serial monitor (sending 1 to 7 raises / lowers a specific bit,) or by pressing one of the physical buttons, if you have any attached.
  if (millis() > lastPrintMillis + delayMillis) {
    if (prevPrintedByte != data_out.raw[selectedButtonByte]) {
      PRINTBIN(data_out.raw[selectedButtonByte]);
      Serial.print("      ");
      printHex(data_out.raw[selectedButtonByte], 2);
      Serial.println();
    }
    prevPrintedByte = data_out.raw[selectedButtonByte];
    lastPrintMillis = millis();
  }
}

/////////////////////////////////////////////
// When CS line goes high - the rim should get ready to transmit the next returnData buffer, right from the begining. (so that the first byte (0xA5) will be sent out on the first Clock cycle (and after CS line went Low)
void cableselect() {          
    SPCR &= ~_BV(SPIE);         // turn OFF interrupts
    SPDR = data_out.raw[0];       // load first byte into SPDR single-byte's buffer
    isrIndex = 0;           // on next SPI interrupt(SPI_STC_vect), load the 2nd byte
    SPCR |= _BV(SPIE);          // turn on interrupts
}

/////////////////////////////////////////////
// SPI interrupt routine
ISR(SPI_STC_vect)       // Note: this routine takes 6 us, and happens once 8 bits are recieved through SPI.
{ 
  byte c = SPDR;
  data_in.raw[isrIndex] = c;
  
  isrIndex++;
  if (isrIndex >= dataLength) {
    isrIndex = 0;
  }
  
  SPDR = data_out.raw[isrIndex];
  //SPDR = returnData[isrIndex];
}

/////////////////////////////////////////////
// return CRC8 from buf
uint8_t crc8(const uint8_t* buf, uint8_t length) {
  uint8_t crc = 0xff;
  while (length) {
    crc = pgm_read_byte_near(_crc8_table + (*buf ^ crc));
    buf++;
    length--;
  }
  return crc;
}

/////////////////////////////////////////////
// read multiplexed analog value through CD74HC4067 16-MUX breakout board connected through A1 analog pin
int read_CD74HC4067(int pin) {
    // Configure the input to read
    switch (pin) {
        case 0:
            digitalWrite(MUX_S0_PIN, 0);
            digitalWrite(MUX_S1_PIN, 0);
            digitalWrite(MUX_S2_PIN, 0);
            digitalWrite(MUX_S3_PIN, 0);
            break;
        case 1:
            digitalWrite(MUX_S0_PIN, 1);
            digitalWrite(MUX_S1_PIN, 0);
            digitalWrite(MUX_S2_PIN, 0);
            digitalWrite(MUX_S3_PIN, 0);
            break;
        case 2:
            digitalWrite(MUX_S0_PIN, 0);
            digitalWrite(MUX_S1_PIN, 1);
            digitalWrite(MUX_S2_PIN, 0);
            digitalWrite(MUX_S3_PIN, 0);
            break;
        case 3:
            digitalWrite(MUX_S0_PIN, 1);
            digitalWrite(MUX_S1_PIN, 1);
            digitalWrite(MUX_S2_PIN, 0);
            digitalWrite(MUX_S3_PIN, 0);
            break;
        case 4:
            digitalWrite(MUX_S0_PIN, 0);
            digitalWrite(MUX_S1_PIN, 0);
            digitalWrite(MUX_S2_PIN, 1);
            digitalWrite(MUX_S3_PIN, 0);
            break;
        case 5:
            digitalWrite(MUX_S0_PIN, 1);
            digitalWrite(MUX_S1_PIN, 0);
            digitalWrite(MUX_S2_PIN, 1);
            digitalWrite(MUX_S3_PIN, 0);
            break;
        case 6:
            digitalWrite(MUX_S0_PIN, 0);
            digitalWrite(MUX_S1_PIN, 1);
            digitalWrite(MUX_S2_PIN, 1);
            digitalWrite(MUX_S3_PIN, 0);
            break;
        case 7:
            digitalWrite(MUX_S0_PIN, 1);
            digitalWrite(MUX_S1_PIN, 1);
            digitalWrite(MUX_S2_PIN, 1);
            digitalWrite(MUX_S3_PIN, 0);
            break;
        case 8:
            digitalWrite(MUX_S0_PIN, 0);
            digitalWrite(MUX_S1_PIN, 0);
            digitalWrite(MUX_S2_PIN, 0);
            digitalWrite(MUX_S3_PIN, 1);
            break;
        case 9:
            digitalWrite(MUX_S0_PIN, 1);
            digitalWrite(MUX_S1_PIN, 0);
            digitalWrite(MUX_S2_PIN, 0);
            digitalWrite(MUX_S3_PIN, 1);
            break;
        case 10:
            digitalWrite(MUX_S0_PIN, 0);
            digitalWrite(MUX_S1_PIN, 1);
            digitalWrite(MUX_S2_PIN, 0);
            digitalWrite(MUX_S3_PIN, 1);
            break;
        case 11:
            digitalWrite(MUX_S0_PIN, 1);
            digitalWrite(MUX_S1_PIN, 1);
            digitalWrite(MUX_S2_PIN, 0);
            digitalWrite(MUX_S3_PIN, 1);
            break;
        case 12:
            digitalWrite(MUX_S0_PIN, 0);
            digitalWrite(MUX_S1_PIN, 0);
            digitalWrite(MUX_S2_PIN, 1);
            digitalWrite(MUX_S3_PIN, 1);
            break;
        case 13:
            digitalWrite(MUX_S0_PIN, 1);
            digitalWrite(MUX_S1_PIN, 0);
            digitalWrite(MUX_S2_PIN, 1);
            digitalWrite(MUX_S3_PIN, 1);
            break;
        case 14:
            digitalWrite(MUX_S0_PIN, 0);
            digitalWrite(MUX_S1_PIN, 1);
            digitalWrite(MUX_S2_PIN, 1);
            digitalWrite(MUX_S3_PIN, 1);
            break;
        case 15:
            digitalWrite(MUX_S0_PIN, 1);
            digitalWrite(MUX_S1_PIN, 1);
            digitalWrite(MUX_S2_PIN, 1);
            digitalWrite(MUX_S3_PIN, 1);
            break;
        default:
            digitalWrite(MUX_S0_PIN, 0);
            digitalWrite(MUX_S1_PIN, 0);
            digitalWrite(MUX_S2_PIN, 0);
            digitalWrite(MUX_S3_PIN, 0);
            break;
    }

    // sample and process the value
    int val=0;
    for (int i= 0; i< ANALOG_SAMPLES; i++) {
        val += analogRead(MUX_SIG_PIN);
    }
    val /= ANALOG_SAMPLES; 
 
    // return the value
    return val;
}


/////////////////////////////////////////////
// handles button presses, rotary switches,....
void inputs_management() {

    int val=0;
    /////////////////////////////////////////////
    // Handle button groups 
    // ONLY ONE BUTTON OF THE GROUP CAN BE PRESSED AT A TIME AS THEY ARE MULTIPLEXED WITH SAME RESISTOR VALUE
        
        /////////////////////////////////////////////
        // Group 1 - read analog value and determine which button is pressed according to the analog value 
        #ifdef HAS_BUTTON_GROUP_1
        val = read_CD74HC4067(BUT_1_PIN);
        if      (val < BUT_THRESHOLD_1)  {                            // Button 4 pressed
            buttonBitChange(BUTTON_4_BUTTON_BIT, true);
            if (BUTTON_DEBUG ==1) Serial.println("Button 4");
        }
        else if (val < BUT_THRESHOLD_2) {                             // Button 3 pressed
            buttonBitChange(BUTTON_3_BUTTON_BIT, true);
            if (BUTTON_DEBUG ==1) Serial.println("Button 3");
        }   
        else if (val < BUT_THRESHOLD_3) {                             // Button 2 pressed
            buttonBitChange(BUTTON_2_BUTTON_BIT, true); 
            if (BUTTON_DEBUG ==1) Serial.println("Button 2");
        }     
        else if (val < BUT_THRESHOLD_4)  {                            // Button 1 pressed
            buttonBitChange(BUTTON_1_BUTTON_BIT, true);               
            if (BUTTON_DEBUG ==1) Serial.println("Button 1");
        }     
        else {                                                        // No button pressed
            buttonBitChange(BUTTON_1_BUTTON_BIT, false);
            buttonBitChange(BUTTON_2_BUTTON_BIT, false);
            buttonBitChange(BUTTON_3_BUTTON_BIT, false);
            buttonBitChange(BUTTON_4_BUTTON_BIT, false);
        }
        
        #endif
  
        /////////////////////////////////////////////
        // Group 2 - read analog value and determine which button is pressed according to the analog value 
        #ifdef HAS_BUTTON_GROUP_2
        val = read_CD74HC4067(BUT_2_PIN);
        if      (val < BUT_THRESHOLD_1) {
            buttonBitChange(BUTTON_8_BUTTON_BIT, true);               // Button 8 pressed  
            if (BUTTON_DEBUG ==1) Serial.println("Button 8");
        }
        else if (val < BUT_THRESHOLD_2) {
            buttonBitChange(BUTTON_7_BUTTON_BIT, true);               // Button 7 pressed
            if (BUTTON_DEBUG ==1) Serial.println("Button 7");
        }
        else if (val < BUT_THRESHOLD_3) {
            buttonBitChange(BUTTON_6_BUTTON_BIT, true);               // Button 6 pressed
            if (BUTTON_DEBUG ==1) Serial.println("Button 6");
        }
        else if (val < BUT_THRESHOLD_4) {
            buttonBitChange(BUTTON_5_BUTTON_BIT, true);              // Button 5 pressed
            if (BUTTON_DEBUG ==1) Serial.println("Button 5");
        }
        else {                                                       // No button pressed
            buttonBitChange(BUTTON_5_BUTTON_BIT, false);
            buttonBitChange(BUTTON_6_BUTTON_BIT, false);
            buttonBitChange(BUTTON_7_BUTTON_BIT, false);
            buttonBitChange(BUTTON_8_BUTTON_BIT, false);
        }
        #endif
  
        /////////////////////////////////////////////
        // Group 3 - read analog value and determine which button is pressed according to the analog value 
        #ifdef HAS_BUTTON_GROUP_3
        val = read_CD74HC4067(BUT_3_PIN);
        if      (val < BUT_THRESHOLD_1) {
            buttonBitChange(BUTTON_12_BUTTON_BIT, true);            // Button 12 pressed  
            if (BUTTON_DEBUG ==1) Serial.println("Button 12");
        }
        else if (val < BUT_THRESHOLD_2) {
            buttonBitChange(BUTTON_11_BUTTON_BIT, true);            // Button 11 pressed
            if (BUTTON_DEBUG ==1) Serial.println("Button 11");
        }
        else if (val < BUT_THRESHOLD_3) {
            buttonBitChange(BUTTON_10_BUTTON_BIT, true);            // Button 10 pressed
            if (BUTTON_DEBUG ==1) Serial.println("Button 10");
        }
        else if (val < BUT_THRESHOLD_4) {
            buttonBitChange(BUTTON_9_BUTTON_BIT, true);             // Button 9 pressed
            if (BUTTON_DEBUG ==1) Serial.println("Button 9");
        }
        else {                                                     // No button pressed
            buttonBitChange(BUTTON_9_BUTTON_BIT, false);
            buttonBitChange(BUTTON_10_BUTTON_BIT, false);
            buttonBitChange(BUTTON_11_BUTTON_BIT, false);
            buttonBitChange(BUTTON_12_BUTTON_BIT, false);
        }
        #endif
  
        /////////////////////////////////////////////
        // Group 4 - read analog value and determine which button is pressed according to the analog value 
        #ifdef HAS_BUTTON_GROUP_4
        val = read_CD74HC4067(BUT_4_PIN);
        if      (val < BUT_THRESHOLD_1) {
            buttonBitChange(BUTTON_16_BUTTON_BIT, true);          // Button 16 pressed 
            if (BUTTON_DEBUG ==1) Serial.println("Button 16"); 
        }
        else if (val < BUT_THRESHOLD_2) {
            buttonBitChange(BUTTON_15_BUTTON_BIT, true);          // Button 15 pressed
            if (BUTTON_DEBUG ==1) Serial.println("Button 15");
        }
        else if (val < BUT_THRESHOLD_3) {
            buttonBitChange(BUTTON_14_BUTTON_BIT, true);          // Button 14 pressed
            if (BUTTON_DEBUG ==1) Serial.println("Button 14");
        }
        else if (val < BUT_THRESHOLD_4) {
            buttonBitChange(BUTTON_13_BUTTON_BIT, true);          // Button 13 pressed
            if (BUTTON_DEBUG ==1) Serial.println("Button 13");
        }
        else {                                                    // No button pressed
            buttonBitChange(BUTTON_13_BUTTON_BIT, false);
            buttonBitChange(BUTTON_14_BUTTON_BIT, false);
            buttonBitChange(BUTTON_15_BUTTON_BIT, false);
            buttonBitChange(BUTTON_16_BUTTON_BIT, false);
        }
        #endif
      
    /////////////////////////////////////////////
    // handle rotary switches
        #ifdef HAS_RS_1
        val = read_CD74HC4067(RS_1_PIN);
        val_eval = val%INTERVAL_RS_1;

        // Check that the measured value is clote to a rotary switch step, if not, it si ignored.
        if (((val_eval >=0) && (val_eval <= INTERVAL_RS_1/2 )) || ((val_eval >= (2*INTERVAL_RS_1 /3)) && (val_eval <= INTERVAL_RS_1))) {
           /* Serial.print(INTERVAL_RS_1);
            Serial.print(" / ");
            Serial.print(val);
            Serial.print(" / ");
            Serial.print(val_eval);
             Serial.print(" / ");*/
            // measured value is shifted of half a rotary switch step to make euclidian division :
            rs_val[0] = (val + INTERVAL_RS_1/2)/INTERVAL_RS_1;
            //if (BUTTON_DEBUG ==1) Serial.println(rs_val[0]);
        }
        // Once position is measured, now check which way rotation is achieved and trigger button press according to configuration file. 
        // button press is handled by a buffer
        if (old_rs_val[0] != rs_val[0]) {   // Rotary switch did move, absolute position is adapted
              if ((rs_val[0]== (RS_1_STEPS_NUMBER-1)) && (old_rs_val[0] == 0)) {    // full rotation from 0 to max value
                  rs_pos[0]++;
                  if (BUTTON_DEBUG ==1) Serial.println("RS_1++");
                  buttonBits_buffer[RS_1_PLUS_BUTTON_BIT]++;   // add action on the buffer
              }
              else if ((rs_val[0]== 0) && (old_rs_val[0] == (RS_1_STEPS_NUMBER-1))) {   // full rotation from max to 0 value
                  rs_pos[0]--;
                  if (BUTTON_DEBUG ==1) Serial.println("RS_1--");
                  buttonBits_buffer[RS_1_LESS_BUTTON_BIT]++;   // add action on the buffer
              }
              else {
                  rs_pos[0] = rs_pos[0] - (rs_val[0] - old_rs_val[0]);
                  if (rs_val[0] > old_rs_val[0]) {
                      rs_pos[0]++;
                      if (BUTTON_DEBUG ==1) Serial.println("RS_1++");
                      buttonBits_buffer[RS_1_PLUS_BUTTON_BIT]++;   // add action on the buffer
                  }
                  else {
                      rs_pos[0]--;
                      if (BUTTON_DEBUG ==1) Serial.println("RS_1--");
                      buttonBits_buffer[RS_1_LESS_BUTTON_BIT]++;   // add action on the buffer
                  }
              }
              old_rs_val[0] = rs_val[0];
        }
        //Serial.println(rs_val[0]);
        #endif

        /////////////////////////////////////////////
        // Rotary switch 2
        #ifdef HAS_RS_2
        val = read_CD74HC4067(RS_2_PIN);
        val_eval2 = val%INTERVAL_RS_2;

        // Check that the measured value is clote to a rotary switch step, if not, it si ignored.
        if (((val_eval2 >=0) && (val_eval2 <= INTERVAL_RS_2/2 )) || ((val_eval2 >= (2*INTERVAL_RS_2/3)) && (val_eval2 <= INTERVAL_RS_2))) {
            rs_val[1] = (val + INTERVAL_RS_2/2)/INTERVAL_RS_2;          // measured value is shifted of half a rotary switch step to make euclidian division :
            //if (BUTTON_DEBUG ==1) Serial.println(rs_val[0]);
        }
        // Once position is measured, now check which way rotation is achieved and trigger button press according to configuration file. 
        // button press is handled by a buffer
        if (old_rs_val[1] != rs_val[1]) {   // Rotary switch did move, absolute position is adapted
              if ((rs_val[1]== (RS_2_STEPS_NUMBER-1)) && (old_rs_val[1] == 0)) {    // full rotation from 0 to max value
                  rs_pos[1]++;
                  if (BUTTON_DEBUG ==1) Serial.println("RS_2++");
                  buttonBits_buffer[RS_2_PLUS_BUTTON_BIT]++;   // add action on the buffer
              }
              else if ((rs_val[1]== 0) && (old_rs_val[1] == (RS_2_STEPS_NUMBER-1))) {   // full rotation from max to 0 value
                  rs_pos[1]--;
                  if (BUTTON_DEBUG ==1) Serial.println("RS_2--");
                  buttonBits_buffer[RS_2_LESS_BUTTON_BIT]++;   // add action on the buffer
              }
              else {
                  rs_pos[1] = rs_pos[1] - (rs_val[1] - old_rs_val[1]);
                  if (rs_val[1] > old_rs_val[1]) {
                      rs_pos[1]++;
                      if (BUTTON_DEBUG ==1) Serial.println("RS_2++");
                      buttonBits_buffer[RS_2_PLUS_BUTTON_BIT]++;   // add action on the buffer
                  }
                  else {
                      rs_pos[1]--;
                      if (BUTTON_DEBUG ==1) Serial.println("RS_2--");
                      buttonBits_buffer[RS_2_LESS_BUTTON_BIT]++;   // add action on the buffer
                  }
              }
              old_rs_val[1] = rs_val[1];
        }
        #endif

        /////////////////////////////////////////////
        // Rotary switch 3
        #ifdef HAS_RS_3
        val = read_CD74HC4067(RS_3_PIN);
        val_eval3 = val%INTERVAL_RS_3;

        // Check that the measured value is clote to a rotary switch step, if not, it si ignored.
        if (((val_eval3 >=0) && (val_eval3 <= INTERVAL_RS_3/2 )) || ((val_eval3 >= (2*INTERVAL_RS_3/3)) && (val_eval3 <= INTERVAL_RS_3))) {
            rs_val[2] = (val + INTERVAL_RS_3/2)/INTERVAL_RS_3;   // measured value is shifted of half a rotary switch step to make euclidian division :
        }
        // Once position is measured, now check which way rotation is achieved and trigger button press according to configuration file. 
        // button press is handled by a buffer
        if (old_rs_val[2] != rs_val[2]) {   // Rotary switch did move, absolute position is adapted
              if ((rs_val[2]== (RS_3_STEPS_NUMBER-1)) && (old_rs_val[2] == 0)) {    // full rotation from 0 to max value
                  rs_pos[2]++;
                  if (BUTTON_DEBUG ==1) Serial.println("RS_3++");
                  buttonBits_buffer[RS_3_PLUS_BUTTON_BIT]++;   // add action on the buffer
              }
              else if ((rs_val[2]== 0) && (old_rs_val[2] == (RS_3_STEPS_NUMBER-1))) {   // full rotation from max to 0 value
                  rs_pos[2]--;
                  if (BUTTON_DEBUG ==1) Serial.println("RS_3--");
                  buttonBits_buffer[RS_3_LESS_BUTTON_BIT]++;   // add action on the buffer
              }
              else {
                  rs_pos[2] = rs_pos[2] - (rs_val[2] - old_rs_val[2]);
                  if (rs_val[2] > old_rs_val[2]) {
                      rs_pos[2]++;
                      if (BUTTON_DEBUG ==1) Serial.println("RS_3++");
                      buttonBits_buffer[RS_3_PLUS_BUTTON_BIT]++;   // add action on the buffer
                  }
                  else {
                      rs_pos[2]--;
                      if (BUTTON_DEBUG ==1) Serial.println("RS_3--");
                      buttonBits_buffer[RS_3_LESS_BUTTON_BIT]++;   // add action on the buffer
                  }
              }
              old_rs_val[2] = rs_val[2];
        }
        #endif

        /////////////////////////////////////////////
        // Rotary switch 4
        #ifdef HAS_RS_4
        val = read_CD74HC4067(RS_4_PIN);
        val_eval4 = val%INTERVAL_RS_4;

        // Check that the measured value is clote to a rotary switch step, if not, it si ignored.
        if (((val_eval4 >=0) && (val_eval4 <= INTERVAL_RS_4/2 )) || ((val_eval4 >= (2*INTERVAL_RS_4/3)) && (val_eval4 <= INTERVAL_RS_4))) {
            rs_val[3] = (val + INTERVAL_RS_4/2)/INTERVAL_RS_4;   // measured value is shifted of half a rotary switch step to make euclidian division :
        }
        // Once position is measured, now check which way rotation is achieved and trigger button press according to configuration file. 
        // button press is handled by a buffer
        if (old_rs_val[3] != rs_val[3]) {   // Rotary switch did move, absolute position is adapted
              if ((rs_val[3]== (RS_4_STEPS_NUMBER-1)) && (old_rs_val[3] == 0)) {    // full rotation from 0 to max value
                  rs_pos[3]++;
                  if (BUTTON_DEBUG ==1) Serial.println("RS_4++");
                  buttonBits_buffer[RS_4_PLUS_BUTTON_BIT]++;   // add action on the buffer
              }
              else if ((rs_val[3]== 0) && (old_rs_val[3] == (RS_4_STEPS_NUMBER-1))) {   // full rotation from max to 0 value
                  rs_pos[3]--;
                  if (BUTTON_DEBUG ==1) Serial.println("RS_4--");
                  buttonBits_buffer[RS_4_LESS_BUTTON_BIT]++;   // add action on the buffer
              }
              else {
                  rs_pos[3] = rs_pos[3] - (rs_val[3] - old_rs_val[3]);
                  if (rs_val[3] > old_rs_val[3]) {
                      rs_pos[3]++;
                      if (BUTTON_DEBUG ==1) Serial.println("RS_4++");
                      buttonBits_buffer[RS_4_PLUS_BUTTON_BIT]++;   // add action on the buffer
                  }
                  else {
                      rs_pos[3]--;
                      if (BUTTON_DEBUG ==1) Serial.println("RS_4--");
                      buttonBits_buffer[RS_4_LESS_BUTTON_BIT]++;   // add action on the buffer
                  }
              }
              old_rs_val[3] = rs_val[3];
        }
        #endif

    /////////////////////////////////////////////
    // handle Advance Paddle modules
    
        /////////////////////////////////////////////
        // APM Left - read analog values and determine which button is pressed according to the analog value 
        #ifdef HAS_APM_L
        data_out.axisX = map(read_CD74HC4067(APM_L_ANALOG_PIN), 0, 1023, 0, 255);
        
        val = read_CD74HC4067(APM_L_BUT_PIN);
        if      (val < APM_THRESHOLD_1) {
            buttonBitChange(APM_L_PD_BUTTON_BIT, true);             // Button Paddle up pressed  
            if (BUTTON_DEBUG ==1) Serial.println("Paddle Left Up");
        }
        else if (val < APM_THRESHOLD_2) {
            buttonBitChange(APM_L_PU_BUTTON_BIT, true);             // Button Paddle Down pressed
            if (BUTTON_DEBUG ==1) Serial.println("Paddle Left Down");
        }
        else {                                                      // No button pressed
            buttonBitChange(APM_L_PU_BUTTON_BIT, false);
            buttonBitChange(APM_L_PD_BUTTON_BIT, false);
        }
        #endif

        /////////////////////////////////////////////
        // APM right - read analog values and determine which button is pressed according to the analog value 
        #ifdef HAS_APM_R
        data_out.axisY = map(read_CD74HC4067(APM_R_ANALOG_PIN), 0, 1023, 0, 255);
        
        val = read_CD74HC4067(APM_R_BUT_PIN);
        if      (val < APM_THRESHOLD_1)  {
            buttonBitChange(APM_R_PD_BUTTON_BIT, true);             // Button Paddle up pressed  
            if (BUTTON_DEBUG ==1) Serial.println("Paddle Right Up");
        }
        else if (val < APM_THRESHOLD_2)  {
            buttonBitChange(APM_R_PU_BUTTON_BIT, true);             // Button Paddle Down pressed
            if (BUTTON_DEBUG ==1) Serial.println("Paddle Right Down");
        }
        else {                                                      // No button pressed
            buttonBitChange(APM_R_PU_BUTTON_BIT, false);
            buttonBitChange(APM_R_PD_BUTTON_BIT, false);
        }
        #endif
        
    /////////////////////////////////////////////
    // handle encoders
        #ifdef HAS_ENC_1
        if(digitalRead(ENC_1_A_PIN) != A_1_set ) {                    // debounce once more
            A_1_set = !A_1_set;
            if ( A_1_set && !B_1_set ) {                              // adjust counter +1 if A leads B
                    encoderPos_1 += 1;
                    buttonBits_buffer[ENC_1_PLUS_BUTTON_BIT]++;       // Increase buffer value
                    if (BUTTON_DEBUG ==1) {Serial.print("Encoder 1 ++ : "); Serial.println(encoderPos_1); }
            }
        }
        if(digitalRead(ENC_1_B_PIN) != B_1_set ) {
            B_1_set = !B_1_set;
            if( B_1_set && !A_1_set ) {                               //  adjust counter - 1 if B leads A
                    encoderPos_1 -= 1;  
                    buttonBits_buffer[ENC_1_LESS_BUTTON_BIT]++;       // Increase buffer value
                    if (BUTTON_DEBUG ==1) {Serial.print("Encoder 1 -- : "); Serial.println(encoderPos_1); }
            }
        }
        #endif
  
        #ifdef HAS_ENC_2
        if(digitalRead(ENC_2_A_PIN) != A_2_set ) {                    // debounce once more
            A_2_set = !A_2_set;
            if ( A_2_set && !B_2_set ) {                              // adjust counter +1 if A leads B
                    encoderPos_2 += 1;
                    buttonBits_buffer[ENC_2_PLUS_BUTTON_BIT]++;       // Increase buffer value
                    if (BUTTON_DEBUG ==1) {Serial.print("Encoder 2 ++ : ");  Serial.println(encoderPos_2); }
            }
        }
        if(digitalRead(ENC_2_B_PIN) != B_2_set ) {
            B_2_set = !B_2_set;
            if( B_2_set && !A_2_set ) {                               //  adjust counter - 1 if B leads A
                    encoderPos_2 -= 1;
                    buttonBits_buffer[ENC_2_LESS_BUTTON_BIT]++;       // Increase buffer value
                    if (BUTTON_DEBUG ==1) {Serial.print("Encoder 2 -- : ");  Serial.println(encoderPos_2); }
            }
        }
        #endif
        
    ////////////////////////////////////////////////////////
    // handle D-PAD - read analog value and determine which button is pressed according to the analog value 
        #ifdef HAS_DPAD
        val = read_CD74HC4067(DPAD_PIN);
        if      (val < DPAD_THRESHOLD_1) { 
            buttonBitChange(DPAD_R_BUTTON_BIT, true);               // RIGHT pressed  
            if (BUTTON_DEBUG ==1) Serial.println("DPAD Right");
        }
        else if (val < DPAD_THRESHOLD_2) { 
            buttonBitChange(DPAD_D_BUTTON_BIT, true);               // DOWN pressed
            if (BUTTON_DEBUG ==1) Serial.println("DPAD Down");
        }
        else if (val < DPAD_THRESHOLD_3) { 
            buttonBitChange(DPAD_L_BUTTON_BIT, true);               // LEFT pressed
            if (BUTTON_DEBUG ==1) Serial.println("DPAD Left");
        }
        else if (val < DPAD_THRESHOLD_4) { 
            buttonBitChange(DPAD_C_BUTTON_BIT, true);               // CENTER pressed
            if (BUTTON_DEBUG ==1) Serial.println("DPAD Center");
        }
        else if (val < DPAD_THRESHOLD_5) {
            buttonBitChange(DPAD_U_BUTTON_BIT, true);               // UP pressed
            if (BUTTON_DEBUG ==1) Serial.println("DPAD Up");
        }
        else {                                                      // No button pressed
            buttonBitChange(DPAD_L_BUTTON_BIT, false);
            buttonBitChange(DPAD_R_BUTTON_BIT, false);
            buttonBitChange(DPAD_U_BUTTON_BIT, false);
            buttonBitChange(DPAD_D_BUTTON_BIT, false);
            buttonBitChange(DPAD_C_BUTTON_BIT, false);
        }
        #endif

    ////////////////////////////////////////////////////////
    // handle Joystick
        #ifdef HAS_JOY
        data_out.axisX = map(read_CD74HC4067(JOY_X_PIN), 0, 1023, 0, 255);          // Handles axis X of Joystick
        data_out.axisY = map(read_CD74HC4067(JOY_Y_PIN), 0, 1023, 0, 255);          // Handles axis Y of Joystick
        if (read_CD74HC4067(JOY_BUT_PIN) < 500) {                                   // Handles Joystick button
            buttonBitChange(JOY_BUTTON_BUTTON_BIT, true);
            if (BUTTON_DEBUG ==1) Serial.println("JOY Button");
        }
        else buttonBitChange(JOY_BUTTON_BUTTON_BIT, false);
        #endif
}

////////////////////////////////////////////////////////
// handling of button presses buffer, mostly required by impulsion based interfaces like encoders & rotary switches
void buttonBit_buffer_handling() {       
    
    if ((flag_buffer == HIGH) && ((millis()-buffer_timer) > 20)) {                  // If action is previously triggerred & timer elapsed, reset it
        buttonBitChange(buffer_index, false);
        if(buttonBits_buffer[buffer_index] > 0) buttonBits_buffer[buffer_index]--;  // mark button press as done and decrement the buffer
        buffer_index = -1;
        flag_buffer = LOW;                                                          // records that button is released
    }
    
    for (int i=0; i<24; i++) {
        if ((buttonBits_buffer[i] > 0) && (flag_buffer == LOW)) {                   // If buffer is not empty, trigger a button press
            buffer_index = i;
            flag_buffer = HIGH;                                                     // records that button is pressed 
            buffer_timer = millis();
            buttonBitChange(i, true);
            break;
        }
    }
}
