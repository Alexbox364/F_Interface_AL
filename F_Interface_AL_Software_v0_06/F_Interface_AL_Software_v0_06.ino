// This code is inherited from the Simhub original Arduino code but modified to 
// take digital input from MCP23017 GPIO expander

// Reads a button matrix attached to a MCP23XXX pin and rotary encoders. 
/* Changelog :

v0.05 :
- added support of v0.05 hardware version

v0.06 :
- added compatibility for all variants

*/
// ok to include only the one needed
// both included here to make things simple for example
#include <Adafruit_MCP23X17.h>
#include <math.h>
#include "pins_arduino.h" 
#include "F_interface_AL.h" 
#include "pins.h" 


#define AVR_BOARD

#define PIN_TYPE_A

#ifdef AVR_BOARD
  #include <avr/pgmspace.h>
#else
  #include <pgmspace.h>
#endif

#include <Adafruit_NeoPixel.h>

#if defined(BUTTON_BOX_PRO_MICRO) || defined(FULL_MICRO) || defined(MICRO_WITH_HEADER) 
  #include <Joystick.h>
#endif  

//////////////////////////////////////////////////////////////////////////////
// CONFIG - Choose your correct configuration file or create your own       //
#include "config_wheel.h" 
//#include "config_wheel_992.h" 
//////////////////////////////////////////////////////////////////////////////

#define dataLength 33
#define PRINTBIN(Num) for (uint32_t t = (1UL << (sizeof(Num) * 8) - 1); t; t >>= 1) //Serial.write(Num& t ? '1' : '0'); // Prints a binary number with leading zeros (Automatic Handling)

void yield_F();                         // function that run time essential sub-fonctions to avoid code blocking by long-time executing other ones. For example intterupt routines have to be run frequently enough
void ISR_MCP();                       // Handles interrupt of the MCP23017 for rotary encoders
void handle_ISR();                    // Handles the interrupt ISR results asynchroneously in the main loop
void set_MCP_listenning_BMX();        // Configure MCP inputs and outputs to detect a button press and trigger interrupt
void set_MCP_scan_cols_BMX(int int_pin);  // Configure MCP inputs and outputs to check a particular row
 void readButtonMatrix();              // reads the button matrix status to check if button is pressed
void inputs_management();             // handles button presses, rotary switches,....
void buttonMatrixStatusChanged(int buttonId, byte Status); // Call back function when a button status has changed in the button matrix
void printBin2(byte aByte);   // Prints binary with leading zeroes
void serialOutput();                      // Prints I/O state to Serial Output
void printmosibuf();                      // Prints MOSI buffer to Serial Output
void printmisobuf();                      // Prints MISO buffer to Serial Output
void printHex(int num, int precision);
void printButtonByteToSerial();           // Prints the selected button byte to Serial, if it was changed. The button byte can be selected by sending either A,B or C through the serial monitor. The bits in the selected button byte can either be changed by the serial monitor (sending 1 to 7 raises / lowers a specific bit,) or by pressing one of the physical buttons, if you have any attached.
void cableselect();                       // When CS line goes high - the rim should get ready to transmit the next returnData buffer, right from the begining. (so that the first byte (0xA5) will be sent out on the first Clock cycle (and after CS line went Low)
void calcOutgoingCrc();
bool checkIncomingCrc();
//ISR(SPI_STC_vect);           // SPI interrupt routine
uint8_t crc8(const uint8_t* buf, uint8_t length);     // return CRC8 from buf

void update_rev_LEDS();               // Get data received from the base about the 9 rev LEDS, parse them and displays them.
void update_7_segments();             // Get data received from the base about the 3x 7 segments displays, parse them and displays them.
void update_OLED();                   // Get data received from the base about the display information, parse, process them and displays them on the OLED
void play_init_rev_LEDs();            // Plays initialization animation

void test_output();   // test inputs of the wheel
void test_leds();     // test LED output of the wheel
void test_7segments();    // test 7 segments display

void buttonBitChange(uint8_t buttonBit, bool bitOn);    // Changes the value of a given buttonBit
void buttonBit_buffer_handling();     // handling of button presses buffer, mostly required by impulsion based interfaces like encoders & rotary switches
unsigned long lastButtonPress;
unsigned long lastPoll;

bool crc8Stat;

Adafruit_MCP23X17 mcp;

uint8_t encoder_plus = 0b00000001;
uint8_t encoder_less = 0b11111111;

fanatec_data_in_t data_in;
fanatec_data_out_t data_out;

/////////////////////////////////////////////////////
// Current values variables
    
    bool flag_ISR = 0;          // Boolean to trigger the action of MCP23017 interrupt

    // Rotary switches management
    int rs_val[4];                                  // Rotary switch current position, ranging from 0 to 7
    int old_rs_val[4] = {1023, 1023, 1023, 1023};   // Rotary switch previous position, ranging from 0 to 7
    int old_FUNKY_val;  // Rotary switch previous position, ranging from 0 to 7
    int old_DPAD_val;   // Rotary switch previous position, ranging from 0 to 7
    int rs_pos [4] = {0, 0, 0, 0};                  // integrated position of rotary switches
    int current_analog_RS_val[4];                   // Current analog Raw position
    int val_eval, val_eval2, val_eval3, val_eval4;  // Processed value to determine if rotary switch analog value is usable
    unsigned int last_RS_measure;
    int oldValRS1 = 0;
    int oldValRS2 = 0;
    int oldValRS3 = 0;
    int oldValRS4 = 0;
    
    unsigned long tempo = 0;          // timer for 1s in main loop
    // Button matrix variables
    byte BMATRIX_COLSDEF[8] = { BMATRIX_COL1, BMATRIX_COL2, BMATRIX_COL3};
    byte BMATRIX_ROWSDEF[8] = { BMATRIX_ROW1, BMATRIX_ROW2, BMATRIX_ROW3, BMATRIX_ROW4, BMATRIX_ROW5, BMATRIX_ROW6, BMATRIX_ROW7};
    uint8_t buttonState;
    unsigned long buttonLastStateChanged;
    unsigned long lastButtonPress_BMX;      // to record the instant when the button was pressed
    int lastButtonPress_BMX_ROW = 0;        // to record the row of the button pressed
    int lastButtonPress_BMX_COL = 0;        // to record the column of the button pressed
    bool FLAG_buttonPressed = 0;            // to flag that a button has been pressed in the button matrix and wait for it to be released
    bool FLAG_buttonPressedConfirmed = 0;   // to flag that a button has been CONFIRMED pressed in the button matrix and wait for it to be released
    int pressedButton;   // a text string  like 11, 12, 13.....17, 21, 22,...., 27, 31, 32 - coded like  column*10 + row
    int pressedButtonID;  //ranging from 0 to 20
    int lastPressedButton;
    int lastPressedButtonID;
    int pressedButton_ISR;  // for a buick detection by ISR handling  routine - a text string  like 11, 12, 13.....17, 21, 22,...., 27, 31, 32 - coded like  column*10 + row
    int pressedButtonID_ISR;  // the button number ranging from 0 to 20
    int flag_button_pressed = -1;     // to tell that a button was pressed
    int flag_button_pressed_ID = -1;     // to tell that a button was pressed - record the ID
    int flag_button_pressed_col = -1;     // to tell that a button was pressed - remind the colums
    int flag_button_pressed_row = -1;     // to tell that a button was pressed - remind the row

    // To record the 2 last pressed buttons simultaneously
    /*int last_pressed_col_1 = -1;
    int last_pressed_row_1 = -1;
    int last_pressed_col_2 = -1;
    int last_pressed_row_2 = -1;*/

    // Encoders management variables
     bool A_1_set = false;   
     bool B_1_set = false;
     bool A_2_set = false;   
     bool B_2_set = false;
     bool A_3_set = false;   
     bool B_3_set = false;
    
    int encoderPos_1 = 0;
    int encoderPos_2 = 0;
    int encoderPos_3 = 0;
    int old_encoderPos_1 = 0;
    int old_encoderPos_2 = 0;
    int old_encoderPos_3 = 0;

    // Buffer management
    unsigned long buffer_timer;
    int buffer_index = -1;
    bool flag_buffer = LOW;
    int buttonBits_buffer[25] =      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};       // Button press buffer handling
    bool button_input_buffer[25]=    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};      // Status of button press coming from input_management function

    // 7 segments
    uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };
    int n = 0;
    unsigned long timeStamp1;

    unsigned long lastPrintMillis = 0;
    int selectedButtonByte = 2;   // button bytes are 3rd to 5th. initialize to 1st relevant byte.
    uint8_t incByte, prevPrintedByte, prevAlphaDisp[3];
    uint8_t isrIndex = 0;
    unsigned long delayMillis = 400; // wait at least the delay time since last spi communication before printing out whatever came in.

#ifdef HAS_TM1637
    TM1637Display display(TM1637_CLK_PIN, TM1637_DIO_PIN);
#endif

#ifdef HAS_NPX
    Adafruit_NeoPixel strip(NPX_NUMBER, NPX_PIN, NEO_GRB + NEO_KHZ800);
#endif

#ifdef HAS_OLED
    Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif


// Configure the Joystick for relevant platforms
#if defined(MICRO_WITH_HEADER) 
  // create joystick object
  Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD,
  15, 0,                  // Button Count, Hat Switch Count
  false, false, false,     // X and Y, but no Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  false, false, false);  // No accelerator, brake, or steering
#endif

#if defined(FULL_MICRO) 
  // create joystick object
  Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD,
  32, 0,                  // Button Count, Hat Switch Count
  true, true, false,     // X and Y, but no Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  false, false, false);  // No accelerator, brake, or steering
#endif

#if defined(BUTTON_BOX_PRO_MICRO) 
  // create joystick object
  Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD,
  32, 0,                  // Button Count, Hat Switch Count
  true, true, false,     // X and Y, but no Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  false, false, false);  // No accelerator, brake, or steerin
#endif
  
/////////////////////////////////////////////////////
// SETUP
void setup() {
  
  pinMode(MISO_PIN, OUTPUT);    // usually when using SPI, arduino will automatically define MISO as output, but not when doing SPI slave.
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(MOSI_PIN, INPUT_PULLUP);
  pinMode(INT_CS_SPI, INPUT_PULLUP);   // connecting CS_ISR pin (D2) to the SS pin (D10) is crucial. See the schematics.
  pinMode(CS_PIN, INPUT_PULLUP);     
  

  // Wheel configuration
  data_out.header = 0xa5;
  data_out.id = 0x0C;

  #ifdef AVR_BOARD
  // SPI initerrupts configuration for the arduino based boards
  attachInterrupt(digitalPinToInterrupt(INT_CS_SPI), cableselect, RISING);
  // SPCR BYTE should be: 11000100   note to self: by raw_capture.ino and fanatec.cpp spi settings, of btClubSportWheel by Darknao, SPI settings are SPI_Mode0 & MSBFIRST. but with logic scope I see that CPHA 1 (falling!) is used by wheel base, which means SPI_MODE1. (and MSBFIRST)
  // (Mode 1 - clock is normally low (CPOL = 0), and the data is sampled on the transition from high to low (trailing edge) (CPHA = 1))
  SPCR |= _BV(SPE);   // turn on SPI
  SPCR |= _BV(SPIE);    // turn on interrupts
  SPCR |= _BV(CPHA);    //turns on CPHA. as I found the CSW wheelbase to use it, with logic scope.
  #endif
  
  Serial.begin(230400);
  Serial.print("F_interface_AL - ");
  Serial.println(VERSION);
  Serial.println("Starting engine...");

  #if defined(FULL_NANO) || defined(FULL_NANO_ESP32) || defined(FULL_MICRO) || defined(BUTTON_BOX_PRO_MICRO)  // Define this for boards having an MCP23017
  // Initialize MCP23017 communication
  mcp.begin_I2C();
  mcp.setupInterrupts(true,false,LOW);
        // OPTIONAL - call this to override defaults
          // mirror INTA/B so only one wire required      @param mirroring true to OR both INTA and INTB pins.
          // active drive so INTA/B will not be floating  @param openDrain true for open drain output, false for active drive output.
          // INTA/B will be signaled with a LOW           @param polarity HIGH or LOW
  

  // configure pin for encoder inputs with pull up
  mcp.pinMode(ENC_1_A_PIN, INPUT_PULLUP);
  mcp.pinMode(ENC_1_B_PIN, INPUT_PULLUP);
  mcp.pinMode(ENC_2_A_PIN, INPUT_PULLUP);
  mcp.pinMode(ENC_2_B_PIN, INPUT_PULLUP);
  mcp.pinMode(ENC_3_A_PIN, INPUT_PULLUP);
  mcp.pinMode(ENC_3_B_PIN, INPUT_PULLUP);
  
  // enable interrupt on each required pins
  mcp.setupInterruptPin(ENC_1_A_PIN, CHANGE);
  mcp.setupInterruptPin(ENC_1_B_PIN, CHANGE);
  mcp.setupInterruptPin(ENC_2_A_PIN, CHANGE);
  mcp.setupInterruptPin(ENC_2_B_PIN, CHANGE);
  mcp.setupInterruptPin(ENC_3_A_PIN, CHANGE);
  mcp.setupInterruptPin(ENC_3_B_PIN, CHANGE);

  mcp.setupInterruptPin(BMATRIX_ROW1, CHANGE);
  mcp.setupInterruptPin(BMATRIX_ROW2, CHANGE);
  mcp.setupInterruptPin(BMATRIX_ROW3, CHANGE);
  mcp.setupInterruptPin(BMATRIX_ROW4, CHANGE);
  mcp.setupInterruptPin(BMATRIX_ROW5, CHANGE);
  mcp.setupInterruptPin(BMATRIX_ROW6, CHANGE);
  mcp.setupInterruptPin(BMATRIX_ROW7, CHANGE);

  pinMode(INTERRUPT_PIN, INPUT);      // No need of the pullup as the MCP23017 will have an active drain output.
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), ISR_MCP, CHANGE);
  #endif

  #if defined(MICRO_WITH_HEADER) || defined(ATMEGA328P_WITH_HEADER) || defined(NANO_WITH_HEADER)
    pinMode(BUT_1_PIN, INPUT_PULLUP);
    pinMode(BUT_2_PIN, INPUT_PULLUP);
    pinMode(BUT_3_PIN, INPUT_PULLUP);
    pinMode(BUT_4_PIN, INPUT_PULLUP);
    pinMode(BUT_5_PIN, INPUT_PULLUP);
    pinMode(BUT_6_PIN, INPUT_PULLUP);
    pinMode(BUT_7_PIN, INPUT_PULLUP);
    pinMode(BUT_8_PIN, INPUT_PULLUP);
    pinMode(BUT_9_PIN, INPUT_PULLUP);
    pinMode(BUT_10_PIN, INPUT_PULLUP);
    pinMode(BUT_11_PIN, INPUT_PULLUP);
    pinMode(BUT_12_PIN, INPUT_PULLUP);
    pinMode(BUT_13_PIN, INPUT_PULLUP);
    pinMode(BUT_14_PIN, INPUT_PULLUP);
    pinMode(BUT_15_PIN, INPUT_PULLUP);
  #endif  

  #if defined(BUTTON_BOX_PRO_MICRO)
    pinMode(FUNKY_PUSH, INPUT_PULLUP);
    pinMode(BUT_1_PIN, INPUT_PULLUP);
    pinMode(BUT_2_PIN, INPUT_PULLUP);
    pinMode(BUT_3_PIN, INPUT_PULLUP);
    pinMode(BUT_4_PIN, INPUT_PULLUP);
    pinMode(BUT_5_PIN, INPUT_PULLUP);
    pinMode(BUT_6_PIN, INPUT_PULLUP);
  #endif

  #ifdef  HAS_BUTTON_MATRIX
      
      set_MCP_listenning_BMX();     // Configure MCP inputs and outputs to detect a button press and trigger interrupt

      // Setup button matrix
      /*for (int x = 0; x < BMATRIX_ROWS; x++) {
          mcp.pinMode(BMATRIX_ROWSDEF[x], INPUT);
      }
      for (int x = 0; x < BMATRIX_COLS; x++) {
          mcp.pinMode(BMATRIX_COLSDEF[x], INPUT_PULLUP);
      }*/
  #endif
  
  #ifdef HAS_NPX
    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.show();            // Turn OFF all pixels ASAP
    strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

    play_init_rev_LEDs();   // Plays initialization animation
  #endif 

  #ifdef HAS_OLED
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display2.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
    display2.display();
    display2.setTextSize(1);              // Normal 1:1 pixel scale
    display2.setTextColor(SSD1306_WHITE); // Draw white text
      
  #endif

  // Configure the Joystick for relevant platforms
  #if defined(MICRO_WITH_HEADER) 
    Joystick.begin();   
  #endif

  #if defined(FULL_MICRO) 
    Joystick.begin();
    Joystick.setXAxisRange(0, MAX_ADC);
    Joystick.setYAxisRange(0, MAX_ADC);
  #endif
  
  #if defined(BUTTON_BOX_PRO_MICRO) 
    Joystick.begin();
    Joystick.setXAxisRange(0, MAX_ADC);
    Joystick.setYAxisRange(0, MAX_ADC);
  #endif
  
  Serial.println("Starting the race !...");
}

void loop() {

  calcOutgoingCrc();
  //yield_F();
  
  crc8Stat = checkIncomingCrc();
  yield_F();
  
  // For debug purposes : 
  // printmosibuf();         // prints data sent from the base to the wheel
  // printmisobuf();         // prints data sent from the wheel to the base
  if (BUTTON_DEBUG == 1) {
    if (millis() - tempo > 200) {
      // serialOutput();           // Prints I/O state to Serial Output
      tempo = millis();  
    }
  }

  // For test purposes
  //test_output();
  //test_leds();
  //test_7segments();
  
  yield_F();             // Handles the interrupt ISR results asynchroneously in the main loop
  
  //Handles inputs and outputs for PCBS having some, skip for others
  #if defined(FULL_NANO) || defined(FULL_NANO_ESP32) || defined(FULL_MICRO) || defined(BUTTON_BOX_PRO_MICRO) || defined(MICRO_WITH_HEADER) || defined(ATMEGA328P_WITH_HEADER) || defined(NANO_WITH_HEADER)
    inputs_management();      // handles button presses, rotary switches,....
    //yield_F();
    buttonBit_buffer_handling();  

    #ifdef HAS_TM1637
        update_7_segments();
        yield_F();
    #endif

    #ifdef HAS_NPX
        update_rev_LEDS();
        yield_F();
    #endif
  #endif
  yield_F();
}

void ISR_MCP() {
  flag_ISR = 1;
}

//////////////////////////////////////////////////////////////////
// Handles the interrupt ISR results asynchroneously in the main loop
// Has to be called as often as possible, especially in long loops to ensure proper catch of encoder moves. 
void handle_ISR() {
  if (flag_ISR == 1) {     // Handle ISR results
    flag_ISR = 0;
    //Serial.println("ISR triggered");
    #ifdef HAS_BUTTON_MATRIX
      // Part of the code that detects a button press of the button matrix based on interrupts. The function readButtonMatrix() will handle the button depress with les reactivity but better reliability. 
      if (flag_button_pressed == -1) {       // If no button is already pressed
        int int_pin = mcp.getLastInterruptPin();    // gets the pressed row
        //Serial.println(int_pin);
        //if (int_pin >=0 && int_pin < 9) {         // TO ADAPT DEPENDING ON THE MCP ROW PINS USED
        if (int_pin == BMATRIX_ROW1 ||
          int_pin == BMATRIX_ROW2 ||
          int_pin == BMATRIX_ROW3 ||
          int_pin == BMATRIX_ROW4 ||
          int_pin == BMATRIX_ROW5 ||
          int_pin == BMATRIX_ROW6 ||
          int_pin == BMATRIX_ROW7 ) {         // TO CHECK IF INTERRUPT COMES FROM A ROW BUTTON MATRIX PIN OR FROM THE ENCODERS
          
          int col_index = -1;
          
          // to check if a button has been pressed (0) or released (1)
          bool status = mcp.digitalRead(int_pin);
          int ID_row = -1;
          
          switch(int_pin) {
            case BMATRIX_ROW1:
              ID_row = 0;
              break;
            case BMATRIX_ROW2:
              ID_row = 1;
              break;
            case BMATRIX_ROW3:
              ID_row = 2;
              break;
            case BMATRIX_ROW4:
              ID_row = 3;
              break;
            case BMATRIX_ROW5:  
              ID_row = 4;
              break;
            case BMATRIX_ROW6:
              ID_row = 5;
              break;
            case BMATRIX_ROW7:
              ID_row = 6;
              break;
            default:
              break;
          }
          if (ID_row != -1) {

          
            set_MCP_scan_cols_BMX(int_pin);     // Configure MCP inputs and outputs to check a particular row
            
            // Scan all the cols to check which button was pressed
            if (status == 0) {                  // If the button is beeing pressed // We don't take car of button release as it will be handled by the readButtonMatrix() function
              if (mcp.digitalRead(BMATRIX_COL1) == 0) col_index = BMATRIX_COL1;
              if (mcp.digitalRead(BMATRIX_COL2) == 0) col_index = BMATRIX_COL2;
              if (mcp.digitalRead(BMATRIX_COL3) == 0) col_index = BMATRIX_COL3;

              // identify the column ID
              int ID_col = -1;
              switch(col_index) {
                case BMATRIX_COL1:
                  ID_col = 0;
                  break;
                case BMATRIX_COL2:
                  ID_col = 1;
                  break;
                case BMATRIX_COL3:
                  ID_col = 2;
                  break;
                default:
                  break;
              }
              
              if (ID_col != -1) {
                pressedButton_ISR = (col_index+1)*10 + ID_row + 1;
                pressedButtonID_ISR = (col_index)* BMATRIX_ROWS + ID_row;
                
                flag_button_pressed = pressedButton_ISR;
                flag_button_pressed_ID = pressedButtonID_ISR;
                flag_button_pressed_col = col_index;
                flag_button_pressed_row = int_pin;
                
                if(BUTTON_DEBUG) Serial.print("Button pressed  : ");
                if(BUTTON_DEBUG) Serial.print(pressedButton_ISR);
                if(BUTTON_DEBUG) Serial.print(" ISR_ : ");
                if(BUTTON_DEBUG) Serial.print(flag_button_pressed);
                if(BUTTON_DEBUG) Serial.print(" / button ID : ");
                if(BUTTON_DEBUG) Serial.print(pressedButtonID_ISR);
                if(BUTTON_DEBUG) Serial.print(" / Fanatec buttonBit : ");
                if(BUTTON_DEBUG) Serial.println(buttonMatrix_buttonBits[pressedButtonID_ISR]);
                
                // trigger the button press to the base
                buttonMatrixStatusChanged(pressedButtonID_ISR, 1);
              }
            }
          }
          set_MCP_listenning_BMX();
        }
        mcp.clearInterrupts();  // clear   
      }
    #endif
    
    #ifdef HAS_ENC_1
      if(mcp.digitalRead(ENC_1_A_PIN) != A_1_set ) {                    // debounce once more
        A_1_set = !A_1_set;
        if ( A_1_set && !B_1_set ) {                              // adjust counter +1 if A leads B
          encoderPos_1 += 1;
        }
      }
      if(mcp.digitalRead(ENC_1_B_PIN) != B_1_set ) {
        B_1_set = !B_1_set;
        if( B_1_set && !A_1_set ) {                               //  adjust counter - 1 if B leads A
          encoderPos_1 -= 1;  
        }
      }
      if (old_encoderPos_1 != encoderPos_1) {
        if (old_encoderPos_1 > encoderPos_1) {
          if (BUTTON_DEBUG ==1) Serial.print("Encoder 1 -- : ");  
          buttonBits_buffer[ENC_1_LESS_BUTTON_BIT]++;       // Increase buffer value
        }
        else {
          if (BUTTON_DEBUG ==1) Serial.print("Encoder 1 ++ : ");  
          buttonBits_buffer[ENC_1_PLUS_BUTTON_BIT]++;       // Increase buffer value
        }
        if (BUTTON_DEBUG ==1) Serial.println(encoderPos_1);
        old_encoderPos_1 = encoderPos_1;
      }
      
    #endif
    
    #ifdef HAS_ENC_2
      if(mcp.digitalRead(ENC_2_A_PIN) != A_2_set ) {                    // debounce once more
        A_2_set = !A_2_set;
        if ( A_2_set && !B_2_set ) {                              // adjust counter +1 if A leads B
            encoderPos_2 += 1;
        }
      }
      if(mcp.digitalRead(ENC_2_B_PIN) != B_2_set ) {
        B_2_set = !B_2_set;
        if( B_2_set && !A_2_set ) {                               //  adjust counter - 1 if B leads A
            encoderPos_2 -= 1;
        }
      }
      if (old_encoderPos_2 != encoderPos_2) {
        if (old_encoderPos_2 > encoderPos_2) {
          if (BUTTON_DEBUG ==1) Serial.print("Encoder 2 -- : ");  
          buttonBits_buffer[ENC_2_LESS_BUTTON_BIT]++;       // Increase buffer value
        }
        else {
          if (BUTTON_DEBUG ==1) Serial.print("Encoder 2 ++ : "); 
          buttonBits_buffer[ENC_2_PLUS_BUTTON_BIT]++;       // Increase buffer value 
        }
        if (BUTTON_DEBUG ==1) Serial.println(encoderPos_2);
        old_encoderPos_2 = encoderPos_2;
      }
      // mcp.clearInterrupts();  // clear   
    #endif
    
    #ifdef HAS_ENC_3
      if(mcp.digitalRead(ENC_3_A_PIN) != A_3_set ) {                    // debounce once more
          A_3_set = !A_3_set;
          if ( A_3_set && !B_3_set ) {                              // adjust counter +1 if A leads B
              encoderPos_3 += 1;
          }
      }
      if(mcp.digitalRead(ENC_3_B_PIN) != B_3_set ) {
          B_3_set = !B_3_set;
          if( B_3_set && !A_3_set ) {                               //  adjust counter - 1 if B leads A
            encoderPos_3 -= 1;
          }
      }
      if (old_encoderPos_3 != encoderPos_3) {
        if (old_encoderPos_3 > encoderPos_3) {
          if (BUTTON_DEBUG ==1) Serial.print("Encoder 3 -- : ");  
          buttonBits_buffer[ENC_3_LESS_BUTTON_BIT]++;       // Increase buffer value
        }
        else {
          if (BUTTON_DEBUG ==1) Serial.print("Encoder 3 ++ : ");  
          buttonBits_buffer[ENC_3_PLUS_BUTTON_BIT]++;       // Increase buffer value
        }
        if (BUTTON_DEBUG ==1) Serial.println(encoderPos_3);
        old_encoderPos_3 = encoderPos_3;
      }
    #endif
  }
}

//////////////////////////////////////////////////////////////////
// Call back function when a button status has changed in the button matrix
void buttonMatrixStatusChanged(int buttonId, byte Status) { 
  
  button_input_buffer[buttonMatrix_buttonBits[buttonId]] = Status;
  
  //Serial.print("Fanatec buttonID presed : ");
  //Serial.println(buttonMatrix_buttonBits[buttonId]);
  
  /*#ifdef INCLUDE_GAMEPAD
        Joystick.setButton(TM1638_ENABLEDMODULES * 8 + ENABLED_BUTTONS_COUNT + buttonId - 1, Status);
        Joystick.sendState();
      #else*/
      /*arqserial.CustomPacketStart(0x03, 2);
      arqserial.CustomPacketSendByte(ENABLED_BUTTONS_COUNT + buttonId);
      arqserial.CustomPacketSendByte(Status);
      arqserial.CustomPacketEnd();*/
      //#endif
}


//////////////////////////////////////////////////////////////////
// reads the button matrix status to check if button is pressed
void readButtonMatrix() {
  if ((millis() - buttonLastStateChanged) > DEBOUNCE_DELAY && flag_button_pressed >=0) {        // We just detect the button release. Button press is detected via interrupe handling routin handle_ISR()
    pressedButton = -1;
    pressedButtonID = -1;

    bool is_empty = 1;      // to check if no button is pressed

    // check if previously pressed button is released :
    mcp.pinMode(flag_button_pressed_col, OUTPUT);
    mcp.digitalWrite(flag_button_pressed_col, LOW);
    mcp.pinMode(flag_button_pressed_row, INPUT_PULLUP);
    //if(BUTTON_DEBUG) Serial.println(mcp.digitalRead(flag_button_pressed_row));
    if (mcp.digitalRead(flag_button_pressed_row) == LOW) {
      // button is still pressed, no action
      
    }
    else {      // button is released, trigger release actions
      if(BUTTON_DEBUG) Serial.print("Button released : ");
      if(BUTTON_DEBUG) Serial.print(flag_button_pressed);
      buttonMatrixStatusChanged(flag_button_pressed_ID, 0);

      if(BUTTON_DEBUG) Serial.print(" ISR_ : ");
      if(BUTTON_DEBUG) Serial.print(flag_button_pressed);
      if(BUTTON_DEBUG) Serial.print(" / button ID : ");
      if(BUTTON_DEBUG) Serial.print(flag_button_pressed_ID);
      if(BUTTON_DEBUG) Serial.print(" / Fanatec buttonBit : ");
      if(BUTTON_DEBUG) Serial.println(buttonMatrix_buttonBits[flag_button_pressed_ID]);
      flag_button_pressed = -1;
      flag_button_pressed_ID = -1;
      flag_button_pressed_row = -1;
      flag_button_pressed_col = -1;
      is_empty = 0;      // to check if no button is pressed

      if(is_empty) {    // if no button is pressed, clear the button bit. 
        for (int j = 1; j<25; j++) {
          button_input_buffer[j] = 0;    // reset status
        }
      }
    }
		return;
	}
}

/////////////////////////////////////////////
// handles button presses, rotary switches,....
void inputs_management() {

    int val=0;

    // As button press can come from various button sources, we use a temporary buffer which is reset at each loop execution. 
    // The button bits are applied once every button status is retrieved
    // reset button_input_buffer
    
   /*for (int j = 1; j<25; j++) {
        button_input_buffer[j] = 0;    // reset status
    }*/
    
    ////////////////////////////////////////////////////////
    // handle D-PAD - read analog value and determine which button is pressed according to the analog value 
        #ifdef HAS_DPAD
        yield_F();
        val = analogRead(DPAD_PIN);
        if (abs(old_DPAD_val - val) > 20) {    // if value has changed, we process it
          if      (val < DPAD_THRESHOLD_1) { 
              button_input_buffer[DPAD_R_BUTTON_BIT] = 1;
              if (BUTTON_DEBUG ==1) Serial.println("DPAD Right");
          }
          else if (val < DPAD_THRESHOLD_2) { 
              button_input_buffer[DPAD_D_BUTTON_BIT] = 1;
              if (BUTTON_DEBUG ==1) Serial.println("DPAD Down");
          }
          else if (val < DPAD_THRESHOLD_3) { 
              button_input_buffer[DPAD_L_BUTTON_BIT] = 1;
              if (BUTTON_DEBUG ==1) Serial.println("DPAD Left");
          }
          else if (val < DPAD_THRESHOLD_4) { 
              button_input_buffer[DPAD_C_BUTTON_BIT] = 1;
              if (BUTTON_DEBUG ==1) Serial.println("DPAD Center");
          }
          else if (val < DPAD_THRESHOLD_5) {
              button_input_buffer[DPAD_U_BUTTON_BIT] = 1;
              if (BUTTON_DEBUG ==1) Serial.println("DPAD Up");
          }
          else {                                                      // No button pressed
            button_input_buffer[DPAD_C_BUTTON_BIT] = 0;
            button_input_buffer[DPAD_U_BUTTON_BIT] = 0;
            button_input_buffer[DPAD_D_BUTTON_BIT] = 0;
            button_input_buffer[DPAD_L_BUTTON_BIT] = 0;
            button_input_buffer[DPAD_R_BUTTON_BIT] = 0;
          }
          old_DPAD_val = val;
        }
        #endif

    ////////////////////////////////////////////////////////
    // handle Funky switch - read analog value and determine which button is pressed according to the analog value 
        #ifdef HAS_FUNKY
        yield_F();
        val = analogRead(FUNKY_A_PIN);
        if (abs(old_FUNKY_val - val) > 20) {    // if value has changed, we process it
          //Serial.println(val);
          if      (val < FUNKY_THRESHOLD_1) { 
              button_input_buffer[FUNKY_R_BUTTON_BIT] = 1;
              if (BUTTON_DEBUG ==1) Serial.println("FUNKY Right");   
          }
          else if (val < FUNKY_THRESHOLD_2) { 
              button_input_buffer[FUNKY_D_BUTTON_BIT] = 1;
              if (BUTTON_DEBUG ==1) Serial.println("FUNKY Down");
          }
          else if (val < FUNKY_THRESHOLD_3) { 
              button_input_buffer[FUNKY_L_BUTTON_BIT] = 1;
              if (BUTTON_DEBUG ==1) Serial.println("FUNKY Left");
          }
          else if (val < FUNKY_THRESHOLD_4) {
              button_input_buffer[FUNKY_U_BUTTON_BIT] = 1;
              if (BUTTON_DEBUG ==1) Serial.println("FUNKY Up");
          }
          else {                                                      // No button pressed
            button_input_buffer[FUNKY_U_BUTTON_BIT] = 0;
            button_input_buffer[FUNKY_D_BUTTON_BIT] = 0;
            button_input_buffer[FUNKY_L_BUTTON_BIT] = 0;
            button_input_buffer[FUNKY_R_BUTTON_BIT] = 0;
            if (BUTTON_DEBUG ==1) Serial.println("FUNKY Released");
          }
          old_FUNKY_val = val;
        }
        #if defined(BUTTON_BOX_PRO_MICRO)
          /*if ((digitalRead(FUNKY_PUSH) == LOW) && (analogRead(FUNKY_A_PIN) > FUNKY_THRESHOLD_4)){
            button_input_buffer[FUNKY_C_BUTTON_BIT] = 1;
            if (BUTTON_DEBUG ==1) Serial.println("FUNKY CENTER Pressed");
          }
          else {                                                      // No button pressed
            button_input_buffer[FUNKY_C_BUTTON_BIT] = 0;
            //if (BUTTON_DEBUG ==1) Serial.println("FUNKY CENTER Released");
          }*/
        #endif
        #endif
    
    #ifdef  HAS_BUTTON_MATRIX
      if (millis() - lastPoll > 5) {
        lastPoll = millis();
        readButtonMatrix();              // reads the button matrix status to check if button is pressed
      }
      
    #endif
      
    /////////////////////////////////////////////
    // handle rotary switches
        
        //evaluated every 50ms
        if ((millis() - last_RS_measure) > 50) {
          int ecart = 0; 
          #ifdef HAS_RS_1
          yield_F();
          val = analogRead(RS_1_PIN);
    
          // we take new measurement and check it's stable over time to debounce
          ecart = 0;
          if ((val - oldValRS1) <0) ecart = oldValRS1 - val;
          else ecart = val - oldValRS1;
          //Serial.println(ecart);

          if(int (ecart / (millis() - last_RS_measure)) < ECART_RS) {      // We determine an evolution rate of the measurement to check it's uder a thershold, meaning values are stable. 
            unsigned long tempo = millis();
            while (millis() - tempo < 50) {   // non blocking debounce for base SPI communication
            }
            val = 0;
            for (int i = 0; i<ANALOG_SAMPLES; i++) {
              val += analogRead(RS_1_PIN);
            }
            val= int(val / ANALOG_SAMPLES);

            oldValRS1 = val;
            //if (BUTTON_DEBUG ==1) Serial.print(val);
            //if (BUTTON_DEBUG ==1) Serial.print(" / ");
            
            val_eval = val%INTERVAL_RS_1;
            //if (BUTTON_DEBUG ==1) Serial.print(val_eval);
            //if (BUTTON_DEBUG ==1) Serial.print(" / ");
            
            // Check that the measured value is close to a rotary switch step, if not, it si ignored.
            if (((val_eval >=0) && (val_eval <= INTERVAL_RS_1/3 )) || ((val_eval >= (2*INTERVAL_RS_1 /3)) && (val_eval <= INTERVAL_RS_1))) {
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
                          rs_pos[0]--;
                          if (BUTTON_DEBUG ==1) Serial.println("RS_1--");
                          buttonBits_buffer[RS_1_LESS_BUTTON_BIT]++;   // add action on the buffer
                      }
                      else {
                          rs_pos[0]++;
                          if (BUTTON_DEBUG ==1) Serial.println("RS_1++");
                          buttonBits_buffer[RS_1_PLUS_BUTTON_BIT]++;   // add action on the buffer
                      }
                  }
                  old_rs_val[0] = rs_val[0];
            }
          }
          //Serial.println(rs_val[0]);
          #endif

          /////////////////////////////////////////////
          // Rotary switch 2
          #ifdef HAS_RS_2
          yield_F();
          val = analogRead(RS_2_PIN);
          
          // we take new measurement and check it's stable over time to debounce
          ecart = 0;
          if ((val - oldValRS2) <0) ecart = oldValRS2 - val;
          else ecart = val - oldValRS2;
          //Serial.println(ecart);

          if(int (ecart / (millis() - last_RS_measure)) < ECART_RS) {      // We determine an evolution rate of the measurement to check it's uder a thershold, meaning values are stable. 
            val = 0;
            for (int i = 0; i<ANALOG_SAMPLES; i++) {
              val += analogRead(RS_2_PIN);
            }
            val= int(val / ANALOG_SAMPLES);

            oldValRS2 = val;

            val_eval2 = val%INTERVAL_RS_2;
          

          
            // Check that the measured value is clote to a rotary switch step, if not, it si ignored.
            if (((val_eval2 >=0) && (val_eval2 <= INTERVAL_RS_2/3 )) || ((val_eval2 >= (2*INTERVAL_RS_2/3)) && (val_eval2 <= INTERVAL_RS_2))) {
                rs_val[1] = (val + INTERVAL_RS_2/2)/INTERVAL_RS_2;          // measured value is shifted of half a rotary switch step to make euclidian division :
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
                  rs_pos[1]--;
                  if (BUTTON_DEBUG ==1) Serial.println("RS_2--");
                  buttonBits_buffer[RS_2_LESS_BUTTON_BIT]++;   // add action on the buffer
                }
                else {
                  rs_pos[1]++;
                  if (BUTTON_DEBUG ==1) Serial.println("RS_2++");
                  buttonBits_buffer[RS_2_PLUS_BUTTON_BIT]++;   // add action on the buffer
                }
              }
              old_rs_val[1] = rs_val[1];
              }
          }
          #endif

          /////////////////////////////////////////////
          // Rotary switch 3
          #ifdef HAS_RS_3
          yield_F();
          val = analogRead(RS_3_PIN);
          // we take new measurement and check it's stable over time to debounce
          ecart = 0;
          if ((val - oldValRS3) <0) ecart = oldValRS3 - val;
          else ecart = val - oldValRS3;
          //Serial.println(ecart);

          if(int (ecart / (millis() - last_RS_measure)) < ECART_RS) {      // We determine an evolution rate of the measurement to check it's uder a thershold, meaning values are stable. 
            val = 0;
            for (int i = 0; i<ANALOG_SAMPLES; i++) {
              val += analogRead(RS_3_PIN);
            }
            val= int(val / ANALOG_SAMPLES);

            oldValRS3 = val;
            
            val_eval3 = val%INTERVAL_RS_3;
            
            // Check that the measured value is clote to a rotary switch step, if not, it si ignored.
            if (((val_eval3 >=0) && (val_eval3 <= INTERVAL_RS_3/3 )) || ((val_eval3 >= (2*INTERVAL_RS_3/3)) && (val_eval3 <= INTERVAL_RS_3))) {
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
                      rs_pos[2]--;
                      if (BUTTON_DEBUG ==1) Serial.println("RS_3--");
                      buttonBits_buffer[RS_3_LESS_BUTTON_BIT]++;   // add action on the buffer
                  }
                  else {
                      rs_pos[2]++;
                      if (BUTTON_DEBUG ==1) Serial.println("RS_3++");
                      buttonBits_buffer[RS_3_PLUS_BUTTON_BIT]++;   // add action on the buffer
                  }
              }
              old_rs_val[2] = rs_val[2];
            }
          }
          #endif

          /////////////////////////////////////////////
          // Rotary switch 4
          #ifdef HAS_RS_4
          yield_F();
          val = analogRead(RS_4_PIN);
          // we take new measurement and check it's stable over time to debounce
          ecart = 0;
          if ((val - oldValRS4) <0) ecart = oldValRS4 - val;
          else ecart = val - oldValRS4;
          //Serial.println(ecart);

          if(int (ecart / (millis() - last_RS_measure)) < ECART_RS) {      // We determine an evolution rate of the measurement to check it's uder a thershold, meaning values are stable. 
            val = 0;
            for (int i = 0; i<ANALOG_SAMPLES; i++) {
              val += analogRead(RS_4_PIN);
            }
            val= int(val / ANALOG_SAMPLES);

            oldValRS4 = val;
            //if (BUTTON_DEBUG ==1) Serial.print(val);
            //if (BUTTON_DEBUG ==1) Serial.print(" / ");
            
            val_eval4 = val%INTERVAL_RS_4;
            
            // Check that the measured value is clote to a rotary switch step, if not, it si ignored.
            if (((val_eval4 >=0) && (val_eval4 <= INTERVAL_RS_4/3 )) || ((val_eval4 >= (2*INTERVAL_RS_4/3)) && (val_eval4 <= INTERVAL_RS_4))) {
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
                    if (BUTTON_DEBUG ==1) Serial.println("RS_4--");
                    buttonBits_buffer[RS_4_LESS_BUTTON_BIT]++;   // add action on the buffer
                  }
                  else {
                    rs_pos[3]++;
                    if (BUTTON_DEBUG ==1) Serial.println("RS_4++");
                    buttonBits_buffer[RS_4_PLUS_BUTTON_BIT]++;   // add action on the buffer
                  }
                }
                old_rs_val[3] = rs_val[3];
            }
          } 
          #endif
        last_RS_measure = millis();
        }
        
    ////////////////////////////////////////////////////////
    // handle Joystick
        #ifdef HAS_ANALOG_AXIS
        yield_F();
        data_out.axisX = map(analogRead(AXIS_X_PIN), 0, 1023, 0, 255);          // Handles axis X 
        data_out.axisY = map(analogRead(AXIS_Y_PIN), 0, 1023, 0, 255);          // Handles axis Y 
        
        // Handle Joystick output for relevant platforms
        #if defined(BUTTON_BOX_PRO_MICRO) || defined(FULL_MICRO)
          Joystick.setXAxis(data_out.axisX);
          Joystick.setYAxis(data_out.axisY);
        #endif  
        
        Serial.print(data_out.axisX);
        Serial.print(" / ");
        Serial.println(data_out.axisY);
        
       #endif

    ////////////////////////////////////////////////////////
    // handle direct button presses (without button matrix)
    #if defined(MICRO_WITH_HEADER) || defined(ATMEGA328P_WITH_HEADER) || defined(NANO_WITH_HEADER)
      if (digitalRead(BUT_1_PIN) == LOW) {buttonBitChange(BUT_1_BIT, 1);} else {buttonBitChange(BUT_1_BIT, 0);} // Button 1
      if (digitalRead(BUT_2_PIN) == LOW) {buttonBitChange(BUT_2_BIT, 1);} else {buttonBitChange(BUT_2_BIT, 0);} // Button 2
      if (digitalRead(BUT_3_PIN) == LOW) {buttonBitChange(BUT_3_BIT, 1);} else {buttonBitChange(BUT_3_BIT, 0);} // Button 3
      if (digitalRead(BUT_4_PIN) == LOW) {buttonBitChange(BUT_4_BIT, 1);} else {buttonBitChange(BUT_4_BIT, 0);} // Button 4
      if (digitalRead(BUT_5_PIN) == LOW) {buttonBitChange(BUT_5_BIT, 1);} else {buttonBitChange(BUT_5_BIT, 0);} // Button 5
      if (digitalRead(BUT_6_PIN) == LOW) {buttonBitChange(BUT_6_BIT, 1);} else {buttonBitChange(BUT_6_BIT, 0);} // Button 6
      if (digitalRead(BUT_7_PIN) == LOW) {buttonBitChange(BUT_7_BIT, 1);} else {buttonBitChange(BUT_7_BIT, 0);} // Button 7
      if (digitalRead(BUT_8_PIN) == LOW) {buttonBitChange(BUT_8_BIT, 1);} else {buttonBitChange(BUT_8_BIT, 0);} // Button 8
      if (digitalRead(BUT_9_PIN) == LOW) {buttonBitChange(BUT_9_BIT, 1);} else {buttonBitChange(BUT_9_BIT, 0);} // Button 9
      if (digitalRead(BUT_10_PIN) == LOW) {buttonBitChange(BUT_10_BIT, 1);} else {buttonBitChange(BUT_10_BIT, 0);} // Button 10
      if (digitalRead(BUT_11_PIN) == LOW) {buttonBitChange(BUT_11_BIT, 1);} else {buttonBitChange(BUT_11_BIT, 0);} // Button 11
      if (digitalRead(BUT_12_PIN) == LOW) {buttonBitChange(BUT_12_BIT, 1);} else {buttonBitChange(BUT_12_BIT, 0);} // Button 12
      if (digitalRead(BUT_13_PIN) == LOW) {buttonBitChange(BUT_13_BIT, 1);} else {buttonBitChange(BUT_13_BIT, 0);} // Button 13
      if (digitalRead(BUT_14_PIN) == LOW) {buttonBitChange(BUT_14_BIT, 1);} else {buttonBitChange(BUT_14_BIT, 0);} // Button 14
      if (digitalRead(BUT_15_PIN) == LOW) {buttonBitChange(BUT_15_BIT, 1);} else {buttonBitChange(BUT_15_BIT, 0);} // Button 15
    #endif

    #if defined(BUTTON_BOX_PRO_MICRO) 
      if (digitalRead(BUT_1_PIN) == LOW) {buttonBitChange(BUT_1_BIT, 1);if (BUTTON_DEBUG ==1) Serial.println("Button 1 pressed");} else {buttonBitChange(BUT_1_BIT, 0);} // Button 1
      if (digitalRead(BUT_2_PIN) == LOW) {buttonBitChange(BUT_2_BIT, 1);if (BUTTON_DEBUG ==1) Serial.println("Button 2 pressed");} else {buttonBitChange(BUT_2_BIT, 0);} // Button 2
      if (digitalRead(BUT_3_PIN) == LOW) {buttonBitChange(BUT_3_BIT, 1);if (BUTTON_DEBUG ==1) Serial.println("Button 3 pressed");} else {buttonBitChange(BUT_3_BIT, 0);} // Button 3
      if (digitalRead(BUT_4_PIN) == LOW) {buttonBitChange(BUT_4_BIT, 1);if (BUTTON_DEBUG ==1) Serial.println("Button 4 pressed");} else {buttonBitChange(BUT_4_BIT, 0);} // Button 4
      if (digitalRead(BUT_5_PIN) == LOW) {buttonBitChange(BUT_5_BIT, 1);if (BUTTON_DEBUG ==1) Serial.println("Button 5 pressed");} else {buttonBitChange(BUT_5_BIT, 0);} // Button 5
      if (digitalRead(BUT_6_PIN) == LOW) {buttonBitChange(BUT_6_BIT, 1);if (BUTTON_DEBUG ==1) Serial.println("Button 6 pressed");} else {buttonBitChange(BUT_6_BIT, 0);} // Button 6
    #endif

}

/////////////////////////////////////////////
// When CS line goes high - the rim should get ready to transmit the next returnData buffer, right from the begining. (so that the first byte (0xA5) will be sent out on the first Clock cycle (and after CS line went Low)
void cableselect() {          
 #ifdef AVR_BOARD
    SPCR &= ~_BV(SPIE);         // turn OFF interrupts
    SPDR = data_out.raw[0];       // load first byte into SPDR single-byte's buffer
    isrIndex = 0;           // on next SPI interrupt(SPI_STC_vect), load the 2nd byte
    SPCR |= _BV(SPIE);          // turn on interrupts
  #endif
}

/////////////////////////////////////////////
// SPI interrupt routine
ISR(SPI_STC_vect)       // Note: this routine takes 6 us, and happens once 8 bits are recieved through SPI.
{ 
  #ifdef AVR_BOARD
    byte c = SPDR;
    data_in.raw[isrIndex] = c;
    
    isrIndex++;
    if (isrIndex >= dataLength) {
      isrIndex = 0;
    }
    
    SPDR = data_out.raw[isrIndex];
    //SPDR = returnData[isrIndex];
  #endif
}

/////////////////////////////////////////////
// test inputs of the wheel
void test_output() {    
  // TEst Outputs
    //Serial.print(n%24 + 1);
    //Serial.print(" - ");
      
  if ((millis()-timeStamp1) > 1000) {
    timeStamp1 = millis();
    n++;
    int i = n%24 + 1;     // to modulate between 0 and 23
  
    // change the button bit
    buttonBitChange(i, 1);
    buttonBitChange(i-1, 0);
    
    data_out.axisX = i*10;
    data_out.axisY = i*10;
  }
}

/////////////////////////////////////////////
// test LED output of the wheel
void test_leds() {
    
    int index = millis()/1000 % 9;
    switch (index) {
        case 0:
            data_in.leds = 0b0000000000000000;
            break;
        case 1:
            data_in.leds = 0b0000000000000001;
            break;
        case 2:
            data_in.leds = 0b0000000000000011;
            break;
        case 3:
            data_in.leds = 0b0000000000000111;
            break;
        case 4:
            data_in.leds = 0b0000000000001111;
            break;
        case 5:
            data_in.leds = 0b0000000000011111;
            break;
        case 6:
            data_in.leds = 0b0000000000111111;
            break;
        case 7:
            data_in.leds = 0b0000000001111111;
            break;
        case 8:
            data_in.leds = 0b0000000011111111;
            break;
        case 9:
            data_in.leds = 0b0000000111111111;
            break;
        
    }
}    

/////////////////////////////////////////////
// test 7 segments display
void test_7segments() {
  #ifdef HAS_TM1637
     int index = millis()/1000 % 9;
     data[0] = 0xff;
     data[1] = 0xff;
     data[2] = 0xff;
     data[3] = display.encodeDigit(index);
     display.setBrightness(0x0f);
     display.setSegments(data);
 #endif
}

/////////////////////////////////////////////
// Prints I/O state to Serial Output
void serialOutput() {
  Serial.print("B1 : ");
  printBin2(data_out.buttons[0]);
  Serial.print(" | B2 : ");
  printBin2(data_out.buttons[1]);
  Serial.print(" | B3 : ");
  printBin2(data_out.buttons[2]);
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
  printBin2(data_in.leds);
  Serial.print(" | Rumble : ");
  printHex(data_in.rumble[0], 2);
  Serial.print(".");
  printHex(data_in.rumble[1], 2);
  Serial.print(" - button : ");
  Serial.print(flag_button_pressed);
  Serial.println("");
}

/////////////////////////////////////////////
void buttonBitChange(uint8_t buttonBit, bool bitOn) {
   
  if ((millis() - lastButtonPress) > DEBOUNCE_DELAY) {      // Debounce
    if (bitOn) {    // changes a selected bit in one of the 3 button bytes to either on (1) or off (0). Valid values for buttonBit is 1 to 24. See buttons list in last comment.
      
      lastButtonPress = millis();
      
      if (((buttonBit <= 8) && (buttonBit >= 0))) {
        bitWrite(data_out.buttons[0], (buttonBit - 1), 1);
      }
      if (((buttonBit <= 16) && (buttonBit >= 9))) {
        bitWrite(data_out.buttons[1], (buttonBit - 1 - 8), 1);
      }
      if (((buttonBit <= 22) && (buttonBit >= 17))) {
        bitWrite(data_out.buttons[2], (buttonBit - 1 - 16), 1);
      }
    
      if (buttonBit == 23) data_out.encoder = encoder_plus;
      if (buttonBit == 24) data_out.encoder = encoder_less;

      // Handle joystick button press
      #if defined(BUTTON_BOX_PRO_MICRO) || defined(FULL_MICRO) || defined(MICRO_WITH_HEADER) 
        Joystick.setButton(buttonBit, 1);
      #endif  

    }
    else {
      if (buttonBit < 23) {
        if (((buttonBit <= 8) && (buttonBit >= 0))) {
          bitWrite(data_out.buttons[0], (buttonBit - 1), 0);
        }
        if (((buttonBit <= 16) && (buttonBit >= 9))) {
          bitWrite(data_out.buttons[1], (buttonBit - 1 - 8), 0);
        }
        if (((buttonBit <= 22) && (buttonBit >= 17))) {
          bitWrite(data_out.buttons[2], (buttonBit - 1 - 16), 0);
        }
      }
      if (buttonBit == 23) data_out.encoder = 0b00000000;
      if (buttonBit == 24) data_out.encoder = 0b00000000;
      
      // Handle joystick button press
      #if defined(BUTTON_BOX_PRO_MICRO) || defined(FULL_MICRO) || defined(MICRO_WITH_HEADER) 
        Joystick.setButton(buttonBit, 0);
      #endif  
    } 
  }
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


////////////////////////////////////////////////////////
// handling of button presses buffer, mostly required by impulsion based interfaces like encoders & rotary switches
void buttonBit_buffer_handling() {       
    for (int i=1; i<25; i++) {
        // Handle impulsion base button presses like encoder, rotary switches
        if ((buttonBits_buffer[i] > 0) && (flag_buffer == LOW)) {                 // If buffer is not empty or if a button is pressed via input_management function trigger a button press
            buttonBitChange(i, 1);
            buffer_index = i;
            flag_buffer = HIGH;              // records that button is pressed for impusion based inputs (rotary encoder, encoder, 
            buffer_timer = millis();              
            //break;
        }
        // Handle momentary button presses like classic buttons, paddles, 
        else if(button_input_buffer[i] == 1) {
            //Serial.println(i);
            buttonBitChange(i, 1);
        }
        else if ( (flag_buffer == HIGH) && (i == buffer_index))   {      // Do nothing if a buffer is ongoing
           
        }
        else {
          buttonBitChange(i, 0);
        }
    }

    if ((flag_buffer == HIGH) && ((millis()-buffer_timer) > 20)) {                  // If action is previously triggerred & timer elapsed, reset it
        buttonBitChange(buffer_index, 0);
        if(buttonBits_buffer[buffer_index] > 0) buttonBits_buffer[buffer_index]--;  // mark button press as done and decrement the buffer
        buffer_index = -1;
        flag_buffer = LOW;                                                          // records that button is released
    }
}
////////////////////////////////////////////////////////
// GEt data received from the base about the 9 rev LEDS, parse them and displays them.
void update_rev_LEDS() {
  #ifdef HAS_NPX
    if (crc8Stat == 1) {
      bool led1 = bitRead(data_in.leds,0);
      bool led2 = bitRead(data_in.leds,1);
      bool led3 = bitRead(data_in.leds,2);
      bool led4 = bitRead(data_in.leds,3);
      bool led5 = bitRead(data_in.leds,4);
      bool led6 = bitRead(data_in.leds,5);
      bool led7 = bitRead(data_in.leds,6);
      bool led8 = bitRead(data_in.leds,7);
      bool led9 = bitRead(data_in.leds,8);

      // LED 1
      if (led1) { strip.setPixelColor(0, strip.Color(0, 0, 255));}
      else {      strip.setPixelColor(0, strip.Color(0, 0, 0));}

      // LED 2
      if (led2) { strip.setPixelColor(1, strip.Color(0, 0, 255));}
      else {      strip.setPixelColor(1, strip.Color(0, 0, 0));}

      // LED 3
      if (led3) { strip.setPixelColor(2, strip.Color(0, 0, 255));}
      else {      strip.setPixelColor(2, strip.Color(0, 0, 0));}

      // LED 4
      if (led4) { strip.setPixelColor(3, strip.Color(0, 255, 0));}
      else {      strip.setPixelColor(3, strip.Color(0, 0, 0));}

      // LED 5
      if (led5) { strip.setPixelColor(4, strip.Color(0, 255, 0));}
      else {      strip.setPixelColor(4, strip.Color(0, 0, 0));}

      // LED 6
      if (led6) { strip.setPixelColor(5, strip.Color(0, 255, 0));}
      else {      strip.setPixelColor(5, strip.Color(0, 0, 0));}

      // LED 7
      if (led7) { strip.setPixelColor(6, strip.Color(255, 255, 0));}
      else {      strip.setPixelColor(6, strip.Color(0, 0, 0));}

      // LED 8
      if (led8) { strip.setPixelColor(7, strip.Color(255, 255, 0));}
      else {      strip.setPixelColor(7, strip.Color(0, 0, 0));}

      // LED 9
      if (led9) { strip.setPixelColor(8, strip.Color(255, 0, 0));}
      else {      strip.setPixelColor(8, strip.Color(0, 0, 0));}
              
      strip.show();
    
      /*Serial.print("LEDs : ");
      Serial.print(led1);
      Serial.print(" / ");
      Serial.print(led2);
      Serial.print(" / ");
      Serial.print(led3);
      Serial.print(" / ");
      Serial.print(led4);
      Serial.print(" / ");
      Serial.print(led5);
      Serial.print(" / ");
      Serial.print(led6);
      Serial.print(" / ");
      Serial.print(led7);
      Serial.print(" / ");
      Serial.print(led8);
      Serial.print(" / ");
      Serial.println(led9);*/
    }
  #endif
  // Custom code to display different thing than what's coming from the Fanatec base LEDS instruction is to write by you
}

////////////////////////////////////////////////////////
// GEt data received from the base about the 3 7 segments displays, parse them and displays them.
void update_7_segments() {
  #ifdef HAS_TM1637
    if (crc8Stat == 1) {
      /*data[0] = data_in.disp[0];
      data[1] = data_in.disp[1];
      data[2] = data_in.disp[2];
      data[3] = data_in.disp[3];
      display.setSegments(data, 3, 0);*/

      //_ASCII_table[] lookup table converts 7 segment bytes to the corresponding ASCII character.
      if (BUTTON_DEBUG ==1) Serial.print("Segments : ");
      if (BUTTON_DEBUG ==1) Serial.write(_7_to_ASCII_table[data_in.disp[0]]);
      if (BUTTON_DEBUG ==1) Serial.print(" / ");
      if (BUTTON_DEBUG ==1) Serial.write(_7_to_ASCII_table[data_in.disp[1]]);
      if (BUTTON_DEBUG ==1) Serial.print(" / ");
      if (BUTTON_DEBUG ==1) Serial.write(_7_to_ASCII_table[data_in.disp[2]]);
      if (BUTTON_DEBUG ==1) Serial.println("");
      

      data[0] = data_in.disp[0];
      data[1] = data_in.disp[1];
      data[2] = data_in.disp[2];
      data[3] = 0x00;
      display.setBrightness(7,1);     // from 0 (dimmed) to 7 (bright), 1:ON/0:OFF instruction
      display.setSegments(data);
      // Custom code to display segmments to write by you
      // Individual segment ASCII character is accessible via _7_to_ASCII_table[data_in.disp[0]] to _7_to_ASCII_table[data_in.disp[2]]
    }
  #endif
}

////////////////////////////////////////////////////////
// GEt data received from the base about the display information, parse, process them and displays them on the OLED
void update_OLED() {
  #ifdef HAS_OLED
    if (crc8Stat == 1) {  
      display2.clearDisplay();              // Clear display

      display2.setCursor(0, 0);             // Start at top-left corner
      display2.cp437(true);                 // Use full 256 char 'Code Page 437' font

      // Display the 3 characters coming from the base, converted from 7 segments data to ASCII indexed character
      display2.write(_7_to_ASCII_table[data_in.disp[0]]);
      display2.write(_7_to_ASCII_table[data_in.disp[1]]);
      display2.write(_7_to_ASCII_table[data_in.disp[2]]);
      
      display2.display();
      
      //_ASCII_table[] lookup table converts 7 segment bytes to the corresponding ASCII character.
      if (BUTTON_DEBUG ==1) Serial.print("OLED : ");
      if (BUTTON_DEBUG ==1) Serial.print(_7_to_ASCII_table[data_in.disp[0]]);
      if (BUTTON_DEBUG ==1) Serial.print(" / ");
      if (BUTTON_DEBUG ==1) Serial.print(_7_to_ASCII_table[data_in.disp[1]]);
      if (BUTTON_DEBUG ==1) Serial.print(" / ");
      if (BUTTON_DEBUG ==1) Serial.println(_7_to_ASCII_table[data_in.disp[2]]);
  
      // Custom code to display segmments to write by you
      // Individual segment ASCII character is accessible via _7_to_ASCII_table[data_in.disp[0]] to _7_to_ASCII_table[data_in.disp[2]]
    }
  #endif
}

////////////////////////////////////////////////////////
// Plays initialization animation
void play_init_rev_LEDs() {
    #ifdef HAS_NPX
      for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
        strip.setPixelColor(i, strip.Color(255,   0,   0));         //  Set pixel's color (in RAM)
        strip.show();                          //  Update strip to match
        delay(50);                           //  Pause for a moment
      }
      for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
        strip.setPixelColor(i, strip.Color(  0,   255,   0));         //  Set pixel's color (in RAM)
        strip.show();                          //  Update strip to match
        delay(50);                           //  Pause for a moment
      }
      for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
        strip.setPixelColor(i, strip.Color(  0,   0, 255));         //  Set pixel's color (in RAM)
        strip.show();                          //  Update strip to match
        delay(50);                           //  Pause for a moment
      }
      for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
        strip.setPixelColor(i, strip.Color(  0,   0, 0));         //  Set pixel's color (in RAM)
        strip.show();                          //  Update strip to match
        delay(50);                           //  Pause for a moment
      }
    #endif
}

////////////////////////////////////////////////////////
// Prints binary with leading zeroes
void printBin2(byte aByte) {
  for (int8_t aBit = 7; aBit >= 0; aBit--)
    Serial.write(bitRead(aByte, aBit) ? '1' : '0');
}


// Configure MCP inputs and outputs to detect a button press and trigger interrupt
void set_MCP_listenning_BMX() {
  mcp.pinMode(BMATRIX_ROW1, INPUT_PULLUP);
  mcp.pinMode(BMATRIX_ROW2, INPUT_PULLUP);
  mcp.pinMode(BMATRIX_ROW3, INPUT_PULLUP);
  mcp.pinMode(BMATRIX_ROW4, INPUT_PULLUP);
  mcp.pinMode(BMATRIX_ROW5, INPUT_PULLUP);
  mcp.pinMode(BMATRIX_ROW6, INPUT_PULLUP);
  mcp.pinMode(BMATRIX_ROW7, INPUT_PULLUP);

  mcp.pinMode(BMATRIX_COL1, OUTPUT);
  mcp.pinMode(BMATRIX_COL2, OUTPUT);
  mcp.pinMode(BMATRIX_COL3, OUTPUT);

  mcp.digitalWrite(BMATRIX_COL1, LOW);
  mcp.digitalWrite(BMATRIX_COL2, LOW);
  mcp.digitalWrite(BMATRIX_COL3, LOW);
}

void set_MCP_scan_cols_BMX(int int_pin) {     // Configure MCP inputs and outputs to check a particular row
    
  mcp.pinMode(BMATRIX_COL1, INPUT_PULLUP);
  mcp.pinMode(BMATRIX_COL2, INPUT_PULLUP);
  mcp.pinMode(BMATRIX_COL3, INPUT_PULLUP);
  
  mcp.pinMode(int_pin, OUTPUT);
  mcp.digitalWrite(int_pin, LOW);
  
  
  /*
  //mcp.pinMode(int_pin, OUTPUT);
  mcp.pinMode(BMATRIX_ROW1, OUTPUT);
  mcp.pinMode(BMATRIX_ROW2, OUTPUT);
  mcp.pinMode(BMATRIX_ROW3, OUTPUT);
  mcp.pinMode(BMATRIX_ROW4, OUTPUT);
  mcp.pinMode(BMATRIX_ROW5, OUTPUT);
  mcp.pinMode(BMATRIX_ROW6, OUTPUT);
  mcp.pinMode(BMATRIX_ROW7, OUTPUT);
  
  // mcp.digitalWrite(int_pin, LOW);
  mcp.digitalWrite(BMATRIX_ROW1, LOW);
  mcp.digitalWrite(BMATRIX_ROW2, LOW);
  mcp.digitalWrite(BMATRIX_ROW3, LOW);
  mcp.digitalWrite(BMATRIX_ROW4, LOW);
  mcp.digitalWrite(BMATRIX_ROW5, LOW);
  mcp.digitalWrite(BMATRIX_ROW6, LOW);
  mcp.digitalWrite(BMATRIX_ROW7, LOW);*/
    
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// function that run time essential sub-fonctions to avoid code blocking by long-time executing other ones. For example intterupt routines have to be run frequently enough
void yield_F() {

  #if defined(FULL_NANO) || defined(FULL_NANO_ESP32) || defined(FULL_MICRO) || defined(BUTTON_BOX_PRO_MICRO)  // Define this for boards having an MCP23017
    handle_ISR();   // for inputs management
  #endif

}                      