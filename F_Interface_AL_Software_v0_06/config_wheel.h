#ifndef config_w_h
#define config_w_h

/* This file can help you configure your wheel
  You can select which Input / outputs you want to use and to which buttons in Fanatec interface you want to associate them.

  To better understand how the wheel communicates with the base, please read the Fanatec_data_structure.h file.

  Configure the inputs / output - comment them out do deactivate them
*/


#define VERSION "0.06.01"

/////////////////////////////////
// TYPE OF PCB //
/////////////////////////////////
// #define ATMEGA328P_NO_HEADER    //  PCB version with only the ATMEGA 328P to unlock the FOrce Feedback - runs at 3,3V
// #define ATMEGA328P_WITH_HEADER  //  PCB version with the ATMEGA 328P to unlock the Force Feedback and headers around the board to connect a few buttons - runs at 3,3V
// #define NANO_NO_HEADER          //  PCB version with only the Arduino nano to unlock the FOrce Feedback - runs at 5V, needs a stepup voltage regulator
// #define NANO_WITH_HEADER        //  PCB version with the Arduino nano to unlock the FOrce Feedback and headers around the board to connect a few buttons - runs at 5V, needs a stepup voltage regulator
// #define MICRO_WITH_HEADER       //  PCB version with the Arduino PRO MICRO to unlock the FOrce Feedback and headers around the board to connect a few buttons + Joystick possibility - runs at 5V, needs a stepup voltage regulator
#define FULL_NANO               //  PCB version with the Arduino nano and the MCP23017 to have a lot of option to connect buttons, rotary encoder and rotary switches - runs at 5V, needs a stepup voltage regulator
//#define FULL_NANO_ESP32         //  PCB version with the Arduino nano ESP32 and the MCP23017 to have a lot of option to connect buttons, rotary encoder and rotary switches + LCD diplay possiblity- runs at 3,3V
//  #define FULL_MICRO               //  PCB version with the Arduino PRO MICRO and the MCP23017 to have a lot of option to connect buttons, rotary encoder and rotary switches + Joystick possiblity- runs at 5V, needs a stepup voltage regulator
// #define BUTTON_BOX_PRO_MICRO  //  PCB version with the Arduino PRO MICRO and the MCP23017 to have a lot of option to connect buttons, rotary encoder and rotary switches + Joystick possiblity to build a button box - runs at 5V, needs a stepup voltage regulator


#define BUTTON_DEBUG 1           // Displays live button output on Serial port for debug purposes
#define ANALOG_SAMPLES 3         // Number of analog smaple to average for each reading.
#define ECART_RS 30              // Threshold used to declare a RS measurement stable so that is can be processed.
#define DEBOUNCE_DELAY 10        // Debounce delay - minimum delay between 2 button presses
#define TEMPORARY_PRESS_TIME 30  // Duration of a temporary button press for impulsion based inputs (like rotary encoders and rotary switches)

#if defined(FULL_NANO_ESP32)  // Define this for boards having a 12 bits ADC
#define MAX_ADC 4096          // MAx value of ADC - ESP is 12 bits : 4095, ESP32 based boards are 12 bits : 4095
#else
#define MAX_ADC 1023  // MAx value of ADC - Arduino nano & micro are 10 bits : 1023, ESP32 based boards are 12 bits : 4095
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////
//      GENERAL CONFIGURATION                                                                       //
//////////////////////////////////////////////////////////////////////////////////////////////////////

#define HAS_BUTTON_MATRIX  // Activates button matrix that has a 3x7 size

//#define HAS_DPAD                // Activated Dpad buttons
// #define HAS_FUNKY                // Activated FUNKY switch buttons - center press not functionnal

// #define HAS_RS_1                // Activates rotary switch 1 + buttonbits assignment
//#define HAS_RS_2  // Activates rotary switch 1 + buttonbits assignment
//#define HAS_RS_3  // Activates rotary switch 1 + buttonbits assignment
// #define HAS_RS_4                // Activates rotary switch 1 + buttonbits assignment

// #define HAS_ANALOG_AXIS         // Activates the analog readings of axis X and Y

#define HAS_ENC_1  // Activates Encoder 1 + buttonbits assignment
//  #define HAS_ENC_2  // Activates Encoder 2 + buttonbits assignment
// #define HAS_ENC_3  // Activates Encoder 3 + buttonbits assignment
// 
// #define HAS_NPX                 // Activates Neopixels
// #define HAS_TM1637              // Activates TM1637 7 segments display
// #define HAS_OLED                // Activates OLED Display
// #defc fine HAS_TFT                 // Activates LCD TFT Display // Not coded yet.

// SEE BELOW FOR FURTHER CONFIGURATION OPTIONS FOR EACH INPUT

/*  Button bits mapping for Fanatec wheel

Some button bits à reserved in Fanatec SPI communicaiton protocol, here are advice based on Darknao / Ishachar work and my experience

1   D-pad Up
2   D-pad Left
3   D-pad Right
4   D-pad Down
5   FREE - Button 11
6   FREE - Button 3
7   FREE - Button 6
8   FREE - Button 4
9   Right Paddle
10  FREE - Button 2   - Paddle up   
11  FREE - Button 8      
12  Left paddle
13  FREE - Button 1    - Paddle up  
14  FREE - Button 5      
15  FREE - Button 9      (3 buttons in triangle, left)
16  FREE - Button 10     (3 buttons in triangle, right)
17  FREE - Button 21
18  D-pad Button
19  FREE - Joystick Button
20  FREE - Button 7
21  FREE - Button 27 in contentmanager
22  Menu button. Button 28 in contentmanager. Does not show up in fanatec's driver)
23
24  Used to store encoder LESS here
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////
//        BUTTON MATRIX                                                                             //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////
// BUTTON MATRIX

#define ENABLED_BUTTONMATRIX 1  //{"Group":"Button matrix","Name":"ENABLED_BUTTONMATRIX","Title":"Button matrix enabled","DefaultValue":"0","Type":"bool"}
#define BMATRIX_COLS 3          //{"Name":"BMATRIX_COLS","Title":"Columns","DefaultValue":"3","Type":"int","Condition":"ENABLED_BUTTONMATRIX>0","Min":2,"Max":8}
#define BMATRIX_ROWS 7          //{"Name":"BMATRIX_ROWS","Title":"Rows","DefaultValue":"3","Type":"int","Condition":"ENABLED_BUTTONMATRIX>0","Min":2,"Max":8}

#define BMX_11_BUTTON_BIT 1   // Button Bit which is triggered when button 11 of matrix is pressed
#define BMX_12_BUTTON_BIT 22  // Button Bit which is triggered when button 12 of matrix is pressed
#define BMX_13_BUTTON_BIT 10  // Button Bit which is triggered when button 13 of matrix is pressed
#define BMX_14_BUTTON_BIT 9   // Button Bit which is triggered when button 14 of matrix is pressed
#define BMX_15_BUTTON_BIT 13  // Button Bit which is triggered when button 15 of matrix is pressed
#define BMX_16_BUTTON_BIT 12  // Button Bit which is triggered when button 16 of matrix is pressed
#define BMX_21_BUTTON_BIT 18  // Button Bit which is triggered when button 21 of matrix is pressed
#define BMX_22_BUTTON_BIT 5   // Button Bit which is triggered when button 22 of matrix is pressed
#define BMX_23_BUTTON_BIT 6   // Button Bit which is triggered when button 23 of matrix is pressed
#define BMX_24_BUTTON_BIT 15  // Button Bit which is triggered when button 24 of matrix is pressed
#define BMX_25_BUTTON_BIT 8   // Button Bit which is triggered when button 25 of matrix is pressed
#define BMX_26_BUTTON_BIT 7   // Button Bit which is triggered when button 26 of matrix is pressed
#define BMX_31_BUTTON_BIT 16  // Button Bit which is triggered when button 31 of matrix is pressed
#define BMX_32_BUTTON_BIT 21  // Button Bit which is triggered when button 32 of matrix is pressed
#define BMX_33_BUTTON_BIT 22  // Button Bit which is triggered when button 33 of matrix is pressed
#define BMX_34_BUTTON_BIT 19  // Button Bit which is triggered when button 34 of matrix is pressed
#define BMX_35_BUTTON_BIT 17  // Button Bit which is triggered when button 35 of matrix is pressed
#define BMX_36_BUTTON_BIT 20  // Button Bit which is triggered when button 36 of matrix is pressed

#define BMX_17_ENC_1_BUTTON_BIT 11  // RESERVED FOR ENC 1 BUTTON PRESS Button Bit which is triggered when button 17 of matrix is pressed
#define BMX_27_ENC_3_BUTTON_BIT 1   // RESERVED FOR ENC 3 BUTTON PRESS Button Bit which is triggered when button 27 of matrix is pressed
#define BMX_37_ENC_2_BUTTON_BIT 14  // RESERVED FOR ENC 2 BUTTON PRESS Button Bit which is triggered when button 37 of matrix is pressed

int buttonMatrix_buttonBits[21] = {
  BMX_11_BUTTON_BIT,
  BMX_12_BUTTON_BIT,
  BMX_13_BUTTON_BIT,
  BMX_14_BUTTON_BIT,
  BMX_15_BUTTON_BIT,
  BMX_16_BUTTON_BIT,
  BMX_17_ENC_1_BUTTON_BIT,
  BMX_21_BUTTON_BIT,
  BMX_22_BUTTON_BIT,
  BMX_23_BUTTON_BIT,
  BMX_24_BUTTON_BIT,
  BMX_25_BUTTON_BIT,
  BMX_26_BUTTON_BIT,
  BMX_27_ENC_3_BUTTON_BIT,
  BMX_31_BUTTON_BIT,
  BMX_32_BUTTON_BIT,
  BMX_33_BUTTON_BIT,
  BMX_34_BUTTON_BIT,
  BMX_35_BUTTON_BIT,
  BMX_36_BUTTON_BIT,
  BMX_37_ENC_2_BUTTON_BIT
};
//////////////////////////////////////////////////////////////////////////////////////////////////////
//        D-PAD                                                                                     //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////
// D-PAD
#define DPAD_L_BUTTON_BIT 1   // Button Bit assigned to D-Pad left button on PCB
#define DPAD_R_BUTTON_BIT 4   // Button Bit assigned to D-Pad right button on PCB
#define DPAD_U_BUTTON_BIT 3   // Button Bit assigned to D-Pad up button on PCB
#define DPAD_D_BUTTON_BIT 2   // Button Bit assigned to D-Pad down button on PCB
#define DPAD_C_BUTTON_BIT 18  // Button Bit assigned to D-Pad center button on PCB

//////////////////////////////////
// Funky switch
#define FUNKY_L_BUTTON_BIT 1  // Button Bit assigned to FUNKY left button on PCB
#define FUNKY_R_BUTTON_BIT 4  // Button Bit assigned to FUNKY right button on PCB
#define FUNKY_U_BUTTON_BIT 2  // Button Bit assigned to FUNKY up button on PCB
#define FUNKY_D_BUTTON_BIT 3  // Button Bit assigned to FUNKY down button on PCB
#define FUNKY_C_BUTTON_BIT 5  // Button Bit assigned to FUNKY center button on PCB

//////////////////////////////////////////////////////////////////////////////////////////////////////
//        ROTARY SWITCHES                                                                           //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////
// ROTARY SWITCH 1
#define RS_1_PLUS_BUTTON_BIT 17  // Button Bit which is triggered when rotary switch 1 is turned clockwise
#define RS_1_LESS_BUTTON_BIT 19  // Button Bit which is triggered when rotary switch 1 is turned counter-clockwise
#define RS_1_STEPS_NUMBER 8      // number of steps of RS_1

//////////////////////////////////
// ROTARY SWITCH 2
#define RS_2_PLUS_BUTTON_BIT 24  // Button Bit which is triggered when rotary switch 2 is turned clockwise
#define RS_2_LESS_BUTTON_BIT 23  // Button Bit which is triggered when rotary switch 2 is turned counter-clockwise
#define RS_2_STEPS_NUMBER 8      // number of steps of RS_2

//////////////////////////////////
// ROTARY SWITCH 3
#define RS_3_PLUS_BUTTON_BIT 20  // Button Bit which is triggered when rotary switch 3 is turned clockwise
#define RS_3_LESS_BUTTON_BIT 21  // Button Bit which is triggered when rotary switch 3 is turned counter-clockwise
#define RS_3_STEPS_NUMBER 8      // number of steps of RS_3

//////////////////////////////////
// ROTARY SWITCH 4
#define RS_4_PLUS_BUTTON_BIT 19  // Button Bit which is triggered when rotary switch 4 is turned clockwise
#define RS_4_LESS_BUTTON_BIT 20  // Button Bit which is triggered when rotary switch 4 is turned counter-clockwise
#define RS_4_STEPS_NUMBER 8      // number of steps of RS_4

//////////////////////////////////////////////////////////////////////////////////////////////////////
//          Analog axis                                                                                //
//////////////////////////////////////////////////////////////////////////////////////////////////////

#define ANALOG_AXIS_X 1  // Axis which is triggered when X axis is triggered (1 is X-Axis, 2 is Y-Axis)
#define ANALOG_AXIS_Y 2  // Axis which is triggered when Y axis is triggered (1 is X-Axis, 2 is Y-Axis)

//////////////////////////////////////////////////////////////////////////////////////////////////////
//          ENCODERS                                                                                //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////
// Left encoder : encoder_1
#define ENC_1_PLUS_BUTTON_BIT 1  // Button Bit which is triggered when encoder 1 is turned clockwise
#define ENC_1_LESS_BUTTON_BIT 4  // Button Bit which is triggered when encoder 1 is turned counter-clockwise

//////////////////////////////////
// Right encoder : encoder_2
#define ENC_2_PLUS_BUTTON_BIT 3  // Button Bit which is triggered when encoder 2 is turned clockwise
#define ENC_2_LESS_BUTTON_BIT 2  // Button Bit which is triggered when encoder 2 is turned counter-clockwise

//////////////////////////////////
// FUNKY encoder : encoder_3
#define ENC_3_PLUS_BUTTON_BIT 1  // Button Bit which is triggered when encoder 2 is turned clockwise
#define ENC_3_LESS_BUTTON_BIT 4  // Button Bit which is triggered when encoder 2 is turned counter-clockwise

//////////////////////////////////
// DIRECT BUTTON PRESSES (FOR PCBs WITHOUT MCP23017)
#define BUT_1_BIT 1
#define BUT_2_BIT 2
#define BUT_3_BIT 3
#define BUT_4_BIT 4
#define BUT_5_BIT 5
#define BUT_6_BIT 6
#define BUT_7_BIT 7
#define BUT_8_BIT 8
#define BUT_9_BIT 9
#define BUT_10_BIT 10
#define BUT_11_BIT 11
#define BUT_12_BIT 12
#define BUT_13_BIT 13
#define BUT_14_BIT 14
#define BUT_15_BIT 15

//////////////////////////////////////////////////////////////////////////////////////////////////////
//          Other peripherals                                                                       //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////
// NEOPIXELS LEDS
#define NPX_NUMBER 10  // Number of LEDs hooked up to the board

//////////////////////////////////
// TM1637 display
#define SEGMENTS_NUMBER 3  // Number of 7 segments of the display
#ifdef HAS_TM1637
#include <TM1637Display.h>
#endif

//////////////////////////////////
// OLED display
#define OLED_TYPE 1  // Type of OLED display hooked up to the board.
#ifdef HAS_OLED
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////
//          THRESHOLDS                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////////

// Analog values read for a voltage diveder with equal resistor values ares :
//    0       - Button 1 pressed, 0 to 1 resistor
//    511     - Button 1 pressed, 1 to 1 resistors
//    683     - Button 2 pressed, 2 to 1 resistors
//    768     - Button 3 pressed, 3 to 1 resistors
//    820     - Button 4 pressed, 4 to 1 resistors
//    854     - Button 5 pressed, 5 to 1 resistors
//    1023    - No button pressed, 5V voltage reported

// Below thresholds are chosen accordingly to discriminate above button presses :

// DPAD analog readings thresholds
#define DPAD_THRESHOLD_1 500
#define DPAD_THRESHOLD_2 810
#define DPAD_THRESHOLD_3 920   //810
#define DPAD_THRESHOLD_4 990   //795
#define DPAD_THRESHOLD_5 1018  //950

//1023 / 1005 / 896 / 0 / 685 / 975 /

// FUNKY SWITCH analog readings thresholds
#define FUNKY_THRESHOLD_1 420
#define FUNKY_THRESHOLD_2 600
#define FUNKY_THRESHOLD_3 750  //810
#define FUNKY_THRESHOLD_4 940  //795


// 334 / 514 / 690 / 840 / 1023 /
//1023 /820 / 684 / 516 / 0

#define INTERVAL_RS_1 (MAX_ADC / (RS_1_STEPS_NUMBER - 1))
#define INTERVAL_RS_2 (MAX_ADC / (RS_2_STEPS_NUMBER - 1))
#define INTERVAL_RS_3 (MAX_ADC / (RS_3_STEPS_NUMBER - 1))
#define INTERVAL_RS_4 (MAX_ADC / (RS_4_STEPS_NUMBER - 1))

// APM analog readings thresholds
#define X_AXIS_THRESHOLD_1 250
#define Y_AXIS_THRESHOLD_2 600

#endif
