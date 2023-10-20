#ifndef config_w_h
#define config_w_h

/* This file can help you configure your wheel
  You can select which Input / outputs you want to use and to which buttons in Fanatec interface you want to associate them.

  To better understand how the wheel communicates with the base, please read the Fanatec_data_structure.h file.

  Configure the inputs / output - comment them out do deactivate them
*/


#define BUTTON_DEBUG 1          // Displays live button output on Serial port for debug purposes
#define ANALOG_SAMPLES 3        // Number of analog smaple to average for each reading. 
#define DEBOUNCE_DELAY 50       // Debounce delay - minimum delay between 2 button presses

//////////////////////////////////////////////////////////////////////////////////////////////////////
//      GENERAL CONFIGURATION                                                                       //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//#define HAS_BUTTON_GROUP_1      // Activates button group 1 - buttons number 1, 2, 3 and 4 on PCB
#define HAS_BUTTON_GROUP_2      // Activates button group 2 - buttons number 5, 6, 7 and 8 on PCB
//#define HAS_BUTTON_GROUP_3      // Activates button group 3 - buttons number 9, 10, 11 and 12 on PCB
//#define HAS_BUTTON_GROUP_4      // Activates button group 4 - buttons number 13, 14, 15 and 16 on PCB

//#define HAS_DPAD                // Activated Dpad buttons
#define HAS_FUNKY                // Activated FUNKY switch buttons - center press not functionnal

//#define HAS_RS_1                // Activates rotary switch 1 + buttonbits assignment
//#define HAS_RS_2                // Activates rotary switch 1 + buttonbits assignment
//#define HAS_RS_3                // Activates rotary switch 1 + buttonbits assignment
#define HAS_RS_4                // Activates rotary switch 1 + buttonbits assignment

#define HAS_APM_L               // Activates advance paddle module left
#define HAS_APM_R               // Activates advance paddle module RIGHT
//#define HAS_JOY                 // Activates Joystick

#define HAS_ENC_1               // Activates Encoder 1 + buttonbits assignment
#define HAS_ENC_2               // Activates Encoder 2 + buttonbits assignment
//#define HAS_ENC_3               // Activates Encoder 3 + buttonbits assignment

#define HAS_NPX                 // Activates Neopixels
#define HAS_TM1637              // Activates TM1637 7 segments display
//#define HAS_OLED                // Activates OLED Display
//#define HAS_TFT                 // Activates LCD TFT Display // Not coded yet. 

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
//        BUTTON GROUPS                                                                             //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////
// BUTTON GROUP 1
#define BUTTON_1_BUTTON_BIT  5     // Button Bit assigned to button 1 on PCB
#define BUTTON_2_BUTTON_BIT  6     // Button Bit assigned to button 2 on PCB
#define BUTTON_3_BUTTON_BIT  7     // Button Bit assigned to button 3 on PCB
#define BUTTON_4_BUTTON_BIT  8     // Button Bit assigned to button 4 on PCB

//////////////////////////////////
// BUTTON GROUP 2
#define BUTTON_5_BUTTON_BIT  18     // Button Bit assigned to button 5 on PCB
#define BUTTON_6_BUTTON_BIT  11     // Button Bit assigned to button 6 on PCB
#define BUTTON_7_BUTTON_BIT  14     // Button Bit assigned to button 7 on PCB
#define BUTTON_8_BUTTON_BIT  15     // Button Bit assigned to button 8 on PCB

//////////////////////////////////
// BUTTON GROUP 3
#define BUTTON_9_BUTTON_BIT   5     // Button Bit assigned to button 9 on PCB
#define BUTTON_10_BUTTON_BIT  5     // Button Bit assigned to button 10 on PCB
#define BUTTON_11_BUTTON_BIT  5     // Button Bit assigned to button 11 on PCB
#define BUTTON_12_BUTTON_BIT  5     // Button Bit assigned to button 12 on PCB

//////////////////////////////////
// BUTTON GROUP 4
#define BUTTON_13_BUTTON_BIT  22     // Button Bit assigned to button 13 on PCB
#define BUTTON_14_BUTTON_BIT  -1     // Button Bit assigned to button 14 on PCB
#define BUTTON_15_BUTTON_BIT  -1     // Button Bit assigned to button 15 on PCB
#define BUTTON_16_BUTTON_BIT  -1     // Button Bit assigned to button 16 on PCB

//////////////////////////////////////////////////////////////////////////////////////////////////////
//        D-PAD                                                                                     //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////
// D-PAD
#define DPAD_L_BUTTON_BIT 1      // Button Bit assigned to D-Pad left button on PCB
#define DPAD_R_BUTTON_BIT 4      // Button Bit assigned to D-Pad right button on PCB
#define DPAD_U_BUTTON_BIT 3      // Button Bit assigned to D-Pad up button on PCB
#define DPAD_D_BUTTON_BIT 2      // Button Bit assigned to D-Pad down button on PCB
#define DPAD_C_BUTTON_BIT   18     // Button Bit assigned to D-Pad center button on PCB

//////////////////////////////////
// Funky switch
#define FUNKY_L_BUTTON_BIT 2      // Button Bit assigned to FUNKY left button on PCB
#define FUNKY_R_BUTTON_BIT 3      // Button Bit assigned to FUNKY right button on PCB
#define FUNKY_U_BUTTON_BIT 1      // Button Bit assigned to FUNKY up button on PCB
#define FUNKY_D_BUTTON_BIT 4      // Button Bit assigned to FUNKY down button on PCB

//////////////////////////////////////////////////////////////////////////////////////////////////////
//        ROTARY SWITCHES                                                                           //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////
// ROTARY SWITCH 1
#define RS_1_PLUS_BUTTON_BIT  -1    // Button Bit which is triggered when rotary switch 1 is turned clockwise
#define RS_1_LESS_BUTTON_BIT  -1    // Button Bit which is triggered when rotary switch 1 is turned counter-clockwise
#define RS_1_STEPS_NUMBER     8     // number of steps of RS_1

//////////////////////////////////
// ROTARY SWITCH 2
#define RS_2_PLUS_BUTTON_BIT  21    // Button Bit which is triggered when rotary switch 2 is turned clockwise
#define RS_2_LESS_BUTTON_BIT  23    // Button Bit which is triggered when rotary switch 2 is turned counter-clockwise
#define RS_2_STEPS_NUMBER     8     // number of steps of RS_2 

//////////////////////////////////
// ROTARY SWITCH 3
#define RS_3_PLUS_BUTTON_BIT  19    // Button Bit which is triggered when rotary switch 3 is turned clockwise
#define RS_3_LESS_BUTTON_BIT  20    // Button Bit which is triggered when rotary switch 4 is turned counter-clockwise
#define RS_3_STEPS_NUMBER     8     // number of steps of RS_3 

//////////////////////////////////
// ROTARY SWITCH 4
#define RS_4_PLUS_BUTTON_BIT  16    // Button Bit which is triggered when rotary switch 4 is turned clockwise
#define RS_4_LESS_BUTTON_BIT  17    // Button Bit which is triggered when rotary switch 4 is turned counter-clockwise
#define RS_4_STEPS_NUMBER     8     // number of steps of RS_4 

//////////////////////////////////////////////////////////////////////////////////////////////////////
//    ADVANCE PADDLE MODULES                                                                        //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////
// ADVANCE PADDLE MODULE LEFT (APM_L)
#define APM_L_PU_BUTTON_BIT  13     // Button Bit which is triggered when upper paddle is pressed (PU) - Paddle Down
#define APM_L_PD_BUTTON_BIT  12     // Button Bit which is triggered when lower paddle is pressed (PD) - Paddle UP
#define APM_L_ANALOG_AXIS    1      // Axis which is triggered when left analog paddle is triggered (1 is X-Axis, 2 is Y-Axis)          

//////////////////////////////////
// ADVANCE PADDLE MODULE RIGHT (APM_R)
#define APM_R_PU_BUTTON_BIT  10     // Button Bit which is triggered when upper paddle is pressed (PU) - Paddle Down
#define APM_R_PD_BUTTON_BIT  9      // Button Bit which is triggered when lower paddle is pressed (PD) - Paddle UP
#define APM_R_ANALOG_AXIS    2      // Axis which is triggered when right analog paddle is triggered (1 is X-Axis, 2 is Y-Axis)          

//////////////////////////////////////////////////////////////////////////////////////////////////////
//          JOYSTICK                                                                                //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////
// Joystick
#define JOY_BUTTON_BUTTON_BIT  1   // Button Bit which is triggered when joystick button is pressed
#define JOY_ANALOG_AXIS_X    1     // Axis which is triggered when right analog paddle is triggered (1 is X-Axis, 2 is Y-Axis)          
#define JOY_ANALOG_AXIS_Y    2     // Axis which is triggered when right analog paddle is triggered (1 is X-Axis, 2 is Y-Axis)          

//////////////////////////////////////////////////////////////////////////////////////////////////////
//          ENCODERS                                                                                //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////
// Left encoder : encoder_1
#define ENC_1_PLUS_BUTTON_BIT  1     // Button Bit which is triggered when encoder 1 is turned clockwise
#define ENC_1_LESS_BUTTON_BIT  4     // Button Bit which is triggered when encoder 1 is turned counter-clockwise

//////////////////////////////////
// Right encoder : encoder_2
#define ENC_2_PLUS_BUTTON_BIT  3     // Button Bit which is triggered when encoder 2 is turned clockwise
#define ENC_2_LESS_BUTTON_BIT  2     // Button Bit which is triggered when encoder 2 is turned counter-clockwise

//////////////////////////////////
// FUNKY encoder : encoder_3
#define ENC_3_PLUS_BUTTON_BIT  16     // Button Bit which is triggered when encoder 2 is turned clockwise
#define ENC_3_LESS_BUTTON_BIT  17     // Button Bit which is triggered when encoder 2 is turned counter-clockwise

//////////////////////////////////////////////////////////////////////////////////////////////////////
//          Other peripherals                                                                       //
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////
// NEOPIXELS LEDS
#define NPX_NUMBER  9     // Number of LEDs hooked up to the board
#ifdef HAS_NPX
  #include <Adafruit_NeoPixel.h>
#endif

//////////////////////////////////
// TM1637 display
#define SEGMENTS_NUMBER  3     // Number of 7 segments of the display
#ifdef HAS_TM1637
  #include <TM1637Display.h>
#endif


//////////////////////////////////
// OLED display
#define OLED_TYPE  1     // Type of OLED display hooked up to the board.
#ifdef HAS_OLED
    #include <SPI.h>
    #include <Wire.h>
    #include <Adafruit_GFX.h>
    #include <Adafruit_SSD1306.h>
    
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 // OLED display height, in pixels
    #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
    #define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
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

// Button group analog readings thresholds
#define BUT_THRESHOLD_1   250
#define BUT_THRESHOLD_2   600
#define BUT_THRESHOLD_3   725
#define BUT_THRESHOLD_4   850

// DPAD analog readings thresholds
#define DPAD_THRESHOLD_1   500
#define DPAD_THRESHOLD_2   810
#define DPAD_THRESHOLD_3   920//810
#define DPAD_THRESHOLD_4   990//795
#define DPAD_THRESHOLD_5   1018//950

//1023 / 1005 / 896 / 0 / 685 / 975 / 

// FUNKY SWITCH analog readings thresholds
#define FUNKY_THRESHOLD_1   300
#define FUNKY_THRESHOLD_2   600
#define FUNKY_THRESHOLD_3   750//810
#define FUNKY_THRESHOLD_4   940//795

//1023 /820 / 684 / 516 / 0

// ROtary switches analog readings thresholds
#define RS_THRESHOLD_1   73
#define RS_THRESHOLD_2   219
#define RS_THRESHOLD_3   365
#define RS_THRESHOLD_4   511
#define RS_THRESHOLD_5   657
#define RS_THRESHOLD_6   803
#define RS_THRESHOLD_7   949

#define INTERVAL_RS_1  (1023/(RS_1_STEPS_NUMBER-1))
#define INTERVAL_RS_2  (1023/(RS_2_STEPS_NUMBER-1))           
#define INTERVAL_RS_3  (1023/(RS_3_STEPS_NUMBER-1))            
#define INTERVAL_RS_4  (1023/(RS_4_STEPS_NUMBER-1))           

// APM analog readings thresholds
#define APM_THRESHOLD_1   250
#define APM_THRESHOLD_2   600

#endif
