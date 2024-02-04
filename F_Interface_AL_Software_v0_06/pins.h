#ifndef PIN_H
#define PIN_H

#include "config_wheel.h" 

#define AVR_BOARD
#ifdef AVR_BOARD

  #ifdef FULL_NANO
    // NEW PINOUT
    #define INTERRUPT_PIN       3     // Pin D3 of arduino nano - equivalent to GPIO 6- interrupt signal coming from MCP23017
    #define INT_CS_SPI          2     // D2
    #define LCD_CS_PIN          5     // D5
    #define LCD_RS_PIN          6     // D6
    #define BUT_1_PIN           7     // D7
    #define BUT_2_PIN           8     // D8
    #define NPX_PIN             9     // D9
    #define CS_PIN              10    // D10
    #define MOSI_PIN            11    // D11
    #define MISO_PIN            12    // D12
    #define CLK_PIN             13    // D13
    #define FUNKY_A_PIN         A7    // A7
    #define DPAD_PIN            A7    // A7
    #define RS_1_PIN            A0    // A0
    #define RS_2_PIN            A1    // A1
    #define RS_3_PIN            A2    // A2
    #define RS_4_PIN            A3    // A3
    // A4 - I2C - SDA
    // A5 - I2C - SCL
    #define AXIS_X_PIN          A3    // A3
    #define AXIS_Y_PIN          A6    // A6
  #endif

#ifdef FULL_MICRO
    #define INTERRUPT_PIN       0     // Pin D3 of arduino nano - equivalent to GPIO 6- interrupt signal coming from MCP23017
    #define INT_CS_SPI          7     // D2
    #define LCD_CS_PIN          5     // D5
    #define LCD_RS_PIN          6     // D6
    #define BUT_1_PIN           A0  
    #define BUT_2_PIN           A1   
    #define BUT_3_PIN           1  
    #define BUT_4_PIN           5   
    #define NPX_PIN             A2   
    #define CS_PIN              7 
    #define MOSI_PIN            16
    #define MISO_PIN            14
    #define CLK_PIN             15
    #define FUNKY_A_PIN         A3   
    #define DPAD_PIN            A3   
    #define RS_1_PIN            A6    
    #define RS_2_PIN            A7   
    #define RS_3_PIN            A8
    #define RS_4_PIN            A9  
    // A4 - I2C - SDA
    // A5 - I2C - SCL
    #define AXIS_X_PIN          A9    // A3
    #define AXIS_Y_PIN          A10    // A6
  #endif

#ifdef BUTTON_BOX_PRO_MICRO
    #define INTERRUPT_PIN       0     // Pin D3 of arduino nano - equivalent to GPIO 6- interrupt signal coming from MCP23017
    #define INT_CS_SPI          2     // D2
    #define LCD_CS_PIN          5     // D5
    #define LCD_RS_PIN          6     // D6
    #define BUT_1_PIN           A1  
    #define BUT_2_PIN           A0   
    #define BUT_3_PIN           15  
    #define BUT_4_PIN           14   
    #define BUT_5_PIN           16  
    #define BUT_6_PIN           7   
    #define NPX_PIN             A2   
    #define CS_PIN              7 
    #define MOSI_PIN            16
    #define MISO_PIN            14
    #define CLK_PIN             15
    #define FUNKY_A_PIN         A3   
    #define FUNKY_PUSH          1  
    #define DPAD_PIN            A3   
    #define RS_1_PIN            6    
    #define RS_2_PIN            4   
    #define RS_3_PIN            4
    #define RS_4_PIN            A9   
    // A4 - I2C - SDA
    // A5 - I2C - SCL
    #define AXIS_X_PIN          9    // A3
    #define AXIS_Y_PIN          10    // A6
  #endif

  #if defined(NANO_NO_HEADER) || defined(NANO_WITH_HEADER)  || defined(ATMEGA328P_NO_HEADER)  || defined(ATMEGA328P_WITH_HEADER) 
    #define INTERRUPT_PIN       3     // Pin D3 of arduino nano - equivalent to GPIO 6- interrupt signal coming from MCP23017
    #define INT_CS_SPI          2     // D2
    #define BUT_1_PIN           0    
    #define BUT_2_PIN           1    
    #define BUT_3_PIN           3    
    #define BUT_4_PIN           4    
    #define BUT_5_PIN           5    
    #define BUT_6_PIN           6    
    #define BUT_7_PIN           7     
    #define BUT_8_PIN           8     
    #define BUT_9_PIN           9     
    #define BUT_10_PIN          A0    
    #define BUT_11_PIN          A1    
    #define BUT_12_PIN          A2    
    #define BUT_13_PIN          A3    
    #define BUT_14_PIN          A4    
    #define BUT_15_PIN          A5    
    #define NPX_PIN             9     // D9
    #define CS_PIN              10    // D10
    #define MOSI_PIN            11    // D11
    #define MISO_PIN            12    // D12
    #define CLK_PIN             13    // D13
    #define FUNKY_A_PIN         A7    // A7
    #define DPAD_PIN            A7    // A7
    #define RS_1_PIN            A0    // A0
    #define RS_2_PIN            A1    // A1
    #define RS_3_PIN            A2    // A2
    #define RS_4_PIN            A3    // A3
    // A4 - I2C - SDA
    // A5 - I2C - SCL
    #define AXIS_X_PIN          A3    // A3
    #define AXIS_Y_PIN          A6    // A6
  #endif

  #if defined(MICRO_WITH_HEADER)
    #define INTERRUPT_PIN       3     // Pin D3 of arduino nano - equivalent to GPIO 6- interrupt signal coming from MCP23017
    #define INT_CS_SPI          7     // D2
    #define BUT_1_PIN           0    
    #define BUT_2_PIN           1    
    #define BUT_3_PIN           3    
    #define BUT_4_PIN           4    
    #define BUT_5_PIN           5    
    #define BUT_6_PIN           6    
    #define BUT_7_PIN           2     
    #define BUT_8_PIN           8     
    #define BUT_9_PIN           9     
    #define BUT_10_PIN          A0    
    #define BUT_11_PIN          A1    
    #define BUT_12_PIN          A2    
    #define BUT_13_PIN          A3    
    #define BUT_14_PIN          10    
    #define BUT_15_PIN          A4    
    #define NPX_PIN             9     // D9
    #define CS_PIN              7    // D10
    #define MOSI_PIN            16
    #define MISO_PIN            14
    #define CLK_PIN             15
    #define FUNKY_A_PIN         A7    // A7
    #define DPAD_PIN            A7    // A7
    #define RS_1_PIN            A0    // A0
    #define RS_2_PIN            A1    // A1
    #define RS_3_PIN            A2    // A2
    #define RS_4_PIN            A3    // A3
    // A4 - I2C - SDA
    // A5 - I2C - SCL
    #define AXIS_X_PIN          A3    // A3
    #define AXIS_Y_PIN          A6    // A6
  #endif 

#else   // ESP32 based boards

    ////////////////////////////////////////////////////////////////////////////////////////////
    // ARDUINO pins
    // ATTENTION : expressed with GPIO number and not number written on Arduino PCB

        #define INTERRUPT_PIN       6     // Pin D3 of arduino nano -equivalent to GPIO 6- interrupt signal coming from MCP23017
        #define lCD_CLK_PIN          1     // D2
        #define LCD_CS_PIN          1     // D5
        #define LCD_RS_PIN          1     // D6
        #define BUT_1_PIN           1     // D7
        #define BUT_2_PIN           1    // D8
        #define NPX_PIN             1    // D9
        #define CS_PIN              1    // D10
        #define MOSI_PIN            1    // D11
        #define MISO_PIN            1    // D12
        #define CLK_PIN             1    // D13
        #define FUNKY_A_PIN         1     // A7
        #define DPAD_PIN            1     // A7
        #define RS_1_PIN            1     // A0
        #define RS_2_PIN            1     // A1
        #define RS_3_PIN            1     // A2
        #define RS_4_PIN            1     // A3
        // A4 - I2C - SDA
        // A5 - I2C - SCL
        #define AXIS_X_PIN          1     // A3
        #define AXIS_Y_PIN          1     // A6
      
#endif
// This file is used to assign pins to peripherals of the wheel

////////////////////////////////////////////////////////////////////////////////////////////
// MCP23017 pins
// MCP23017 pin mapping goes from 1 to 16 (A0 = 0 / A7 = 7 / B0 = 8 / B7 = 15    
    #define BMATRIX_COL1 0         //{"Name":"BMATRIX_COL1","Title":"Column 1 pin","DefaultValue":"2","Type":"pin;Button M. Col 1","Condition":"ENABLED_BUTTONMATRIX>0 && BMATRIX_COLS>=1"}
    #define BMATRIX_COL2 1         //{"Name":"BMATRIX_COL2","Title":"Column 2 pin","DefaultValue":"3","Type":"pin;Button M. Col 2","Condition":"ENABLED_BUTTONMATRIX>0 && BMATRIX_COLS>=2"}
    #define BMATRIX_COL3 2         //{"Name":"BMATRIX_COL3","Title":"Column 3 pin","DefaultValue":"4","Type":"pin;Button M. Col 3","Condition":"ENABLED_BUTTONMATRIX>0 && BMATRIX_COLS>=3"}
    
    #define BMATRIX_ROW1 15         //{"Name":"BMATRIX_ROW1","Title":"Row 1 pin","DefaultValue":"6","Type":"pin;Button M. Row 1","Condition":"ENABLED_BUTTONMATRIX>0 && BMATRIX_ROWS>=1"}
    #define BMATRIX_ROW2 14         //{"Name":"BMATRIX_ROW2","Title":"Row 2 pin","DefaultValue":"7","Type":"pin;Button M. Row 2","Condition":"ENABLED_BUTTONMATRIX>0 && BMATRIX_ROWS>=2"}
    #define BMATRIX_ROW3 13         //{"Name":"BMATRIX_ROW3","Title":"Row 3 pin","DefaultValue":"8","Type":"pin;Button M. Row 3","Condition":"ENABLED_BUTTONMATRIX>0 && BMATRIX_ROWS>=3"}
    #define BMATRIX_ROW4 12         //{"Name":"BMATRIX_ROW4","Title":"Row 4 pin","DefaultValue":"9","Type":"pin;Button M. Row 4","Condition":"ENABLED_BUTTONMATRIX>0 && BMATRIX_ROWS>=4"}
    #define BMATRIX_ROW5 11         //{"Name":"BMATRIX_ROW5","Title":"Row 5 pin","DefaultValue":"2","Type":"pin;Button M. Row 5","Condition":"ENABLED_BUTTONMATRIX>0 && BMATRIX_ROWS>=5"}
    #define BMATRIX_ROW6 10         //{"Name":"BMATRIX_ROW6","Title":"Row 6 pin","DefaultValue":"2","Type":"pin;Button M. Row 6","Condition":"ENABLED_BUTTONMATRIX>0 && BMATRIX_ROWS>=6"}
    #define BMATRIX_ROW7 5         //{"Name":"BMATRIX_ROW7","Title":"Row 7 pin","DefaultValue":"2","Type":"pin;Button M. Row 7","Condition":"ENABLED_BUTTONMATRIX>0 && BMATRIX_ROWS>=7"}
    
    #define ENC_1_A_PIN 7
    #define ENC_1_B_PIN 6
    #define ENC_2_A_PIN 3
    #define ENC_2_B_PIN 4
    #define ENC_3_A_PIN 8
    #define ENC_3_B_PIN 9

#endif
