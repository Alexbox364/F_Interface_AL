#ifndef PIN_H
#define PIN_H

// This file is used to assign pins to peripherals of the wheel

// Pins Arduino

#define ENC_3_A_PIN         A5 // Can be used as TM1637 is not used in the project
#define ENC_3_B_PIN         A4 // Can be used as TM1637 is not used in the project
#define INT_CS_SPI          2
#define MUX_S0_PIN          3
#define MUX_S1_PIN          4
#define ENC_2_A_PIN         5
#define ENC_2_B_PIN         6
#define MUX_S2_PIN          7
#define MUX_S3_PIN          8
#define NPX_PIN             9
#define CS_PIN              10
#define MOSI_PIN            11
#define MISO_PIN            12
#define CLK_PIN             13

#define TM1637_DIO_PIN      A0
#define TM1637_CLK_PIN      A4

#define MUX_SIG_PIN         A1
#define ENC_1_A_PIN         A3
#define ENC_1_B_PIN         A2



// CD74HC4067 pinout

#define RS_1_PIN            0    // C0
#define BUT_1_PIN           1    // C1
#define APM_L_ANALOG_PIN    2    // C2
#define APM_L_BUT_PIN       3    // C3
#define BUT_3_PIN           4    // C4
#define RS_3_PIN            5    // C5
#define DPAD_PIN            6    // C6
#define JOY_X_PIN           7    // C7
#define JOY_Y_PIN           8    // C8
#define JOY_BUT_PIN         9    // C9
#define RS_4_PIN            10   // C10
#define BUT_4_PIN           11   // C11
#define APM_R_BUT_PIN       12   // C12
#define APM_R_ANALOG_PIN    13   // C13
#define BUT_2_PIN           14   // C14
#define RS_2_PIN            15   // C15

#endif
