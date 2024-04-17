
# F_Interface_AL Project v0.06
##DIY steering wheel and button bow project

You want to build your own DIY Fanatec base compatible Steering Wheel? You're at the right place :-)

Welcome to the F_interface project. 

Purpose of this project is to bring a versatile platform to build DIY steering wheels intended to work with Fanatec simracing bases. 

![image](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/05ad3eea-12d0-4a2f-92bb-38c1958d1449)

Fanatec ecosystem is great for simracing, but when it comes to integrate custom DIY wheel to enrich your experience, possibilities are limited. 

You can use the Podium hub to hook up a rim and few buttons but you don’t have much more possibilities. 

The F_interface offers you the opportunity to build a custom DIY wheel with a variety of combinations including :

- Up to 24 push buttons
- Up to 2 rotary encoders
- Up to 4 rotary switches
- Advance paddle modules
- TM1637 segmented displays
- OLED displays
- TFT LCD display
- Individually addressable RGB LEDs

This project brings you both a hardware and software platform to help you build your dream DIY wheel. 

It is build on using the SPI protocol from Fanatec to communicate between the wheel and the base. This way, you don’t need any cable or wireless communication between the wheel and the base, everything goes through the pins in the center of the Fanatec QR system.

This work is based on previous work from [Darknao](https://github.com/darknao/btClubSportWheel) and [Ishachar](https://github.com/lshachar/Arduino_Fanatec_Wheel) who brought public the base of the SPI communication between the Fanatec base and the wheels.

The SPI protocol is not yet fully understood. So what is currently feasible is : 
- use up to 24 buttons (including D-pad / rotary encoders / rotary switches)  
- use 2 axis for clutch / brake / acceleration
- get 9 status LEDs
- get 7 segment display information
—> tu sum it up, all features of wheels like Porsche 918 RSR, BMW GT2 v1...

What is currently not possible :
- get RGB Led information
- get enriched information displayed en latest OLED displays like lap time, tire pressure,…
- more buttons 
—> to sum it up, extended features brought by Maclaren GT3 v2 wheel, formula v2.5, Porsche endurance are not accessible yet, unless we better understand SPI protocol used by these wheels and by the data port.
BUT, there is a hope to integrate these feature via bluetooth using a Arduino nano ESP 32 and Simhub in future development to integrate TFT LCD screens, if your project really needs it. 


# The Hardware
The New F_interface is available in multiple form factors to suit your needs in terms of wheel setup. All board versions were designed using Fritzing, all source files are available in the repo. It is designed to be easily soldered by anyone using as few various components as possible and using only cheap and available components. It’s versatile, in the way you can only solder group of components that interest you for your build, especially if you want more rotary switches or more buttons,…

All details about the board can be found in the F_Interface_HW file. 

On the hardware side, you now have the choice between various versions depending on your needs. They vary in terms of sizes and inputs/output capabilities. They though all use the same base schematic to establish the link with the Fanatec base.

A presentation of the range is available below. The main hardware platform which is described if the original one beeing the FULL_NANO version. 

- A smallest form factor PCB based on an ATMEGA328P microcontroller just to unlock the force feedback for advanced users : ATMEGA328P_NO_HEADER
- A smallest form factor PCB based on an ATMEGA328P microcontroller just to unlock the force feedback + a few buttons for advanced user : ATMEGA328P_WITH_HEADER
- A small form factor PCB based on an arduino nano just to unlock the force feedback : NANO_NO_HEADER
- A small form factor PCB based on an arduino nano just to unlock the force feedback + a few buttons : NANO_WITH_HEADER
- A small form factor PCB based on an arduino PRO MICRO just to unlock the force feedback + a few buttons : PROMICRO_WITH_HEADER
- A podium hub form factor PCB based on an arduino nano with full features (buttons, rotary encoder, rotary switches) : FULL_NANO
- A podium hub form factor PCB based on an arduino PRO MICRO with full features (buttons, rotary encoder, rotary switches) : FULL_PROMICRO
- A button box wich is not intended to work with a Fanatec wheel base but which is very close in terms of I/Os.
![image](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/3a3cffeb-f88c-4be3-9b74-080740cbc458)


![image](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/f12a07e1-ef71-4b7d-ae6c-28a475392876)

![image](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/89bfb240-84ee-4d4c-89a4-aa0e77400f30)

![image](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/780d1046-e766-4f7a-a386-a11db42bdd44)

![image](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/5a700041-33e2-458d-90e8-8f59c6465f52)

![image](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/db05a0d0-50b5-450f-8606-36714938fa43)

![image](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/e76fd763-71e5-464b-bcad-9219753a48e5)

![image](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/07957c85-cdcc-4949-8595-0379a224bebf)

# Force feedback unlocker

Work from user Lele1494 is shared in the repo in the folder FFB unlocker_Lele1494.

It features a Fanatec force feedback unlocker in the shape of a small dongle. 

![311509914-fe021610-d0ed-4bec-9fa5-f2b88aa8f5ab](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/c37d99e1-1686-4526-87cb-87205821473f)
![313604595-764e521d-4c99-446e-8980-a8b3b699f9a5](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/5995927e-afc0-403b-94de-05d8257cd5ff)
![313604620-f500a8e4-6ad1-47dd-8eb8-25f93431ef64](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/8aa6c685-336d-41f9-9724-ee9345af024b)

# Old presentation for legacy : F_Interface_AL Project v0.02

You want to build your own DIY Fanatec base compatible Steering Wheel? You're at the right place :-)

Welcome to the F_interface project. 

Purpose of this project is to bring a versatile platform to build DIY steering wheels intended to work with Fanatec simracing bases. 

![image](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/05ad3eea-12d0-4a2f-92bb-38c1958d1449)


Fanatec ecosystem is great for simracing, but when it comes to integrate custom DIY wheel to enrich your experience, possibilities are limited. 

You can use the Podium hub to hook up a rim and few buttons but you don’t have much more possibilities. 

The F_interface offers you the opportunity to build a custom DIY wheel with a variety of combinations including :

- Up to 24 push buttons
- Up to 2 rotary encoders
- Up to 4 rotary switches
- Advance paddle modules
- TM1637 segmented displays
- OLED displays
- TFT LCD display
- Individually addressable RGB LEDs

This project brings you both a hardware and software platform to help you build your dream DIY wheel. 

It is build on using the SPI protocol from Fanatec to communicate between the wheel and the base. This way, you don’t need any cable or wireless communication between the wheel and the base, everything goes through the pins in the center of the Fanatec QR system.

This work is based on previous work from [Darknao](https://github.com/darknao/btClubSportWheel) and [Ishachar](https://github.com/lshachar/Arduino_Fanatec_Wheel) who brought public the base of the SPI communication between the Fanatec base and the wheels.

The SPI protocol is not yet fully understood. So what is currently feasible is : 
- use up to 24 buttons (including D-pad / rotary encoders / rotary switches)  
- use 2 axis for clutch / brake / acceleration
- get 9 status LEDs
- get 7 segment display information
—> tu sum it up, all features of wheels like Porsche 918 RSR, BMW GT2 v1...

What is currently not possible :
- get RGB Led information
- get enriched information displayed en latest OLED displays like lap time, tire pressure,…
- more buttons 
—> to sum it up, extended features brought by Maclaren GT3 v2 wheel, formula v2.5, Porsche endurance are not accessible yet, unless we better understand SPI protocol used by these wheels and by the data port.
BUT, there is a hope to integrate these feature via bluetooth using a Arduino nano ESP 32 and Simhub in future development to integrate TFT LCD screens, if your project really needs it. 

# The Hardware
The F_interface hardware was designed using Fritzing, all source files are available in the repo. It is designed to be easily soldered by anyone using as few various components as possible and using only cheap and available components. It’s versatile, in the way you can only solder group of components that interest you for your build, especially if you want more rotary switches or more buttons,…

All details about the board can be found in the F_Interface_vM file. 

<img width="352" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/f327799d-820b-4bf4-8f58-2bab7216fdd2">

Example of assembled PCB
![AUDI_DTM_AL1](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/e70b61e8-7912-476e-8376-082202e174dc)

Example of integration in a DIY steering wheel
![AUDI_DTM_AL6](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/f3aeb342-67dc-4d44-a3ce-de1bea5175cb)

## Dimensions 
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/7790480c-969f-4794-a90d-17e17b044e7c">

## BOM
Bill of Material will vary depending on you project but the full list is here :
- 1 arduino nano (Approx. 3€)
- 1 PCB (Approx. 10€ if ordered by 5 on JLCPCB)
- 1 CD74HC4067 - 16 channel multiplexer (Approx. 0,5€)
- 1 Step up voltage regulator (Approx. 2€)
- 1 Level shifter (Approx 2€)
- Depending on you build, some buttons, encoder, rotary switches,…
- Some 2,54 and 2mm pitch, straight or elbowed pin arrays to hook up your buttons, switches,…
- Up to 4x 100 Ohms Resistors for Rotary switches
- Up to 26x 10k Ohms Resitors for buttons
- Up to 8x 10 or 47 nF Capacitors for inputs debounce

# The Software
The F_interface software was designed based on Darknao and Ishachar previous work. 
It handles SPI communication with the base to respect the Fanatec data structure.

It has been build in a versatile way in order to configure your wheel with the input / output you want on your wheel. 
see config_wheel.h to active / deactivate via #define //#define the various options

# More details
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/c9fc6f53-a6ff-4284-ac24-a52525f89764">

<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/34862f4f-bf23-4b5c-a13b-1b1f453420c0">

Abbreviations are used on the PCB and in the software to refer to input types :

APM	Advance Paddle Module
RS	Rotary switch
BUT	Button group
ENC 	Encoder
JOY	Joystick
DPAD	Group of 5 buttons that make a direction pad – up / down / left / right / center
<img width="669" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/121464bc-9fe0-424a-9864-cd4b01a13729">

General schematic will help you understand the PCB wiring and link with the code.
It can be found in the Fritzing file

![F_Interface_AL_Hardware_v0 02_bb](https://github.com/Alexbox364/F_Interface_AL/assets/17022734/ec5a1f42-6b32-48bb-b337-7370230d49e7)

## Arduino Nano
The setup uses a classic arduino nano, original or clone.
As the wheels run on 3,3v for SPI communication while arduino runs at 5V, some adaptations are necessary like a level shifter and a step up voltage regulator. 

As the base requires a fast response from arduino on SPI communication port at startup, the arduino bootloader has to be removed. Arduino will then be programmed via ICSP port. See this page for further details : [Arduino as ISP](https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoISP)

## Step up voltage regulator
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/d162fe5a-2789-47ee-a165-1b3ca6de2bbf">

## Level shifter
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/b991780d-8508-4975-99e6-9b70c5600a94">

## Wheel input
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/4f81d53c-a84c-4791-ad2c-9f7ce7c9562b">

## CD74HC4067 multiplexer
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/dd78e55c-3b8b-4550-a36a-a48f128be233">

## Rotary switch
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/f78fc1c2-3e2c-4c21-b121-b32fe3a48c1b">

## Encoder
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/5123f462-2a70-4c5f-b2bd-42bf7b582a57">

## Button groups
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/2381482e-1d71-4c58-b1c2-bb351b275e74">

## APM
I advise you used the fantastic design from [Stuyo] (https://diy-sim.com/sim-projects/sim-gear/item/dual-paddle-shifter-clutch-module-v15-
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/4c13e8f3-85af-488d-8983-b5002c5c25f7">

## DPAD
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/fea6c3cf-9f0c-4c01-a6aa-b6d17b89c75d">

## Joystick
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/70b9521a-addf-4633-a44c-272ec2fd1055">

## TM1637
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/397309bb-a431-4c1f-8f61-b7df1ef2dd59">

## OLED display
<img width="612" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/2abdbbab-0ad6-4740-aacf-9b4e0431523c">














