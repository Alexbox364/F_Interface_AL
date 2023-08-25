# F_Interface_AL Project v0.02

You want to build your own DIY Fanatec base compatible Steering Wheel? You're at the right place :-)

Welcome to the F_interface project. 

Purpose of this project is to bring a versatile platform to build DIY steering wheels intended to work with Fanatic simracing bases. 

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

<img width="557" alt="image" src="https://github.com/Alexbox364/F_Interface_AL/assets/17022734/b5dcc4ed-7849-4299-a5e0-010143d4b87b">

## Arduino Nano
The setup uses a classic arduino nano, original or clone.
As the wheels run on 3,3v for SPI communication while arduino runs at 5V, some adaptations are necessary like a level shifter and a step up voltage regulator. 

As the base requires a fast response from arduino on SPI communication port at startup, the arduino bootloader has to be removed. Arduino will then be programmed via ICSP port. See this page for further details : https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoISP

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














