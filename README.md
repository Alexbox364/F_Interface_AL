# F_Interface_AL
You want to build your own DIY Fanatec base compatible Steering Wheel? You're at the right place :-)

Welcome to the F_interface project. 

Purpose of this project is to bring a versatile platform to build DIY steering wheels intended to work with Fanatic simracing bases. 

Fanatec ecosystem is great for simracing, but when it comes to integrate custom DIY wheel to enrich your experience, possibilities are limited. 

You can use the Podium hub to hook up a rim and few buttons but you don’t have much more possibilities. 

The F_interface offers you the opportunity to build a custom DIY wheel with a variety of combinations including :

- Up to 24 push buttons
- Up to 2 rotary encoders
- Up to 4 rotary switches
- Advance paddle modules
- TM1637 segmented displays
- OLED displays
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

The Hardware :
The F_interface hardware was designed using Fritzing, all source files are available in the repo. It is designed to be easily soldered by anyone using as few various components as possible and using only cheap and available components. It’s versatile, in the way you can only solder group of components that interest you for your build, especially if you want more rotary switches or more buttons,…

All details about the board can be found in the F_Interface_vM file. 

The Software : 
The F_interface software was designed based on Darknao and Ishachar previous work. 
It handles SPI communication with the base to respect the Fanatec data structure.

