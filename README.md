# ADS124S08
Very simplistic and stripped down Particle Photon/Electron library for the TI chip ADS124S08 (and similar). This MAY work with other Arduino-esque microcontrollers, but it's not tested with anything other than the Particle platform.

My primary goal was to work with PT100 temperature sensors, so when that was successful I stopped developing the library. Feel free to fork and take it further with more examples and features.

## Getting the chip up and running
For this chip you really have to read the entire datasheet. There's important info strewn across the entire datasheet and without it you'll get no response from the chip. To excite a 4-wire PT100 sensor you'll need to get all of this right:

- Tie the CLK pin to DGND if the internal oscillator is used
- Tie the START/SYNC pin to DGND to control conversions by commands
- Tie the RESET pin to IOVDD if the RESET pin is not used
- If the DRDY output is not used, leave the DRDY pin unconnected or tie the DRDY pin to IOVDD using a weak pullup resistor
- The internal reference must be on for the IDAC to work
- If the internal reference is used, REFCOM must be connected to ground
- If the internal reference is used, REFOUT should have a 1-47uf cap to REFCOM

There's no application note that tells you how these all play together. You'll have to read every bit of information in the datasheet to get anything sensible out of the chip, so read carefully and download Application Notes for similar ADC's from TI to learn how to make your specific application work. There's several similarities, so if you find an application note that covers your application, it'll make it faster to figure out what chip specific features to turn on/off.
