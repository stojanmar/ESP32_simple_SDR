# ESP32_simple_SDR
This project is an attempt to make SDR receiver based purrely on ESP32. This means, that input and ouptut sampling is made without any external ADC and DAC converters (codex).
Internal ADC and internal DAC are used to realize input and output sampling simultaneously. Got the inspiration from grate project All_mode_SDR made by tjlab-jf3hzb and his welthy repository https://github.com/tjlab-jf3hzb/All_mode_SDR. I based my initial work on code snippets from there and created a modified version with focus to eliminate ADC,DAC converters.
This may become a simplest and inexpensive SDR radio at least for the receive operation.
The project is in early development and needs further improovements. Therefore the documentation will follow later.
To make a first trial a version with 24kHz I2S sampling is used to proove the cocept, simply to avoid making too much HW and radther use IQ streams avalable on many KiwiSDR online radios.
My next attempt will be sampling with 48kHz, 96kHz and possibly with 120kHz. the ESP32 build in ADC can sample even much higer, so this may leed me to try even higher rates.

Breaf instructions to make a working proto:
- Use GPIO33 for Analog input. Ensure the input is connected to reference voltage of 1,6 VDC through an resistor of about 47k; adjust the offset variable in SW to ensure simetrical sampling.
  (later this will be done by automatic calibration routine in SW).
  Apply a I or Q signal from an Tayloe Mixer or use one channel from IQ signnals from one of many Kiwi SDR internet sights. The signals are available on earphone output on PC or notebook.
  Kiwi radio sights have limited bandwith in IQ stream to 12khz, and for the 24khz sampling SDR this is just on the limit. Higher sampling rates will be possible only by using 
  Tayloe mixer. This SDR needs only one channel from stream or mixer (I and Q are provided in SW).
   
- Use GPIO26 for DAC output to any AF amplifier through an capacitor. It is recommended to use RC filtering on pin 26 using an resistor of 4.7k and capacitor of 22nF before input to AF amplifier.
- Use USB serial port monitor (115200) to change different setings like switch demodulation between USB, LSB, AM and FM. Commands: usb, lsb, am ,fm  .....

  A LOT TO FOLLOW - among others:
  - Improove the operation for best receiver signal noise experience
  - add a display and touch controls
  - allow hardware switches and encoders
  - add a remote control by using serial RX pin oo ESP32 and avoid need for USB connection
  - check if TX operation is possible
  - ...... 
 
