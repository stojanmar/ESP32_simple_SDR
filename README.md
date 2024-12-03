# ESP32_simple_SDR
This project is an attempt to make SDR receiver based purrely on ESP32. This means, that input and ouptut sampling is made without any external ADC and DAC converters (codex).
Internal ADC and internal DAC are used to realize input and output sampling simultaneously.
The project is in early development and need some improovements. Therefore the documentation will follow later.
To make a first trial:
- Use GPIO33 for Analog input. Ensure the input is connected to reference voltage of 1,6 VDC through an resistor of about 47k; adjust the offset variable in SW to ensure simetrical sampling.
  (later this will be done by automatic calibration routine in SW).
  Apply a I or Q signal from an Tayloe Mixer or use one channel from IQ signnals from one of many Kiwi SDR internet sights. The signals are available on earphone output on PC or notebook.
  Kiwi radio sights have limited bandwith in IQ stream to 12khz. Since this version of SW needs a bandwidth of at least 24kHz the demodulated sound is much affected, Adequat quality is establiched
  only with Tayloe output. This SDR even does not need a Tayloe as it needs only one channel from mixer (I and Q are provided in SW). I intend to run this  setup in near future simply by using an conventional Shrt 
  wave received, modifing it for an low frequency IF outup.
  This first attempt is sampling with 48kHz to be able to use bandwith of Kiwi  IQ stream. My next attempt will be sampling with 96kHz and next with 120kHz. the ESP build in ADC can sample much higer,
  so this may leed me to even higher rates. 
  
- Use GPIO26 for DAC output to any AF amplifier through an capacitor. It is recommended to use RC filtering on pin 26 using an resistor of 4.7k and capacitor of 22nF before input to AF amplifier.
- Use USB serial port monitor (115200) to change different setings like switch demodulation between USB, LSB, AM and FM. Commands: usb, lsb, am ,fm  .....

  A LOT TO FOLLOW - among others:
  - polich the operation for best receiver signal noise experience
  - add a display
  - allow hardware switches and encoders
  - add a remote control by using serial RX pin of ESP32 and avoid need for USB connection
  - check if TX operation is possible
  - ...... 
 
