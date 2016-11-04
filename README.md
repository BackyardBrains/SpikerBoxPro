# SpikerBoxPro

Code Composer project for SpikerBoxPro firmware made for MSP430f5522. 
In order to run this you must first install MSP430 USB dev pack that can be found on TI website:

http://www.ti.com/tool/msp430usbdevpack

Firmware is made for a Texas Instruments MSP430f5522 controler. Controller is doing an aquisition of bioelectric signals, sends data through HID USB to host computer and controlls hardware I/O interface made for SpikerBox Pro expansion boards. 

Details of V0.09 firmware for SpikerBox Pro (Neuron and Muscle)

- samples 2 channels (A0, A1) at 10kHz with 10 bits resolution
- sends data in real time via HID datapype device class USB endpoint
- receive and sends custom messages embeded in sample data stream:
      - start/stop of sampling stream
      - inform Host about firmware version, hardware version and SpikerBox device type
      - sends information about events
      - sends information about maximal sampling rate and number of channels
      - sends information state of power rail in device (ON or OFF)
      - start firmware update procedure
      - sends information about type of device connected to hardware I/O interface
 Detailed explanation of messages and communication protocol can be found in:
 ([Guide for implementation of Host software for SpikerBox HID USB device](documentation/SpikeRecorderHIDspecification.pdf))
- controls hardware I/O interface that enables additional 2 analog channels and up to 5 digital inputs for
   triggering of events. Details on I/O interface can be found in:
   ([SpikerBox Pro Hardware SDK](documentation/SpikerBoxProHardwareSDK.pdf))
- mesure state of power rail and controls power indication RGB LED.
 


        
