.. _camera-dhyana:

Tucsen / Dhyana
---------------------------

.. image:: dhyana_95_0.jpg
.. image:: dhyana_95_1.jpg

The Dhyana95 uses backside-illuminated sCMOS thinned chip technology to avoid light interference from the wiring layer, 
thereby increasing the pixel area and improving the photoelectric conversion rate.

.. image:: dhyana_95_2.jpg

Intoduction
```````````
This plugin control a TUCSEN Dhyana (95) camera under WINDOWS, using TUCam (32 bits) SDK 1.0.0.9 library.



Prerequisite
````````````


Initialisation and Capabilities
````````````````````````````````


Camera initialisation
......................


Std capabilites
................

This plugin has been implemented in respect of the mandatory capabilites .

* HwDetInfo

 It only supports Bpp16.

* HwSync

  Supported trigger types are:
   - IntTrig
   - ExtTrigSingle
   - ExtGate
  
  
Optional capabilites
........................

* Cooling

  - Cooling method : Peltier cooling
  - Cooling temperature : Forced air (Ambient at +25 Celsius):-10 Celsius
  - The TUCam SDK allows accessing the temperature (R/W).

* HwRoi

  Roi parameters (x, y , width, height) must be power of 2 and > 32


* HwBin

  There is no hardware support for binning.


* HwShutter

  There is no shutter control.
  
* Output trigger Mode

  This plugin provides the ability to manage output triggers modes on each channel.
  Supported trigger modes : 
  
  - ExposureStart
  - Readout
  - Global
  
* Output trigger option
  
  This plugin provides the ability to manage output triggers options on each channel.
  Supported trigger options : 
   
  - Width
  - Delay
   
* Output trigger channel
 
  This plugin provides the ability to manage camera channels.
  Supported channels : 
   
  - Channel 1
  - Channel 2
  - Channel 3

Configuration
`````````````

No Specific hardware configuration are needed


How to use
````````````
