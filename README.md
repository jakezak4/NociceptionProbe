# Nociception Probe with Magnet Wire
Probe system for assaying nociception in Drosophila.  
Designed by Jacob Jaszczak and Luke Breuer  
Built by Jacob Jaszczak

*Installation and Startup* 

1) Install microcontroler driver. The board URC10 uses CH340G USB to UART converter. Download Windows driver here: http://sparks.gogo.co.nz/assets/_site_/downloads/CH34x_Install_Windows_v3_4.zip  
For MAC and Linux, see URC10 Cytron manual. 

2) Install Arduino IDE 
https://www.arduino.cc/en/software

3) Download and copy files in thermocouple libraries to the Arduino libraries folder as subfolder "PWFusion_MAX31856"  
	2A: Create folder "PWFusion_MAX31856" in "libraries" directory  
	
	2B: path example: ...\Documents\Arduino\libraries\PWFusion_MAX31856
	
	2C: Add the 4 files starting with "PlayingWithFusion_" to the subfolder 

4) Download and move NociceptionProbe.ino file to Arduino folder as subfolder "NociceptionProbe".  
		*note folder name and file name must match.*   
	4A: Create folder "NociceptionProbe" in "Arduino" directory  
	
	4B: path example: ...\Documents\Arduino\thermal_degeneration\thermal_degeneration.ino  
	
	4C: Open "NociceptionProbe.ino" in Arduino IDE 

5) Set up IDE  
	4A: Tools > Board > select "Arduino Uno"
	
	4B: Tools > Port > select port that is activated by the USB connection.
						*If port does not appear, or IDE cannot connect to the board during upload, then trouble shoot through Windows Device Manager*

	4C: Tools > Serial Monitor > Baud > select "38400"

6) Set program parameters in the "Assay variables" section of code. *note, time variables must be in seconds
  - targetTemp = target temperature that program is held at. 
  - beginningHold = seconds after start button is pressed before the thermal program begins.
  - holdTarget = seconds that the target temperature is held for. After hold time is complete the Peltiers will turn off, but he fans and the thermocouples will stay on. 
  - assayTime = total assay time, afterwhich the fans will shutdown and the thermocouples will stop recording.  
  	-Total assay time needs to include beginningHold + ramp time + holdTarget + any extra for equilibrating to room temp. 
  
7) Upload "NociceptionProbe.ino" code to microcontroller (button at top left with arrow). 

8) Open Serial Monitor (Ctrl + Shift + M) to view output or run SerialPlotAndLog. 
              *Note: Serial Monitor and SerialPlotAndLog cannot be run at the same time   


*Running Temperature Treatment Program* 

1) Before running program:  
	1A: ?
  1B: ? 

2) Press the START button to begin program. The LED will turn on. 

3) To stop program at any time, press the STOP button, which will turn off the magnet wire heating but still read the thermocouples. Or press the RESET button, which will reload the code in the microcontroller and reset all variables. 


