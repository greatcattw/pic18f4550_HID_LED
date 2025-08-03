# pic18f4550_HID_LED
pic18f4550 HID class with 8 LEDs on/off  
VID=0xffff  
PID=0x1000  
2 bytes of out end point. 1 of byte for 8 LEDs.  
<br>

## note
I added a file of cfg_bit.h to specify the settings in pic18f4550.  
Thus, select the file of hex that matches your board.  

<br>![pic](pic/a.jpg)
<br>
This EVK with 24Mhz xtal.  
<br>
<br>![pic](pic/b.jpg)<br>
<br>
This EVK with 20Mhz xtal.  
<br>

## Flash device of pic18f4550
![pic](pic/pg24a.JPG)<br>
Run MPLAB IDE  
Select your programmer  
<br>
![pic](pic/pg24b.JPG)<br>
Import the file of hex  
<br>
![pic](pic/pg24c.JPG)<br>
Check the setting of clock that matches you board  
<br>

## Test with linux  

![pic](pic/t1.png)<br>
Turn on 8 of LED.  
<br>
![pic](pic/t2.png)<br>
Turn off 8 of LED.  
<br>

