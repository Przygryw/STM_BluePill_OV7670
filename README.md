# STM_BluePill_OV7670
Project that made me suffer for a long time, to finally get an image.
Code is based on STM32F1038t6. 
In the beginning STM32 sets itself to use OV7670 and sends bunch of regulation commands via I2C bus.
Info about each process is transferred in UART, through USB port to the terminal (I use RealTerm).
After pressing button connected to proper STM port, interrupts enabled. 
In this place VSYNC is on and when VSYNC is detected, STM waits for HREF.
Each HREF (line) interrupt starts alorithm which gets each PCLK (pixel) and then puts it into array.
When HREF occured second time, line is send to UART and then via terminal to proper file (.bin recommended if you use my test algo).

Basic settings:
  Baudrate 921600
  Resolution: 154x144
  RGB: 444
