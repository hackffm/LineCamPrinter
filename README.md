# LineCamPrinter
LineCamPrinter based on Teensy 3.2, thermal receipt printer and TSL1401 single line camera.

See https://www.hackerspace-ffm.de/wiki/index.php?title=LineCamPrinter for more information.

## Requirements
Use Arduino with Teensy 3.2 to compile. Navigate to /hardware/teensy/avr/cores/teensy3/serial2.c and modify the line 
'''
#define SERIAL2_TX_BUFFER_SIZE     40000UL // number of outgoing bytes to buffer
'''

