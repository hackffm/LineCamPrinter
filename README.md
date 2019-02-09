# LineCamPrinter
LineCamPrinter based on Teensy 3.2, thermal receipt printer and TSL1401 single line camera.

See [LineCamPrinter](https://www.hackerspace-ffm.de/wiki/index.php?title=LineCamPrinter) for more information.

## Requirements
Use Arduino with Teensy 3.2 to compile. Navigate to /hardware/teensy/avr/cores/teensy3/serial2.c and modify the following line: 

    #define SERIAL2_TX_BUFFER_SIZE     40000UL // number of outgoing bytes to buffer

Use a Windows tool to modify the serial baud rate of the receipt printer to 57600 baud, otherwise it is incredibly slow. The printer then needs the use of the DTR line, but then the print speed is only limited to the amount of pixel to darken on the paper... 

## Connections
|Teensy| TSL1401 | Receipt printer | Buttons
|--|--|--|--|
| 11 | SI | | |
| 12 | CLK | | |
| A10 | AO | | |
| 9 | | TX | |
| 10 | | RX | |
| 23 | | DTR | |
| 6 | | | Trigger |
| 4 | | | Shutter Time Select 1 |
| 3 | | | Shutter Time Select 2 |
| 5 | | | Picture Eject |
| A0 | | | Voltage divider to VBat |

**Remarks:** Connect camera analog line AO via 1k to A10 of Teensy (in the middle of PCB). Avoid using the OUT line here that is there on some TSL1401 camera modules - this one has a too slow analog buffer implemented that is not necessary with Teensy.



