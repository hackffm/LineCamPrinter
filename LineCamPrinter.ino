/* Teensy 3.2, LineCamPrinter
 * https://github.com/hackffm/LineCamPrinter
 * Copyright (c) 2019 Lutz Lisseck
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// mega328:    14344us -> 8.9kHz
// teensy 3.2:  1280us -> 100kHz

// Terminal  at 0 Rx,  1 Tx (115200 Baud) Serial1
// Printer   at 9 Rx, 10 Tx (57600 Baud), CTS pin 23 Serial2

#include <ADC.h>

/* Use AO not OUT on TSL1401 modules (don't use integrated amplifier as this one is crap and slow) */
#define TSL1401_SI    11
#define TSL1401_CLK   12
#define TSL1401_OUT   A10  

#define SHUTTER_BUT   6
#define SELECT1       4
#define SELECT2       3
#define EXTRA_BUT     5
#define VBAT          A0

#if SERIAL2_TX_BUFFER_SIZE < 30000UL
#warning Increase SERIAL2_TX_BUFFER_SIZE in serial2 in teensy folder to 32000 to increase performance.
#endif

// Stores direct data of one line
uint16_t TSL1401_buf16[132];

#define MINBRIGHT 16

// Stores contrast adapted data of one line (8 bit)
uint8_t  TSL1401_buf8[132];

// Dither-Algorithms need 3 lines, printing needs 8 lines already dithered
#define  DITHER_WIDTH 384
#define  DITHER_BUF_LINES 8
#define  DITHER_BUF_LINE_MASK 0x07
uint8_t  DitherBuf[DITHER_BUF_LINES][DITHER_WIDTH];
uint8_t  DitherLineIn  = 0;

#define  DITHERED_BUF_LINES 48
uint8_t  DitheredBuf[DITHERED_BUF_LINES+1][DITHER_WIDTH / 8];
int16_t  DitheredLinesIn = 0;  // Where to put dithered lines into buffer
int16_t  DitheredLines = 0;    // How many non-printed lines are in buffer
int16_t  DitheredLinesOut = 0; // From where to take lines to print from buffer

// Atkinson
//#define  DITHER_KERNEL_DIVIDER 8
//const int8_t DitherKernel[][3] = {{1, 0, 1}, {2, 0, 1}, {-1, 1, 1}, {0, 1, 1}, {1, 1, 1}, {0, 2, 1}};

// Floyd-Steinberg
#define  DITHER_KERNEL_DIVIDER 16
const int8_t DitherKernel[][3] = {{1, 0, 7}, {-1, 1, 3}, {0, 1, 5}, {1, 1, 1}};


uint16_t TSL1401_valmax;
uint16_t TSL1401_valmin;
float    TSL1401_avgmax;
float    TSL1401_avgmin;

uint32_t shutter_time = 2000;

// Stores processed data of one line
uint8_t Tx1_buf[DITHER_WIDTH+2];
uint8_t Tx2_buf[132];

volatile uint8_t TSL1401_ExposeStarted;
volatile uint8_t TSL1401_ExposeStopped;

IntervalTimer TSL1401_TimerStartExpose;
IntervalTimer TSL1401_TimerStopExpose;
ADC *adc = new ADC(); // adc object;


int SelectValue = 0;
int SelectValueOld = -1;
float VBat = 0.0;
uint32_t VBatTs = 0;

uint32_t ExtraButTs = 0;
int      ExtraButStatus = 0;

// Step 0: Initialize timers
void TSL1401_Init(int32_t ExposeDuration, int32_t ExposePeriod) {
  //uint32_t ts;
  TSL1401_TimerStartExpose.end();
  TSL1401_TimerStopExpose.end();

  digitalWrite(TSL1401_SI, LOW);
  digitalWrite(TSL1401_CLK, LOW);

  // Clkin 170 more clks
  for(int i=0;i<170;i++) {
    digitalWrite(TSL1401_CLK, HIGH);
    digitalWrite(TSL1401_CLK, LOW);    
  }

  TSL1401_PrepareExpose();
  //ts = micros();
  TSL1401_TimerStartExpose.begin(TSL1401_StartExpose, ExposePeriod);
  //while((uint32_t)((uint32_t)micros() - (uint32_t)ts) < (uint32_t)ExposeDuration) ;
  delayMicroseconds(ExposeDuration);
  TSL1401_TimerStopExpose.begin(TSL1401_StopExpose, ExposePeriod);
  
}

// Step 1
void TSL1401_PrepareExpose() {
  // Clkin SI signal
  digitalWriteFast(TSL1401_CLK, LOW);
  delayMicroseconds(1);
  digitalWriteFast(TSL1401_SI, HIGH);
  delayMicroseconds(1);
  digitalWriteFast(TSL1401_CLK, HIGH);
  delayMicroseconds(1);
  digitalWriteFast(TSL1401_SI, LOW);
  delayMicroseconds(1);
  digitalWriteFast(TSL1401_CLK, LOW);
  delayMicroseconds(1);

  // Clkin 17 more clks
  for(int i=0;i<17;i++) {
    digitalWriteFast(TSL1401_CLK, HIGH);
    delayMicroseconds(1);
    digitalWriteFast(TSL1401_CLK, LOW);  
    delayMicroseconds(1);  
  }  
  
  noInterrupts();
  TSL1401_ExposeStarted = 0;
  TSL1401_ExposeStopped = 0;
  interrupts();
}

// Step 2: Triggered by interrupt
void TSL1401_StartExpose() {
  if(TSL1401_ExposeStarted == 0) {
    TSL1401_ExposeStarted = 1;
    // Clkin 115 more clks
    for(int i=0;i<115;i++) {
      digitalWriteFast(TSL1401_CLK, HIGH);
      delayMicroseconds(1);
      digitalWriteFast(TSL1401_CLK, LOW);  
      delayMicroseconds(1);  
    }
  }
}

// Step 3: Triggered by interrupt 
void TSL1401_StopExpose() {
  if(TSL1401_ExposeStopped == 0) {
    // Clkin SI signal
    digitalWriteFast(TSL1401_SI, HIGH);
    delayMicroseconds(1);
    digitalWriteFast(TSL1401_CLK, HIGH);
    delayMicroseconds(1);
    digitalWriteFast(TSL1401_SI, LOW);
    delayMicroseconds(1);
    digitalWriteFast(TSL1401_CLK, LOW); 
    delayMicroseconds(1);
    TSL1401_ExposeStopped = 1; 
  }
}

// Step 4: can be started in main loop if TSL1401_ExposeStopped == 1
void TSL1401_ReadLine() {
  uint16_t valadc;
  int16_t  valmapped;
  uint8_t  val8;

  adc->startSingleRead(TSL1401_OUT);
  TSL1401_valmax = 0;
  TSL1401_valmin = 65535u;
  
  // Read analog
  for(int i=0;i<130;i++) {
    while(!adc->isComplete()) ;
    valadc = adc->readSingle();
    digitalWriteFast(TSL1401_CLK, HIGH);
    delayMicroseconds(1);
    adc->startSingleRead(TSL1401_OUT);
    
    TSL1401_buf16[i] = valadc;

    if(valadc > TSL1401_valmax) TSL1401_valmax = valadc;
    if(valadc < TSL1401_valmin) TSL1401_valmin = valadc;
    
    valmapped = map(valadc,TSL1401_avgmin,TSL1401_avgmax,MINBRIGHT,255);
    if(valmapped > 255) valmapped = 255;
    if(valmapped < 0) valmapped = 0;
    val8 = valmapped;

    TSL1401_buf8[i] = val8;
    
    if(val8 == 255) val8 = 254;
    Tx2_buf[i] = val8;
    digitalWriteFast(TSL1401_CLK, LOW); 
    delayMicroseconds(1);
  } 
  TSL1401_avgmin = 0.99 * TSL1401_avgmin + 0.01 * (float)TSL1401_valmin; 
  TSL1401_avgmax = 0.99 * TSL1401_avgmax + 0.01 * (float)TSL1401_valmax;
}

// Step 5: Put the line in dither buf and dither one line
void DitherOneLine() {
  int i, x;
  uint16_t p1, p2;
  uint8_t *pDB = &DitherBuf[DitherLineIn][0];
  const uint8_t DitherKernelElements = sizeof(DitherKernel) / sizeof(DitherKernel[0]);
  uint8_t bitCount, buildByte;
  
  // put pixel interpolated in dither buffer
  for(i=0; i<128; i++) {
    p1 = TSL1401_buf8[i];
    p2 = TSL1401_buf8[i+1];
    *pDB++ = (uint8_t)p1;
    *pDB++ = (uint8_t)((p1*2 + p2)/3);
    *pDB++ = (uint8_t)((p1 + p2*2)/3);
  }  
  
  uint8_t *pDBIdx[3] = { &DitherBuf[(DitherLineIn-2u)&DITHER_BUF_LINE_MASK][0], 
                         &DitherBuf[(DitherLineIn-1u)&DITHER_BUF_LINE_MASK][0],
                         &DitherBuf[DitherLineIn][0] };
  //y = (DitherLineIn-2u)&DITHER_BUF_LINE_MASK;
  //Serial1.print(" -");
  //Serial1.print(y);
  //Serial1.print(" - ");
  //Serial1.print(DitherLineIn);
  //Serial1.print(" - ");
  
  uint8_t *pBitBuf = &DitheredBuf[DitheredLinesIn][0];
  buildByte = 0; 
  bitCount = 0;
  for(x=0; x<DITHER_WIDTH; x++) {
    int bright = pDBIdx[0][x];
    int err;
    if(bright <= 127) {
      pDBIdx[0][x] = 0;
      err = bright;
      buildByte |= 1;
    } else {
      pDBIdx[0][x] = 255;
      err = bright-255;
    }
    
    // build also bit-array
    bitCount++;
    if(bitCount >= 8) {
      *pBitBuf++ = buildByte;
      buildByte = 0;
      bitCount = 0;
    } else {
      buildByte = buildByte << 1;
    }
    
    for(i=0; i<DitherKernelElements; i++) {
      int x2 = x + DitherKernel[i][0];
      int y2 = DitherKernel[i][1];
      uint8_t facti =  DitherKernel[i][2];
      if ((x2 < DITHER_WIDTH) && (x2 > 0)) {
        bright = pDBIdx[y2][x2];
        bright += (err * facti) / DITHER_KERNEL_DIVIDER;
        bright = constrain(bright, 0, 255);
        pDBIdx[y2][x2] = (uint8_t)bright;
      }
    }      
  }
  
  // Debug-Output
 // for(i=0; i<DITHER_WIDTH; i++) {
 //   Tx1_buf[i] = pDBIdx[0][i];
 //   if(Tx1_buf[i] == 255) Tx1_buf[i] = 254;
 // }  
  
  DitherLineIn++;
  if(DitherLineIn >= DITHER_BUF_LINES) DitherLineIn = 0;
  
  DitheredLinesIn++;
  if(DitheredLinesIn >= DITHERED_BUF_LINES) DitheredLinesIn = 0;
  
  DitheredLines++;
}  



void setup() {
  // main output for fast data streaming through Teensy's USB 
  Serial.begin(115200);

  // for debug terminal (needs extra USB-seriel converter)
  pinMode(0, INPUT_PULLUP);
  Serial1.setRX(0);
  Serial1.setTX(1);
  Serial1.begin(115200,SERIAL_8N1);

  // Connection to thermal printer, thermal printers baud rate changed to 57600 via Windows-Software from somewhere
  // CTS line must be connected to not overflow the printer
  pinMode(9, INPUT_PULLUP);
  Serial2.setRX(9);
  Serial2.setTX(10);  
  //pinMode(23, INPUT_PULLUP); // CTS
  Serial2.begin(57600, SERIAL_8N1);
  Serial2.attachCts(23);
  
  pinMode(TSL1401_SI, OUTPUT);
  pinMode(TSL1401_CLK, OUTPUT);
  pinMode(TSL1401_OUT, INPUT);
  pinMode(SHUTTER_BUT, INPUT_PULLUP);
  pinMode(SELECT1, INPUT_PULLUP);
  pinMode(SELECT2, INPUT_PULLUP);
  pinMode(EXTRA_BUT, INPUT_PULLUP);
  pinMode(VBAT, INPUT);  
  
  pinMode(A11, OUTPUT);
  digitalWrite(A11, LOW);

  adc->setReference(ADC_REFERENCE::REF_1V2, ADC_0);
  adc->setAveraging(0, ADC_0); // set number of averages 0,4,8,16,32
  adc->setResolution(16, ADC_0); // set bits of resolution

  // it can be any of the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
  // see the documentation for more information
  // additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
  // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // change the conversion speed
  // it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed  

  Serial2.write(255);
  delay(50);
  Serial2.write(27);  // Sleep off (important!)
  Serial2.write('8');
  Serial2.write(0);
  Serial2.write(0);   
  
  Serial2.write(27);  // Init command
  Serial2.write('@'); 
  delay(50);
  
  Serial2.write(27);  // Esc 7 (print settings) Heating dots, heat time, heat interval
  Serial2.write('7');
  Serial2.write(3); Serial2.write(100); Serial2.write(250);
 
  Serial2.write(18); Serial2.write('#'); Serial2.write((2 << 5) | 10);  
  Serial2.write(29); Serial2.write('a'); Serial2.write(1 << 5);
  
  
  Serial2.println(F("LineCamPrinter Ready. \n\n"));
  Serial1.println(F("LineCamPrinter Ready. \n\n"));
 // adc->enablePGA(1, ADC_0); // Gain 1, 2, 4, 8, 16, 32 or 64
//  adc->printError();
  TSL1401_Init(shutter_time, 10000ul);
}

uint8_t Convert[] = " .:-=+*#%@";

void loop() {
  uint32_t tsDur;

 if(TSL1401_ExposeStopped) {
    TSL1401_ReadLine();
    
    Tx1_buf[DITHER_WIDTH] = 255;
    TSL1401_PrepareExpose(); 
    
    if(digitalRead(SHUTTER_BUT)==0) {
      tsDur = micros();
      VBatTs = millis();
      DitherOneLine();    
      
        // Debug-Output
      //  for(int i=0; i<DITHER_WIDTH; i++) {
      //    Tx1_buf[i] = (DitheredBuf[DitheredLinesOut][i/8] & (1<<(7-(i%8))))?0:254;
      //    //if(Tx1_buf[i] == 255) Tx1_buf[i] = 254;
      //  } 
      Tx2_buf[129] = 255;
      //Serial.write(Tx2_buf, 130);         // Output to processing here
      //Serial.write(Tx1_buf, DITHER_WIDTH+1); 
      
      
      int line_count = 1;
      
      if(DitheredLines >= line_count) {
        //printer.printBitmap(DITHER_WIDTH,line_count,&DitheredBuf[DitheredLinesOut][0],false);
      
      
        Serial2.write(18);
        Serial2.write('V');
        Serial2.write(line_count);
        Serial2.write(0);
        Serial2.write(&DitheredBuf[DitheredLinesOut][0], 48*line_count);
      
      
        DitheredLines -= line_count;
        DitheredLinesOut+=line_count;
        if(DitheredLinesOut >= DITHERED_BUF_LINES) DitheredLinesOut -= DITHERED_BUF_LINES;
      }

      tsDur = micros() - tsDur;
      Serial1.println(tsDur);
    }
    

 }
 
 serialParser();

 SelectValue = (digitalRead(SELECT1)?0:1) | (digitalRead(SELECT2)?0:2);
 if(SelectValue != SelectValueOld) {
   const uint32_t ShutterTimes[4] = {20,2000,20000};
   Serial1.print("Sel: ");
   Serial1.println(SelectValue);
   //if(TSL1401_avgmax >= 65000.0) shutter_time /= 2;
   //if(TSL1401_avgmax < 8000.0) shutter_time *= 2;
   shutter_time = ShutterTimes[SelectValue & 3];
   if(shutter_time == 0) shutter_time = 1;
   Serial1.print("Shutter: ");
   Serial1.println(shutter_time);
   delay(500);
   if(SelectValue == 0) TSL1401_Init(shutter_time, 5000ul); // old  , 40000ul
   if(SelectValue == 1) TSL1401_Init(shutter_time, 10000ul); // old: , 20000ul
   if(SelectValue == 2) TSL1401_Init(shutter_time, 10000ul);  
   SelectValueOld = SelectValue;
 }
 
 if((uint32_t)((uint32_t)millis() - (uint32_t)VBatTs) > 30000) {
   VBatTs = millis();
   VBat = (8.17 / 40623.0) * adc->analogRead(VBAT);
   if(VBat < 6.4) {
     Serial2.print("VBat = ");
     Serial2.println(VBat);   
   }
 }
 
 if(digitalRead(EXTRA_BUT) == 0) {
   if(ExtraButStatus == 0) ExtraButTs = millis();
   ExtraButStatus = 1;
 } else {
   if(ExtraButStatus == 1) {
     if((uint32_t)((uint32_t)millis() - (uint32_t)ExtraButTs) > 2000) {
       
     } else if((uint32_t)((uint32_t)millis() - (uint32_t)ExtraButTs) > 20) {
       Serial2.write(29); Serial2.write('!'); Serial2.write(0);
       Serial2.println("HACKFFM.DE LineCamPrinter\nby Lutz Lisseck 2019\n\n\n");
     } 
   }     
   ExtraButStatus = 0;
 }  
   
 
}

uint8_t PrinterPar_N1 = 3;
uint8_t PrinterPar_N2 = 100;
uint8_t PrinterPar_N3 = 250;

void serialParser() {
  static char cmd[64];
  static byte charCount = 0;

  int r;

  if(Serial1.available()) {
    // if any char in serial buffer available then do the parsing

    char c;
    c = Serial1.read(); // read one char from serial buffer

    if((c==8) && (charCount>0)) charCount--; // backspace

    if(c>=32) { // char in num char range then add char to cmd array
      cmd[charCount] = c;
      charCount++;
    }

    if((c==0x0D) || (c==0x0A) || (charCount>60) ) {
      // if the char is NL(New Line 0x0A) 
      // or CR (carriage return 0x0D) 
      // or cmd array buffer limit reached
      // parse the cmd buffer
      cmd[charCount]=0; // clear the last char in cmd buffer
      
      if(charCount>=1) { // prevent empty cmd buffer parsing

        switch(cmd[0]) {
          case '?':
            // show command listup
            Serial1.println(F("LineCamPrinter."));
            break;
      
          case 'p':
            //printerTest(); 
            break;       
            
          case 'f':
            TSL1401_Init(2000, 10000ul);
            Serial1.println("Fast exposure 100 Hz");
            break;
            
          case 'm':
            TSL1401_Init(2000, 20000ul);
            Serial1.println("Medium exposure 50 Hz");
            break;            
            
          case 's':
            TSL1401_Init(2000, 40000ul);
            Serial1.println("Slow exposure 25 Hz");
            break;   
            
          case '1':
          case '2':
          case '3':          
            if(charCount>=2) {
              short xx; 
              r = sscanf(&cmd[1],"%hd",&xx);
              if(r>=1) {
                switch(cmd[0]) {
                  case '1':
                    Serial1.print("PrinterPar_N1 = ");
                    PrinterPar_N1 = xx;
                    Serial1.println(PrinterPar_N1);
                    break;  
                  case '2':
                    Serial1.print("PrinterPar_N2 = ");
                    PrinterPar_N2 = xx;
                    Serial1.println(PrinterPar_N2);
                    break;
                  case '3':
                    Serial1.print("PrinterPar_N3 = ");
                    PrinterPar_N3 = xx;
                    Serial1.println(PrinterPar_N3);
                    break;                    
                }
                setPrinterSettings();
              }
            }
            break;      

          case 'S':          
            if(charCount>=2) {
              int xx; 
              r = sscanf(&cmd[1],"%d",&xx);
              if(r>=1) {
                    Serial1.print("shutter_time = ");
                    shutter_time = xx;
                    Serial1.println(shutter_time);
              }
            }
            break;            

          case 'v': // 8.17V = 40623
            {
              int vbatadc = adc->analogRead(VBAT);
              Serial1.println(vbatadc);
              VBat = (8.17 / 40623.0) * vbatadc;
              Serial1.print("VBat = ");
              Serial1.println(VBat);  
            }              
            break;            
          
          case 'e':
            Serial1.print("valmax: ");
            Serial1.print(TSL1401_valmax);
            Serial1.print(", valmin: ");
            Serial1.print(TSL1401_valmin);
            Serial1.print(", avgmax: ");
            Serial1.print(TSL1401_avgmax);
            Serial1.print(", avgmin: ");
            Serial1.println(TSL1401_avgmin);
            break;            
          
          default:
            Serial1.println("hae?\a");
            break;

        }
      }
      charCount = 0;
      Serial1.print(">");
    }
  }
}

void setPrinterSettings() {
  Serial2.write(27);
  Serial2.write('7');
  Serial2.write(PrinterPar_N1);
  Serial2.write(PrinterPar_N2);
  Serial2.write(PrinterPar_N3);
  
  Serial2.print("ESC 7 ");
  Serial2.print(PrinterPar_N1);
  Serial2.print(" ");
  Serial2.print(PrinterPar_N2);
  Serial2.print(" ");  
  Serial2.println(PrinterPar_N3);
}


