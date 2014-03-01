/*
Code by David Wilde M0WID
Original VFO code by Richard Visokey AD7C - www.ad7c.com
Rotary library also by someone else.

//**  The code for the bar graph was copied from  TF3LJ website
//**  It relies on a direct swipe from the AVRLIB lcd.c/h.
//**            Therefore see the AVRLIB copyright notice.
//** 
//**  TF3LJ states:  The essentials for bargraph display have been copied and improved/adapted to
//**  my own taste, including several different customized bargraph display styles.
//**
//**  Peak Bar (sticky bar) indicator added as an option.
//**
//**  Initial version.: 2009-09-08, Loftur Jonasson, TF3LJ
//**
//**  Last update to this file: 2013-09-13, Loftur Jonasson, TF3LJ / VE2LJX


Revision 0.03 - March 1st, 2014
ToDo:
    Split out some functions to a lib (bargraph, dds control, dB calcs)



Some values are stored in EEPROM:  ToDo - make this a complex data type so self documenting

Freq 0-6 - could be improved!
Zero Adjust 7,8
Presets 9-PRESETS*3*4 - currently 144 bytes
Attenuator - 200
powerCalPoints 202-217

*/

// Include the library code
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <rotary.h>
#include <EEPROM.h>

//Setup some items
#define DDS 9850
#define PRESETS 12
//#define DDS 9851
#define W_CLK 11      // Pin 8 - connect to AD9850/1 module word load clock pin (CLK)
#define FQ_UD 12      // Pin 9 - connect to freq update pin (FQ)
#define DATA 10       // Pin 10 - connect to serial data load pin (DATA)
#define RESET 13      // Pin 11 - connect to reset pin (RST)
#define R_BUTTON 4    // Rotary encoder button
#define M_BUTTON 5    // Button for Menu access
#define Vin1 A0         // V1 analogue input (Ground to input from DDS)
#define Vin2 A2         // V2 analogue input (voltage across load resistor - measures current)
#define Vin3 A1         // V3 analogue input (Ground to mid point of branch with antenna - voltage across antenna)
#define Vin4 A3         // V4 analogue input - signal AD8307 log amp, or another diode detector, used for scalar analyser
#define OpAmpGain 1.50  // Gain of Op amp used for reading the V1/V3 values = 15k/10k
#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }
#define F_MAX 30000000  // Max frequency limit
#define F_MIN 1000000   // Min Frequency limit
#define PEP_BUFFER 100  // Size of PEP buffer.  Determines decay time for PEP values, based on loop scan time

// progress bar defines
#define PROGRESSPIXELS_PER_CHAR	6

// custom LCD characters for bargraph
const uint8_t __attribute__ ((progmem)) LcdCustomChar[][8] = {
	{0x00, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x00}, // 0. 0/5 full progress block
	{0x00, 0x10, 0x10, 0x15, 0x10, 0x10, 0x00, 0x00}, // 1. 1/5 full progress block
	{0x00, 0x18, 0x18, 0x1d, 0x18, 0x18, 0x00, 0x00}, // 2. 2/5 full progress block
	{0x00, 0x1c, 0x1c, 0x1d, 0x1C, 0x1c, 0x00, 0x00}, // 3. 3/5 full progress block
	{0x00, 0x1e, 0x1e, 0x1E, 0x1E, 0x1e, 0x00, 0x00}, // 4. 4/5 full progress block
	{0x00, 0x1f, 0x1f, 0x1F, 0x1F, 0x1f, 0x00, 0x00}, // 5. 5/5 full progress block
	{0x06, 0x06, 0x06, 0x16, 0x06, 0x06, 0x06, 0x06} // 6. Peak Bar
};

Rotary r = Rotary(2,3); // sets the pins the rotary encoder uses.  Must be interrupt pins.

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
int_fast32_t rx=7200000;          // Starting frequency of VFO
int_fast32_t rx2=1;               // variable to hold the updated frequency
int_fast32_t increment = 10000;    // starting VFO update increment in HZ.

int_fast32_t startF=7000000;      // Starting point of freq scan 
int_fast32_t endF=7200000;        // End point of freq scan
int_fast32_t scanInc=10000;       // Increments during scan
int_fast32_t scanPresets [PRESETS][3] =   {{ 3500000, 3800000, 50000},
                                    { 7000000, 7200000, 50000},
                                    {10100000,10150000, 20000},
                                    {14000000,14350000, 50000},
                                    {18070000,18170000, 20000},
                                    {21000000,21450000, 50000},
                                    {24800000,24990000, 20000},
                                    {28000000,29700000, 50000},
                                    {0,0, 20000},
                                    {0,0, 20000},
                                    {0,0, 20000},
                                    {0,0, 20000}};
int powerCalPoints [2][2] = {{270,800},
                             {170,400}};        // Storage for the power calibration points
                                  // First index selects the calibration point
                                  // Second index selects the values (0 = dBm*10, 1 = corresponding ADC4 value)
                                    
int scanSamples=3;                // number of reading to take at each freq during the scan
int scanSampleCount=0;            // counter for the samples

bool scanning = 0;                // used to record if in scanning state
bool scanComplete = 0;
bool serialOn=0;                  // flag used to turn osc on/off via serial port
bool oscOn=0;                     // used to indicate if oscillator is to be turned on or not
bool oscOn2=1;                    // previous oscillator state

bool rButtonPress = 0, mButtonPress = 0;      // state of rotary encoder button and Left Button
bool rButtonPress1 = 0, mButtonPress1 = 0;    // state of rotary encoder button and Left Button last time
bool mButtonLong=0;  // menu button long press pulse
bool mButtonShort=0; // menu button short press pulse
bool rButtonLong=0;  // Rotary coder long button press pulse
bool rButtonShort=0; // Rodary coder short button pulse
//bool mButtonLongMem;
//bool rButtonLongMem;
int rButtonCount, mButtonCount=0;    // count how long the button is pressed to determine if long or short press


int  hertzPosition = 4;             // position of cursor for adjusting freq
byte ones,tens,hundreds,thousands,tenthousands,hundredthousands,millions ;  //Placeholders
String freq; // string to hold the frequency
int_fast32_t timepassed = millis(); // int to hold the arduino miilis since startup
int memstatus = 1;  // value to notify if memory is current or old. 0=old, 1=current.
//float ARef = 5.0;
float ARef = 1.1;

//  Variables for Menu

bool inMenu=0;      // 1 if in the menus, otherwise 0
bool testOn=0;      // 1 if oscillator on via LButton
int menuPos=0;      // used so returns to previous menu position when re-entering the menu
int menuPos2=0;     // used to determine if menu position has changed
//PROGMEM prog_char menu_00[]="Mode";
//PROGMEM prog_char menu_01[]="Start Scan";
//PROGMEM prog_char menu_02[]="End Scan";
int menuNo=8;  //  max menu entry
int menuLevel=0, menuLevel2=0;   // determines if encoder moves between menu items(0) or changes value (1)
int menuVal=0, menuVal2=99, menuMax=9, menuMin=0;

int mode=0;         // 0=SWR/R; 1=V1/V2; 2=Scan ; 3=Power; 4=Power Calibration point 1; 5=Power Calibration Point 2
int pDisplay=1;      // select display type when in power mode
int presetNo=1;      // selected preset for scan limits

// PROGMEM const char *menu_item[]={menu_00, menu_01, menu_02};

int ForceFreq = 0;  // Change this to 0 after you upload and run a working sketch to activate the EEPROM memory.  YOU MUST PUT THIS BACK TO 0 AND UPLOAD THE SKETCH AGAIN AFTER STARTING FREQUENCY IS SET!

double n=0;              //  used to count intervals between analogue updates
int_fast32_t ADC1, ADC2, ADC3, ADC4;    //  values read from Analog inputs relating to V1, V2 and V3 and V4 from AD8307
float R, X;              //  calculated resistance and reactance of load
float SWR;               //  calculated SWR of load against 50R
float dBm;               //  Calculated power in dBm, corrected for attenuator value
//int dBmx10;              //  dBm to 1 decimal place, *10  
int attenuator;          //  Value of attenuator fitted * 10 in dB
float power;             //  Calculated power in watts or milliwatts, corrected for attenuator value
int16_t pep_dBm;
float pepPower;           //  store for peak power
//unsigned peakDelay=15000;     //  interval between peak resets (scan cycles)
int ZeroAdjust=0;        //  offset to correct for minor errors in op amp gains and different R values - difference between ADC1 and ADC3 with no load
int V2Adjust=0;          //  offset to correct for minor component tolerance errors - difference between ADC2 and ADC3 with 50R load

void setup() {
  pinMode(R_BUTTON,INPUT); // Connect to a button that goes to GND on push
  digitalWrite(R_BUTTON,HIGH);  //  Turn on pull up resistor
  pinMode(M_BUTTON,INPUT); // Connect to a button that goes to GND on push
  digitalWrite(M_BUTTON,HIGH);  //  Turn on pull up resistor
  pinMode(Vin1, INPUT);
  digitalWrite(Vin1,LOW);
  pinMode(Vin2, INPUT);
  digitalWrite(Vin2,LOW);
  pinMode(Vin3, INPUT);
  digitalWrite(Vin3,LOW);
  pinMode(Vin4, INPUT);
  digitalWrite(Vin4,LOW);
 
  // initialize the serial communication:
  Serial.begin(9600);
  
  if (ARef == 1.1) {
    analogReference(INTERNAL);   // set 1.1 v internal reference
  }
  else {
    analogReference(DEFAULT);    // Vcc
  }
  
  lcd.begin(16, 2);            // 1602 LCD display
  //  send down custom chars for bargraph to the display
  lcd_bargraph_init();
  
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();                       // Enable interrupts
  
  // set up the ports for the AD9850/1
  pinMode(FQ_UD, OUTPUT);
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT); 
  pulseHigh(RESET);
  pulseHigh(W_CLK);
  pulseHigh(FQ_UD);  // this pulse enables serial mode on the AD9850/51 - Datasheet page 12.
  lcd.setCursor(hertzPosition,0);   
  
  // Load the stored frequency  
  if (ForceFreq == 0) {
    Serial.println("Reading EEPROM");
    freq = String(EEPROM.read(0))+String(EEPROM.read(1))+String(EEPROM.read(2))+String(EEPROM.read(3))+String(EEPROM.read(4))+String(EEPROM.read(5))+String(EEPROM.read(6));
    rx = freq.toInt();
    // read the saved zero adjust value.  Can be negative!
    ZeroAdjust=read16bits(7);
    readPresets();            //  get the scan presets from memory
    readCalPoints();          //  get the Power calibration points from memory - locations 50-57
    attenuator = read16bits(200);
  }
  Serial.println("VFO Serial Output");
}


void loop() {  // MAIN LOOP  **********************************************************************************

  byte commandChar=0;             //  received character
  
    //  test if the oscillator is to be on or off
  oscOn=((testOn)||(scanning)||(serialOn))&&(!inMenu);
  if ((rx != rx2)||(oscOn!=oscOn2)) {                    // Frequency has changed
        rx=checkFreq(rx);            // check in bounds
        showDDSFreq();              //  update frequency on the LCD, but not to the serial port
        if (oscOn) {
          sendFrequency(rx);           //  Send to DDS (AD9851)
        }
        else {
          sendFrequency(0);
        }
        rx2 = rx;
        oscOn2=oscOn;
        n=0;                         // reset the counter to delay send of data for a time after a change
  }

  if ((mButtonShort) || (rButtonShort)) delay(100);    // debounce short button presses
  
  // test the Menu button, and set flags if pressed for long or short for one scan
  mButtonPress1=mButtonPress;   // store state of buttons and flags last time round the loop
  rButtonPress1=rButtonPress;
  //mButtonLongMem=mButtonLong;
  //rButtonLongMem=rButtonLong;
  rButtonPress = !(digitalRead(R_BUTTON));    // NO Button
  mButtonPress = digitalRead(M_BUTTON);       // NC button
  
  // test menu button
  if (mButtonPress) {    // NC button, so goes high if pressed
    mButtonCount++;
    delay(100);
  }
  else {
    mButtonCount=0;
  }
  mButtonShort = (!mButtonPress && mButtonPress1 && (mButtonCount<20)); // True if Button was pressed last time, but count for long button not reached
  mButtonLong = (mButtonCount == 20);  // True for one scan round loop when mButtonCount = 20

  // test rotary encoder button
  if (rButtonPress) {    // NO button, so goes low if pressed
    rButtonCount++;
    delay(100);
  }
  else {
    rButtonCount=0;
  }
  rButtonShort = (!rButtonPress && rButtonPress1 && (rButtonCount<20)); // True if Button not pressed, was pressed last time, but count for long button not reached
  rButtonLong = (rButtonCount== 20);  // True for one scan round loop when mButtonCount  = 20
  
  if (inMenu) {
      // reset scanning flags
      scanning=0;  
      scanComplete=0;
      serialOn=0;
      testOn=0;
      
      //  display menu text if position has changed
      if ((menuPos!=menuPos2)||(menuVal!=menuVal2)) {
        showMenu();
        menuPos2=menuPos;
        menuVal2=menuVal;
      }

      //  press of encoder button changes from changing menu (line 0) to changing value (line 1)
      if(rButtonShort) {
            //Serial.println("menu RButton Press");
            if (menuLevel) {
                menuLevel=0;
                if (menuPos==5) { // Preset changed
                   startF=scanPresets[presetNo-1][0];
                   endF=scanPresets[presetNo-1][1];
                   scanInc=scanPresets[presetNo-1][2];
                }
                menuVal=menuPos;
            }
            else { menuLevel=1;};
            showMenu();
      };
    
      if (mButtonLong) {    
      //  Exit Menu
            inMenu=0;
            showDDSFreq();
            //  save the zeroadjust and preset values in case they have changed
            //Serial.print("Saving values on menu exit");
            storeZeroAdjust();
            storePresets();
            storeCalPoints();
            store16bits(200,attenuator);

//            while (digitalRead(L_BUTTON)==HIGH) {  // wait until button released
//            }
//            delay(250);  // debounce
//            LButtonState=LOW;
      }
  }
  else {    // not in menu
      if(rButtonShort) {
        if (mode<2) {
            setincrement();        
        }
      };

      if (rButtonLong) {
        if (mode==4) {                      //  Power cal point 1
          saveCalPoint(1);
        }
        else if (mode==5) {                      //  Power cal point 2
          saveCalPoint(2);
        }
      }  

   
      if (mButtonLong) {
          //  Enter Menu
          //  Rotary Encoder now changes the menu selection until the encoder button is pressed
          //  pressing the M_BUTTON again exists the menu
          storeMEM();  // save to memory in case < 2 secs have elapsed since it was changed
          inMenu=1;
          menuLevel=0;
          showMenu();
          testOn=0;
      }
      
      if (mButtonShort) {  //  Short Menu Button press
        if ((mode==0)||(mode==1)) {                //  SWR or ADC modes
          testOn=!testOn;                      // toggle the oscillator
        }
      }
         
      
        // Write the frequency to memory if not stored and 2 seconds have passed since the last frequency change.
      if(memstatus == 0){   
          if ((timepassed+2000 < millis())&&(!scanning)) storeMEM();
      }

      
      // write the V1 and V2 values to LCD, but only every so many scans and only if not in the menus
      if ((n>15000)||(mode==3)) {
          ADC1 = analogRead(Vin1);
          ADC2 = analogRead(Vin2);
          ADC3 = analogRead(Vin3);
          ADC4 = analogRead(Vin4);
        //Serial.print("4 ");        //~~~~~~~~~~~~~Debug~~~~~~~~~~~~~~~~~
        //Serial.println(ADC4);        //~~~~~~~~~~~~~Debug~~~~~~~~~~~~~~~~~
          lcd.setCursor(0,1);
          lcd.noCursor();
          switch (mode) {
            case 0:  // Show SWR and R
              showSWR();
              break;
            case 1:  //Show ADC values
              lcd.print("A");
              lcdPrintInt(ADC1,4);
              lcd.print(",");
              lcdPrintInt(ADC2,4);
              lcd.print(",");
              lcdPrintInt(ADC3,4);
              break;
            case 2:  // Show Scan progress as well as SWR and R
              showSWR();
              lcd.setCursor(0,0);
              lcdPrintInt(scanSampleCount,2);
              break;
            case 3:  // Power - display ADC value and power in dBm and watts
              showPower();
              break;
            case 4:  // Power calibration point 1
              // ToDo: Display dBm value and allow it to be changed with rotary coder
              // Display ADC4
              // Long press of encoder button stores the value
              showCal(1);
              break;       
            case 5:  // Power calibration point 2
              // Todo: Display dBm value and allow it to be changed with rotary coder
              // Display ADC4
              // Long press of encoder button stores the value
              showCal(2);
              break;       
            default:
              lcd.print("                ");
              break;
          }
          lcd.setCursor(hertzPosition,0);  //  put the cursor back under the frequency digit to be changed
          lcd.cursor();
         
          if (oscOn) printFreq();          //  send the data to the serial port
          n=0;
          
          if (scanning) {
            scanSampleCount++;
            lcd.setCursor(15,0);
            lcd.print(scanSampleCount);    //  display progress
            //  increment frequency by scanInc every scanSamples
            if (scanSampleCount > scanSamples) {
              rx=rx+scanInc;
              scanSampleCount=0;
              if (rx>endF) {    // reached end of scan
                  scanComplete=1;
                  Serial.println("X");    // tell PC that scan is complete
                  scanning=0;
              }
            }
          }
      }
      else {
          n++;
      }

  }  // not in Menu

  // Check if any serial commands received 
  if (Serial.available()>0) {
    //    Serial.write(Serial.read());  
    // read the char
    commandChar = Serial.read();
    switch(commandChar) {
      case 'A':    // set up cal point 1 ADC value - usually higher power than point 2
          //  Parameter passed in is dBm value (into port, so eg 27dBm transmitter, 40db attenuator, -13dBm to port)
          powerCalPoints[0][0]=Serial.parseFloat();
          ADC4 = analogRead(Vin4);
          powerCalPoints[0][1]=ADC4;
          //  Echo back the results
          Serial.print("A ");
          Serial.print(powerCalPoints[0][0]);
         printDelim();
          Serial.println(powerCalPoints[0][1]);
          storeCalPoints();
          break;
      case 'B':    // set up cal point 2 ADC value - usually lower power than point 2
          powerCalPoints[1][0]=Serial.parseFloat();
          ADC4 = analogRead(Vin4);
          powerCalPoints[1][1]=ADC4;
          //  Echo back the results
          Serial.print("B ");
          Serial.print(powerCalPoints[1][0]);
          printDelim();
          Serial.println(powerCalPoints[1][1]);
          storeCalPoints();
          break;
      case 'C':
          V2Adjust=Serial.parseFloat();
          break;
      case 'D':    // 
          printPowerCalPoints();
          break;
      case 'F':
          rx=Serial.parseFloat();
          break;
      case 'H':
          endF=Serial.parseFloat();
          break;
      case 'I':
          scanInc=Serial.parseFloat();
          break;
      case 'L':
          startF=Serial.parseFloat();
          break;
      case 'M':          // Change mode
          mode=Serial.parseInt();
          break;
      case 'N':
          serialOn=0;    // Turn oscillator off
          scanning=0;
          scanSampleCount=0;
          break;
      case 'O':    // Letter O
          ZeroAdjust=Serial.parseFloat();
          break;
      case 'P':
          //  send all preset values to PC
          printPresets();
          break;
      case 'S':
          if (!inMenu) {
            scanning=1;    // start scan, but only if not in the menus
            rx=startF;
          }
          break;
      case 'T':    // 
          attenuator=Serial.parseFloat();
          break;
      case 'X':
          scanning=0;    // finish scan
          scanSampleCount=0;
          Serial.println("X");
          break;
      case 'Y':
          if ((!inMenu)&&(mode<3)) serialOn=1;    //  Turn oscillator on, but only if not in the menus and analyser mode
          break;
      default:
          Serial.println("serial default");
          break;
    }
  }     
            
  
  
}  // END of MAIN LOOP  ****************************************************************************************


ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result) {    
    if (inMenu) {
      if (result == DIR_CW){
        if (menuLevel==0) {
          menuPos++;
          if (menuPos>menuNo) {menuPos=0;};
        }
        else {  // menuLevel==1 - editing value
          if (menuVal<menuMax) menuVal++;
        }
      }
      else {
        if (menuLevel==0) {
          if (menuPos==0) {menuPos=menuNo;} else {menuPos--;};
        }
        else {  // menuLevel==1 - editing value
          if (menuVal>menuMin) menuVal--;
        }
      }
    }
    else {  // not in menu
      switch(mode) {
      case 0:  //SWR - changes frequency
        if (result == DIR_CW){rx=rx+increment;}
        else {rx=rx-increment;};       
        break;
      case 1:  // ADC - changes frequency
        if (result == DIR_CW){rx=rx+increment;}
        else {rx=rx-increment;};       
        break;
      case 2:  //Scan - does nothing
        break;
      case 3:  //Power - changes display type
        //  pDisplay has different values:
        //  1=ADC, dBm, Watts
        //  2=dBm, bar graph display
        //  3=Watts, bar graph
        //  4=Watts (Peak)
        if (result==DIR_CW) {
          pDisplay++;
          if (pDisplay>4) { pDisplay=1;};
        }
        else {
          pDisplay--;
          if (pDisplay<1) {pDisplay=4;};
        }
        break;
      default:
        break;
      }
    }
  }
}





//-----------------------------------------------------------------------------------------
// Initialize LCD for bargraph display - Load 6 custom bargraph symbols
//-----------------------------------------------------------------------------------------
void lcd_bargraph_init(void)
{

	for (uint8_t i=0; i<7; i++)
	{
		lcd.createChar(i,(uint8_t*)LcdCustomChar[i]);
	}
}

//-----------------------------------------------------------------------------------------
// Display Bargraph - including Peak Bar, if relevant
//
// "length" indicates length of bargraph in characters 
// (max 16 on a 16x2 display or max 20 on a 20x4 display)
//
// (each character consists of 6 bars, thereof only 5 visible)
//
// "maxprogress" indicates full scale (16 bit unsigned integer)
//
// "progress" shown as a proportion of "maxprogress"  (16 bit unsigned integer)
//
// if "prog_peak" (16 bit unsigned integer) is larger than "progress",
// then Peak Bar is shown in the middle of that character position
//-----------------------------------------------------------------------------------------
void lcdProgressBarPeak(uint16_t progress, uint16_t prog_peak, uint16_t maxprogress, uint8_t length)
{
	uint8_t i;
	uint16_t pixelprogress;
	uint8_t c;

	if (progress >= maxprogress) progress = maxprogress;	// Clamp the upper bound to prevent funky readings

	// draw a progress bar displaying (progress / maxprogress)
	// starting from the current cursor position
	// with a total length of "length" characters
	// ***note, LCD chars 0-6 must be programmed as the bar characters
	// char 0 = empty ... char 5 = full, char 6 = peak bar - disabled if maxprogress set as 0 (or lower than progress)

	// total pixel length of bargraph equals length*PROGRESSPIXELS_PER_CHAR;
	// pixel length of bar itself is
	pixelprogress = ((uint32_t)progress*(length*PROGRESSPIXELS_PER_CHAR)/maxprogress);
		
	// print exactly "length" characters
	for(i=0; i<length; i++)
	{
		// check if this is a full block, or partial or empty
		if( ((i*PROGRESSPIXELS_PER_CHAR)+PROGRESSPIXELS_PER_CHAR) > pixelprogress )
		{
			// this is a partial or empty block
			if( ((i*PROGRESSPIXELS_PER_CHAR)) > pixelprogress )
			{
				// If an otherwise empty block contains previous "Peak", then print peak char
				// If this function is not desired, simply set prog_peak at 0 (or as equal to progress)
				if(i == ((uint32_t)length * prog_peak)/maxprogress)
					c = 6;				
				// othwerwise this is an empty block
				// use space character?
				else
					c = 0;
			}
			else
			{
				// this is a partial block
				c = pixelprogress % PROGRESSPIXELS_PER_CHAR;
			}
		}
		else
		{
			// this is a full block
			c = 5;
		}
		
		// write character to display
		lcd.write(c);
	}
}



int_fast32_t checkFreq(int_fast32_t fc) {
  // if out of bounds then revert to previous frequency value
  if (fc >F_MAX){fc=F_MAX;}; // UPPER VFO LIMIT
  if (fc <F_MIN){fc=F_MIN;}; // LOWER VFO LIMIT
  return fc;
}


// frequency calc from datasheet page 8 = <sys clock> * <frequency tuning word>/2^32
void sendFrequency(double frequency) {
#if DDS==9850  
  int32_t freq = frequency * 4294967295/125000000;  // note 125 MHz clock on 9850.  You can make 'slight' tuning variations here by adjusting the clock frequency.
#else
  int32_t freq = frequency * 4294967295/180000000;  // note 180 MHz clock on 9851.  You can make 'slight' tuning variations here by adjusting the clock frequency.
#endif
  for (int b=0; b<4; b++, freq>>=8) {
    tfr_byte(freq & 0xFF);
  }
#if DDS==9850
  tfr_byte(0x000);   // Final control byte, all 0 for 9850 chip
#else
  tfr_byte(0x001);   // Final control byte, keep 6x set for 9851 chip
#endif
  pulseHigh(FQ_UD);  // Done!  Should see output if not in standby
}


// transfers a byte, a bit at a time, LSB first to the 9850/1 via serial DATA line
void tfr_byte(byte data)
{
  for (int i=0; i<8; i++, data>>=1) {
    digitalWrite(DATA, data & 0x01);
    pulseHigh(W_CLK);   //after each bit sent, CLK is pulsed high
  }
}

void setincrement(){
  if(increment == 10){increment = 100;  hertzPosition=7;}
  else if (increment == 100){increment = 1000;  hertzPosition=5;}
  else if (increment == 1000){increment = 10000;  hertzPosition=4;}
  else if (increment == 10000){increment = 100000;  hertzPosition=3;}
  else if (increment == 100000){increment = 1000000;  hertzPosition=1;}  
  else{increment = 10;  hertzPosition=8;};  
   lcd.cursor();
   lcd.setCursor(hertzPosition,0); 
   delay(250); // Adjust this delay to speed up/slow down the button menu scroll speed.
};

void lcdPrintInt(int i, int charNum) {
  //  print with right justify.  Always uses charNum space on the lcd
  //  No check to make sure value fits in charNum space, no check on charNum value
    if (i>=0) {
      if ((i<10000)&&(charNum>4)) lcd.print(" ");
      if ((i<1000)&&(charNum>3)) lcd.print(" ");
      if ((i<100)&&(charNum>2)) lcd.print(" ");
      if ((i<10)&&(charNum>1)) lcd.print(" ");
    }
    else {  //negative - to keep simple ignore possibility of negative numbers less than -999
      if ((i>-1000)&&(charNum>4)) lcd.print(" ");
      if ((i>-100)&&(charNum>3)) lcd.print(" ");
      if ((i>-10)&&(charNum>2)) lcd.print(" ");
      if ((i>-1)&&(charNum>1)) lcd.print(" ");
    }
    lcd.print(i);
}

void lcdPrintIntAsDecimal(int i, int charNum, int decimalPos) {
  //  routine to display a number stored as integer as a decimal
  //  split up into the integer and decimal parts
  int decimal, whole, mult, p, cNum;
  if (decimalPos == 0) {
    lcdPrintInt(i,charNum);
  }
  else {
    whole = i;
    mult=1;
    cNum=charNum;
    for (p=0; p<decimalPos; p++) {
      whole = whole/(10);
      mult=mult*10;
    }
    decimal=i % mult;
    if (decimal<0) {
      if (whole==0) lcd.print("-");
      decimal = decimal * -1;    //  Could be optimised, but easier to follow code
      cNum=cNum--;               // leave space for the - sign
    }
    lcdPrintInt(whole,cNum-decimalPos-1);
    lcd.print(".");
    lcdPrintInt(decimal,decimalPos);
  }  
}

void showSWR() {
  if (oscOn) { 
    calcRX();
    if (R>=50.0) {SWR=R/50.0;} else {SWR=50.0/R;};
    if (SWR > 99.0) SWR=99.9;  // limit to avoid display going wrong
    if (SWR<10.00) lcd.print(" ");
    lcd.print(SWR,2);
    lcd.print(",");
    if (R >=1000.0) R=999.9;
    if (R<1000.0) lcd.print(" ");
    if (R<100.0) lcd.print(" ");
    if (R<10.0) lcd.print(" ");
    lcd.print(R,1);
  }
  else {
    lcd.print("OFF             ");
  }
}

/* #############################################################################

Show Power

Also calculates PEP power

################################################################################*/

void showPower() {
  static uint16_t a=0;  // ring buffer counter
  static int16_t db_buff[PEP_BUFFER];
  int16_t dMax=-32767;   // set max to low value at start
  
  calcdBm();
  db_buff[a]=100*dBm;
  a++;
  if (a > 99) a=0;
  
  power = calcWatts(dBm);
  // store the power value into buffer used to calculate PEP values
  
  //   find max value in the buffer
  for (uint16_t x=0; x<PEP_BUFFER;x++) {
    dMax = max(dMax, db_buff[x]);
  }
  pepPower=calcWatts(float(dMax/100));
  
  lcd.setCursor(0,0);
  lcd.print("Power: ");
  lcd.print(dBm);
  lcd.print("dBm   ");
  lcd.setCursor(0,1);
  //  pDisplay has different values to select what is displayed on the second line:
  //  1=dBm on first line, ADC, Watts on second
  //  2=dBm, bar graph 0-5W display on second
  //  3=dBm bar graph 0-100W
  //  4=Watts (Peak)
  switch (pDisplay) {
    case 1:
      lcdPrintInt(ADC4, 4);
      lcd.print(" : ");
      if (power > 2000) {
        lcd.print(power/1000.0);
        lcd.print("W    ");
      }
      else {
        lcd.print(power);
        lcd.print("mW   ");
      }
      break;
    case 2:
      lcdProgressBarPeak(uint16_t(power),uint16_t(pepPower), 5000,16);
      break;
    case 3:
      lcdProgressBarPeak(uint16_t(power/1000),uint16_t(pepPower/1000),100,16);
      break;
    case 4:
      if (power > 2000) {
        lcd.print(power/1000.0);
        lcd.print("W  Pk: ");
      }
      else {
        lcd.print(power);
        lcd.print("mW Pk: ");
      }
      if (pepPower > 2000) {
        lcd.print(pepPower/1000.0);
        lcd.print("W ");
      }
      else {
        lcd.print(pepPower);
        lcd.print("mW ");
      }
      break;
    default:
      break;
  }
}


void showCal(int point) {
  //  Display the values for the selected power calibration point
  //  First line is the Point number, with the set power level in dBm (no allowance for attenuator!)
  //  Second line is the current setting and the current ADC4 value
  lcd.setCursor(0,0);
  lcd.print("Power Cal ");
  lcdPrintInt(point,1);
  lcd.print(" ");
  lcdPrintIntAsDecimal(powerCalPoints[point-1][0],4,1);
  lcd.print("dBm");
  lcd.setCursor(1,0);
  lcdPrintInt(powerCalPoints[point-1][1],4);
  lcd.print(", ");
  lcdPrintInt(ADC4,4);
}

void showDDSFreq(bool SerialOut){
    millions = int(rx/1000000);
    hundredthousands = ((rx/100000)%10);
    tenthousands = ((rx/10000)%10);
    thousands = ((rx/1000)%10);
    hundreds = ((rx/100)%10);
    tens = ((rx/10)%10);
    ones = ((rx/1)%10);
    showFreq(0,0,rx);
    lcd.print("hz  ");
    if (oscOn) {
        if(scanning) {lcd.print("S");} else {lcd.print(" I");}
    }
    else {lcd.print(" O");};
    lcd.setCursor(hertzPosition,0); //  position cursor under digit to be changed
    lcd.cursor();    // turn underline cursor on
 
    if (SerialOut) printFreq();      
    timepassed = millis();
    memstatus = 0; // Trigger memory write
};

void showDDSFreq(){
  //  overload allowing for optional paramater serial out.  If omitted then don't sent to serial port.
  showDDSFreq(0);
}



void showFreq(int lcdCol, int lcdRow, int_fast32_t f){
    //  This displays a frequency value at the desired cursor position  
    
    lcd.setCursor(lcdCol,lcdRow);
    if (f<10000000) lcd.print(" ");
    lcd.print(int(f/1000000));
    lcd.print(".");
    lcd.print((f/100000)%10);
    lcd.print((f/10000)%10);
    lcd.print((f/1000)%10);
    lcd.print(".");
    lcd.print((f/100)%10);
    lcd.print((f/10)%10);
    lcd.print((f/1)%10);
 };


void printFreq() {
        Serial.print("D ");
        Serial.print(rx);
        printDelim();
        Serial.print(ADC1);
        printDelim();
        Serial.print(ADC2);
        printDelim();
        Serial.print(ADC3);
        printDelim();
        Serial.println(ADC4);
}

void printDelim() {
  Serial.print(", ");
}


void calcRX() {
//  Calculate the resistance and reactance of the load from the input voltage(V1) and current voltage (V2)
//  V3 is the voltage across the load.  ADC1. ADC2 and ADC3 are the raw values from the A-D convertor
//  We need to be careful to avoid overflows as V3 approaches V1 in value, when the load is far from 50R
//  Also we measure RMS values of so cannot determine if it is -ve, so cannot determine sign of reactance X
//

float vR;
float diff;
float t1;
float adc1_2, adc2_2, adc3_2, vDiff;
 
 
//  Assume V1 is reading accurately, and correct ADC2 and 3 accordingly
//  Correction values from calibration spreadsheet and optimised for 50R load
//  Need to test if valid for other conditions
//  
float corrADC2;
float corrADC3;

corrADC2=(rx/-1000000.0+30.0+float(ADC2));
corrADC3 = ADC3-ZeroAdjust;

  if (ADC1==1023) {       //  Overflow, open circuit load
    R = 999.9;
    X = 0;
  }
  else if (corrADC2<10) {    //  low current - very high resistance or open circuit
    R = 999.9;
    X = 0;
  }
  else if (corrADC3<10) {    //   low output voltage, short circuit or very low R
    R = 0;
    X = 0;
  }
  else {             //   Calculate R
    //  Vr = (V1^2-V2^2-V3^2)/2*V2
    //  I = V2/51
    //  R = Vr/I = 51*(V1^2-V2^2-V3^2)/2*V2^2 = 25.5*(ADC1*ADC1-ADC2*ADC2-ADC3*ADC3)/(ADC2*ADC2)
    if (ADC1 > corrADC2+corrADC3) {
      // Impossible - assume pure resistance and calculate based on ADC3/ADC1
      t1 = float(ADC1)/(float(ADC1)/2.0+float(corrADC3));
      R = 51.0/(t1-1.0);
      X=0;
    }
    else {
      adc1_2 = ADC1*ADC1;
      adc2_2 = corrADC2*corrADC2;
      adc3_2 = corrADC3*corrADC3;
      vDiff = adc1_2-adc2_2-adc3_2;
      vR = vDiff/2*corrADC2;
      R = 25.5*float(vDiff)/float(adc2_2);
      X = sqrt(float(adc3_2)-vR*vR)*51/float(corrADC2);
      Serial.print("ADC1:");
      Serial.print(ADC1); printDelim(); Serial.println(adc1_2);
      Serial.print(ADC2); printDelim(); Serial.print(corrADC2); printDelim(); Serial.println(adc2_2);
      Serial.print(ADC3); printDelim(); Serial.print(corrADC3); printDelim(); Serial.println(adc3_2);
      Serial.println(vDiff);
    }
  }
}


void calcdBm() {
  // routine to calculate measdured power in dBm based on ADC4 value, calibration points and attenuator
  // dBm*10 = (Dc1-Dc2)*(Vx-Vc2)/(Vc1-Vc2) + Dc2
  // For V substitute ADC4 values for the relevant points, conversion to V cancels out
  float intdBmx10;
  float slope;
  //printPowerCalPoints();
  slope=float(powerCalPoints[0][0]-powerCalPoints[1][0])/float(powerCalPoints[0][1]-powerCalPoints[1][1]);
  //Serial.print ("Slope: ");
  //Serial.print (slope);
  intdBmx10 = slope*float(ADC4-powerCalPoints[1][1])+float(powerCalPoints[1][0]);
  //Serial.print (", intdBmx10: ");
  //Serial.print (intdBmx10);
  dBm=(intdBmx10 + float(attenuator))/10.0;
  //dBmx10=int(dBm/10.0);      //  May need to do some casting of variables
  //Serial.print (", dBmx10: ");
  //Serial.println (dBmx10);
}


void printPowerCalPoints() {
  Serial.print ("D1: ");
  Serial.print(powerCalPoints[0][0]);
  Serial.print (" D2: ");
  Serial.print(powerCalPoints[1][0]);
  Serial.print (" V1: ");
  Serial.print(powerCalPoints[0][1]);
  Serial.print (" V2: ");
  Serial.println(powerCalPoints[1][1]);
}  

float calcWatts(float d) {
  // routine to calculate the power in watts or milliwatts (maybe even microwatts)
  
   return pow(10.0,d/10.0);
  
}



void storeMEM(){
  //Write each frequency section to a EPROM slot.  Yes, it's cheating but it works!
   EEPROM.write(0,millions);
   EEPROM.write(1,hundredthousands);
   EEPROM.write(2,tenthousands);
   EEPROM.write(3,thousands);
   EEPROM.write(4,hundreds);       
   EEPROM.write(5,tens);
   EEPROM.write(6,ones);   
   memstatus = 1;  // Let program know memory has been written
   //Serial.println("Freq saved");
}


void storeZeroAdjust(){
  store16bits(7,ZeroAdjust);
}

void saveCalPoint(int p) {
  //  get current ADC4 value, put in the relevant position in the calibration point array, and save the array to EEPROM
  powerCalPoints[p-1][1]=ADC4;
  storeCalPoints();
}

void storeCalPoints() {
  // save the power cal points array to EEPROM
  // not many points so do it the long way
  store16bits(202,powerCalPoints[0][0]);
  store16bits(204,powerCalPoints[1][0]);
  store16bits(206,powerCalPoints[0][1]);
  store16bits(208,powerCalPoints[1][1]);
}

void readCalPoints() {
  powerCalPoints[0][0] = read16bits(202);
  powerCalPoints[1][0] = read16bits(204);
  powerCalPoints[0][1] = read16bits(206);
  powerCalPoints[1][1] = read16bits(208);
}

void printPresets() {
//  print the preset values to the PC
  for (int n=0; n<PRESETS; n++) {
    printPreset(n);
  }
}

void printPreset(int p) {
  //  print a particular preset value to the serial port
  Serial.print("P ");
  Serial.print(p);
  for (int i=0; i<3; i++) {
     Serial.print(" ");
     Serial.print(scanPresets[p][i]); 
  }
  Serial.println();
}

void storePresets(){
  //  Save the presets values to EEPROM
  //  Each int_fast32_t takes up 4 bytes
  //  Start address of preset area is 9
  for (int n=0; n<PRESETS; n++) {
    for (int p=0; p<3; p++) {
      store32bits(n*4+p*4+9, scanPresets[n][p]);
    }
  }
}



void readPresets(){
  //  read the presets values from EEPROM
  //  Each int_fast32_t takes up 4 bytes
  //  Start address of preset area is 9
  for (int n=0; n<PRESETS; n++) {
    for (int p=0; p<3; p++) {
      store32bits(n*4+p*4+9, scanPresets[n][p]);
    }
  }
}



void store32bits(int addr, int_fast32_t value){
  //  store a 32 bit variable to EEPROM
  //  low byte first
    byte b=0;
    int_fast32_t tempValue;
    tempValue=value;
    
    for (int n = 0; n<4; n++) {
      b=(tempValue & 0xFF);
//      Serial.print(b);
      EEPROM.write(addr+n,b);
      tempValue=tempValue >> 8;
    }
}

void store16bits(int addr, int value){
  //  store a 16 bit variable to EEPROM
  //  low byte first
   
      EEPROM.write(addr,lowByte(value));
      EEPROM.write(addr+1,highByte(value));
    //Serial.print("store16: ");
    //Serial.println(value);
}


int_fast32_t read32bits(int addr) {
  //  read a 32 bit variable from EEPROM
    byte b=0;
    int_fast32_t tempValue=0;
    
    for (int n = 0; n<4; n++) {
      b=EEPROM.read(addr+n);
      tempValue = tempValue<<8 + b;
    }
    return tempValue;  
}

int read16bits(int addr) {
  //  read a 16 bit variable from EEPROM
    byte b;
    word readValue=0;
  
    b = EEPROM.read(addr+1);
    readValue = (b << 8) + EEPROM.read(addr);
    //Serial.print("Read16: ");
    //Serial.println(readValue);
    return readValue;  
}


void showMenu() {
  //  
  char buffer[32];
  //  show first menu item on top line
  lcd.clear();
  switch (menuPos) {
     case 0:
       lcd.print("Mode: ");
       lcd.setCursor(0,1);
       menuMax=5;       // value limits for mode change
       menuMin=0;
       if ((menuLevel==menuLevel2)&&(menuLevel)) {
           mode=menuVal;    //  menuVal is updated with encoder interrupt routine
       }
       else {  // first time into this level
           if (menuLevel) {menuVal=mode;} else {menuVal=menuPos;};
       }
       lcd.print(mode);
     
       switch (mode) {
           case 0: 
             lcd.print(" SWR/R   ");
             break;
           case 1: 
             lcd.print(" A/D     ");
             break;
           case 2: 
             lcd.print(" Scan    ");
             break;
           case 3:
             lcd.print(" Power   ");
             break;
           case 4:
             lcd.print(" PCal 1  ");
             break;
           case 5:
             lcd.print(" PCal 2  ");
             break;
           default: 
             lcd.print(" Invalid!");
             break;
       }
       break;
     case 1:
       lcd.print("Start Freq:");  
       showFreq(0,1,startF);
       break;
     case 2:
       lcd.print("End Freq:");  
       showFreq(0,1,endF);
       break;
     case 3:
       lcd.print("Scan increment:");  
       showFreq(0,1,scanInc);
       break;
     case 4:
       lcd.print("Zero:    ");
       lcd.setCursor(0,1);
       menuMax=200;       // value limits for Zero Adjustment
       menuMin=-200;
       if ((menuLevel==menuLevel2)&&(menuLevel)) {
           if (ZeroAdjust!=menuVal) {
             ZeroAdjust=menuVal;    //  menuVal is updated with encoder interrupt routine
           }
       }
       else {  // first time into this level
           if (menuLevel) {menuVal=ZeroAdjust; Serial.println("Zero First");} else {menuVal=menuPos; Serial.println("Zero Last");};
       }
       lcd.print("            ");
       lcd.setCursor(0,1);
       lcdPrintInt(ZeroAdjust,4);
       break;       
     case 5:
       lcd.print("Preset:     ");
       lcd.setCursor(0,1);
       menuMax=12;       // value limits for preset change
       menuMin=1;
       if ((menuLevel==menuLevel2)&&(menuLevel)) {  // check if first time at this level
           presetNo=menuVal;    //  menuVal is updated by the encoder interrupt routine
       }
       else {  // first time into this level
           if (menuLevel) {menuVal=presetNo;} else {menuVal=menuPos;};
       }
       lcdPrintInt(presetNo,2);
       lcd.print (":-");
       showFreq(5,1,scanPresets[presetNo-1][0]);
       break;
     case 6:
       lcd.print("Attenuator *10:");
       lcd.setCursor(0,1);
       menuMax=600;       // value limits for Attenuator Adjustment
       menuMin=-100;
       if ((menuLevel==menuLevel2)&&(menuLevel)) {
           if (attenuator!=menuVal) {
             attenuator=menuVal;    //  menuVal is updated with encoder interrupt routine
           }
       }
       else {  // first time into this level
           if (menuLevel) {menuVal=attenuator; } else {menuVal=menuPos; };
       }
       lcd.print("            ");
       lcd.setCursor(0,1);
       lcdPrintIntAsDecimal(attenuator,5,1);
       break;       
     case 7:
       lcd.print("Pcal 1 dBm * 10:");
       lcd.setCursor(0,1);
       menuMax=170;       // value limits for Calibration point 1
       menuMin=-900;
       if ((menuLevel==menuLevel2)&&(menuLevel)) {
           if (powerCalPoints[0][0]!=menuVal) {
             powerCalPoints[0][0]=menuVal;    //  menuVal is updated with encoder interrupt routine
           }
       }
       else {  // first time into this level
           if (menuLevel) {menuVal=powerCalPoints[0][0]; } else {menuVal=menuPos; };
       }
       lcd.print("            ");
       lcd.setCursor(0,1);
       lcdPrintIntAsDecimal(powerCalPoints[0][0],5,1);
       lcd.print(", ");
       lcdPrintInt(powerCalPoints[0][1],4);
       break;       
     case 8:
       lcd.print("Pcal 2 dBm * 10:");
       lcd.setCursor(0,1);
       menuMax=170;       // value limits for Calibration point 2
       menuMin=-900;
       if ((menuLevel==menuLevel2)&&(menuLevel)) {
           if (powerCalPoints[1][0]!=menuVal) {
             powerCalPoints[1][0]=menuVal;    //  menuVal is updated with encoder interrupt routine
           }
       }
       else {  // first time into this level
           if (menuLevel) {menuVal=powerCalPoints[1][0]; } else {menuVal=menuPos; };
       }
       lcd.print("            ");
       lcd.setCursor(0,1);
       lcdPrintIntAsDecimal(powerCalPoints[1][0],5,1);
       lcd.print(", ");
       lcdPrintInt(powerCalPoints[1][1],4);
       break;       
     default:
       break;
  } // of switch
  //  position cursor on line to be changed by encoder
  if (menuLevel==0) {
    lcd.setCursor(0,0);
  }
  
  else {
    lcd.setCursor(0,1);
  }  
  lcd.cursor();    // turn underline cursor on
  menuLevel2=menuLevel;
} // of showMenu()
;
