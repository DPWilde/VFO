/*
Main code by Richard Visokey AD7C - www.ad7c.com
Revision 2.0 - November 6th, 2013
*/

// Include the library code
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <rotary.h>
#include <EEPROM.h>

//Setup some items
#define W_CLK 12      // Pin 8 - connect to AD9851 module word load clock pin (CLK)
#define FQ_UD 11      // Pin 9 - connect to freq update pin (FQ)
#define DATA 13       // Pin 10 - connect to serial data load pin (DATA)
#define RESET 10      // Pin 11 - connect to reset pin (RST)
#define R_BUTTON 4    // Rotary encoder button
#define Vin1 A0         // V1 analogue input
#define Vin3 A1         // V3 analogue input
#define OpAmpGain 100.0  // Gain of Op amp used for reading the V1/V3 values
#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }
Rotary r = Rotary(2,3); // sets the pins the rotary encoder uses.  Must be interrupt pins.

//LiquidCrystal lcd(12, 13, 7, 6, 5, 4); // I used an odd pin combination because I need pin 2 and 3 for the interrupts.
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
int_fast32_t rx=7200000; // Starting frequency of VFO
int_fast32_t rx2=1; // variable to hold the updated frequency
int_fast32_t increment = 10; // starting VFO update increment in HZ.
int buttonstate = 0;
String hertz = "10 Hz";
int  hertzPosition = 9;
byte ones,tens,hundreds,thousands,tenthousands,hundredthousands,millions ;  //Placeholders
String freq; // string to hold the frequency
int_fast32_t timepassed = millis(); // int to hold the arduino miilis since startup
int memstatus = 1;  // value to notify if memory is current or old. 0=old, 1=current.
float ARef = 5.0;




int ForceFreq = 0;  // Change this to 0 after you upload and run a working sketch to activate the EEPROM memory.  YOU MUST PUT THIS BACK TO 0 AND UPLOAD THE SKETCH AGAIN AFTER STARTING FREQUENCY IS SET!

int loopCount = 1;
double n=0;            //  used to count intevals between analogue updates
int ADC1, ADC3;        //  values read from Analog inputs relating to V1 and V3
float Rx;              //  calculated resistance of load
float SWR;              //  calculated SWR of load against 50R

void setup() {
  pinMode(R_BUTTON,INPUT); // Connect to a button that goes to GND on push
  digitalWrite(R_BUTTON,HIGH);  //  Turn on pull up resistor
  pinMode(Vin1, INPUT);
  digitalWrite(Vin1,LOW);
  pinMode(Vin3, INPUT);
  digitalWrite(Vin3,LOW);
  // initialize the serial communication:
  Serial.begin(9600);
  
  if (ARef == 1.1) {
    analogReference(INTERNAL);   // set 1.1 v internal reference
  }
  else {
    analogReference(DEFAULT);    // Vcc
  }
  
  lcd.begin(16, 2);            // 1602 LCD display
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();
  pinMode(FQ_UD, OUTPUT);
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT); 
  pulseHigh(RESET);
  pulseHigh(W_CLK);
  pulseHigh(FQ_UD);  // this pulse enables serial mode on the AD9850/51 - Datasheet page 12.
  lcd.setCursor(hertzPosition,0);    
  //lcd.print(hertz);
   // Load the stored frequency  
  if (ForceFreq == 0) {
    Serial.println("Reading EEPROM");
    freq = String(EEPROM.read(0))+String(EEPROM.read(1))+String(EEPROM.read(2))+String(EEPROM.read(3))+String(EEPROM.read(4))+String(EEPROM.read(5))+String(EEPROM.read(6));
    rx = freq.toInt();  
  }
  Serial.println("VFO Serial Output");

}


void loop() {  // MAIN LOOP  **********************************************************************************

  
  if (rx != rx2){    
        showFreq(1);
        sendFrequency(rx);
//        Serial.println("test");
        loopCount++;
        if (loopCount>10) loopCount=1;
        rx2 = rx;
      }
      
  buttonstate = digitalRead(R_BUTTON);
  if(buttonstate == LOW) {
        setincrement();        
    };

  // Write the frequency to memory if not stored and 2 seconds have passed since the last frequency change.
  if(memstatus == 0){   
      if(timepassed+2000 < millis()){
        storeMEM();
        }
      }
      
  // Check if any serial commands received 
  if (Serial.available())
      Serial.write(Serial.read());  //Just echo back at the moment
      
  // write the V1 and V2 values to LCD, but only every 100 scans
  if (n>30000) {
    ADC1 = analogRead(Vin1);
    ADC3 = analogRead(Vin3);
    Rx=calcRx(ADC1, ADC3);
    SWR=Rx/50.0;
    lcd.setCursor(0,1);
    lcd.noCursor();
    lcd.print("S=");
    if (Rx<1000.00) lcd.print(" ");
    if (Rx<100.00) lcd.print(" ");
    if (Rx<10.00) lcd.print(" ");
    lcd.print(SWR);
    if (Rx<100.00) lcd.print(" ");
    if (Rx<10.00) lcd.print(" ");

//    lcdPrintInt(ADC1);
//    lcd.setCursor(5,1);
//    lcdPrintInt(ADC3);
    lcd.print(" R=");
    if (Rx<100.00) lcd.print(" ");
    if (Rx<10.00) lcd.print(" ");
    lcd.print(Rx);
    
    lcd.setCursor(hertzPosition,0);  //  put the cursor back under the frequency digit to be changed
    lcd.cursor();
    printFreq();
    n=0;
  }
  else {
    n++;
  }
  
  
}  // END of MAIN LOOP  ****************************************************************************************


ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result) {    
    if (result == DIR_CW){rx=rx+increment;}
    else {rx=rx-increment;};       
      if (rx >=30000000){rx=rx2;}; // UPPER VFO LIMIT
      if (rx <=1000000){rx=rx2;}; // LOWER VFO LIMIT
  }
}



// frequency calc from datasheet page 8 = <sys clock> * <frequency tuning word>/2^32
void sendFrequency(double frequency) {  
  //int32_t freq = frequency * 4294967295/125000000;  // note 125 MHz clock on 9850.  You can make 'slight' tuning variations here by adjusting the clock frequency.
  int32_t freq = frequency * 4294967295/180000000;  // note 180 MHz clock on 9851.  You can make 'slight' tuning variations here by adjusting the clock frequency.
  for (int b=0; b<4; b++, freq>>=8) {
    tfr_byte(freq & 0xFF);
  }
//  tfr_byte(0x000);   // Final control byte, all 0 for 9850 chip
  tfr_byte(0x001);   // Final control byte, all 0 except W32(enable 6x) for 9851 chip
  pulseHigh(FQ_UD);  // Done!  Should see output
}
// transfers a byte, a bit at a time, LSB first to the 9850 via serial DATA line
void tfr_byte(byte data)
{
  for (int i=0; i<8; i++, data>>=1) {
    digitalWrite(DATA, data & 0x01);
    pulseHigh(W_CLK);   //after each bit sent, CLK is pulsed high
  }
}

void setincrement(){
  if(increment == 10){increment = 100; hertz = "100 Hz"; hertzPosition=8;}
  //else if (increment == 50){increment = 100;  hertz = "100 Hz"; hertzPosition=4;}
  else if (increment == 100){increment = 1000; hertz="1 kHz"; hertzPosition=6;}
  //else if (increment == 500){increment = 1000; hertz="1 Khz"; hertzPosition=6;}
  else if (increment == 1000){increment = 10000; hertz="10 Khz"; hertzPosition=5;}
  //else if (increment == 2500){increment = 5000; hertz="5 Khz"; hertzPosition=6;}
  //else if (increment == 5000){increment = 10000; hertz="10 Khz"; hertzPosition=5;}
  else if (increment == 10000){increment = 100000; hertz="100 Khz"; hertzPosition=4;}
  else if (increment == 100000){increment = 1000000; hertz="1 Mhz"; hertzPosition=2;}  
  else{increment = 10; hertz = "10 Hz"; hertzPosition=9;};  
//   lcd.setCursor(0,1);
//   lcd.print("                ");
   lcd.cursor();
   lcd.setCursor(hertzPosition,0); 
//   lcd.print(hertz); 
   delay(250); // Adjust this delay to speed up/slow down the button menu scroll speed.
};

void lcdPrintInt(int i) {
  //  print with right justify.  Uses 5 chars space
    if (i<10000) lcd.print(" ");
    if (i<1000) lcd.print(" ");
    if (i<100) lcd.print(" ");
    if (i<10) lcd.print(" ");
    lcd.print(i);
}

void showFreq(bool SerialOut){
    millions = int(rx/1000000);
    hundredthousands = ((rx/100000)%10);
    tenthousands = ((rx/10000)%10);
    thousands = ((rx/1000)%10);
    hundreds = ((rx/100)%10);
    tens = ((rx/10)%10);
    ones = ((rx/1)%10);
    lcd.setCursor(0,0);
    lcd.print("                ");
   if (millions > 9){lcd.setCursor(1,0);}
   else{lcd.setCursor(2,0);}
    lcd.print(millions);
    lcd.print(".");
    lcd.print(hundredthousands);
    lcd.print(tenthousands);
    lcd.print(thousands);
    lcd.print(".");
    lcd.print(hundreds);
    lcd.print(tens);
    lcd.print(ones);
    lcd.print(" Mhz  ");
    lcd.cursor();    // turn underline cursor on
    lcd.setCursor(hertzPosition,0); //  position cursor under digit to be changed
 
    if (SerialOut) printFreq();      
    timepassed = millis();
    memstatus = 0; // Trigger memory write
};
void showFreq(){
  //  overload allowing for optional paramater serial out.  If omitted then don't sent to serial port.
  showFreq(0);
}

void printFreq() {
//  millions etc are calculated in show freq, called whenever freq is changed
        Serial.print(millions);
//        Serial.print(".");
        Serial.print(hundredthousands);
        Serial.print(tenthousands);
        Serial.print(thousands);
//        Serial.print(".");
        Serial.print(hundreds);
        Serial.print(tens);
        Serial.print(ones);
        Serial.print(",");
        Serial.print(analogRead(A0));
        Serial.print(",");
        Serial.println(analogRead(A1));
//        Serial.print(" Mhz  ");
}

float calcRx(int ADC1, int ADC3) {
//  Calculate the resistance of the load from the input voltage(V1) and bridge voltage (V3)
//  V2 is the voltage across the load.  ADC1 and ADC3 are the raw values from the A-D convertor
//  We need to be careful to avoid overflows as V3 approaches V1 in value, when the load is far from 50R
//  Also we measure RMS value of V3 so cannot determine if it is -ve, so cannot determine if Rx is > or < 50R
//  Maybe best to also measure V2, or measure V2 instead of V3?
//  For the moment ignore the forward voltage drop on the diodes, but the germanium diodes are non-linear in this region
//  and the value should be corrected
//
//  Formula Rx = 50/(2V1/(V3+V1)-1)
//  First calculate the voltages
//  Assume forward voltage drop proportional to measured V (not true)
//  And just add a value to the ADC reading to compensate

//  float V1, V3;
  
  //  We have OpAmp gain of 100, resolution 1024, so actual measured voltage after diodes should be ADC*100*Aref/1024
//  V1=float(ADC1)*100.0*ARef/1024.0
//  V3=float(ADC3)*100.0*ARef/1024.0

int corrADC1, corrADC3;

corrADC1=ADC1+0.2*ADC1;
corrADC3=ADC3+21.0*sqrt(ADC3)-0.5*ADC3;

  
  if ((ADC1-ADC3)<5) {
    return 999.0;
  }
  else {
//    return 50.0/(2.0*V1/(V3+V1)-1.0);
    return 50.0/(2.0*corrADC1/(corrADC3+corrADC1)-1.0);
  }
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
   Serial.println("Freq saved");

};


