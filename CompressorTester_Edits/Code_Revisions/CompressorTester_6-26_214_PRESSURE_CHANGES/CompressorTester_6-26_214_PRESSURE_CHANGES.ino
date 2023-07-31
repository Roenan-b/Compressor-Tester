/*

v1.15C
-Same as 1.14C but changed the min pressure to 23 below max to accomodate 77psi

*/
// Declare external libraries
#include <Arduino.h>
#include <PerfTimer.h>
#include <util/atomic.h>  
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <Math.h>

Adafruit_ADS1115 ads;

// Declare pins
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);    //(rs, enable, d4, d5, d6, d7) 
#define NUM_SAMPLES 10        //Pressure sensor input average samples
long sum = 0;
unsigned char sample_count = 0;
int startPin = 35;             // the number of the start button pin
int exhaustPin = 36;           // the number of the exhaust button pin


//Pin location/address
#define STATE0PIN 22
#define STATE1PIN 23
#define STATE2PIN 24
#define STATE3PIN 25
#define STATE4PIN 26
#define STATE5PIN 27
#define STATE6PIN 28
#define BUZZERPIN 29
#define PASSPIN 30
#define COMPFAILPIN 31
#define LEAKFAILPIN 32
#define COMPPIN 33
#define EXHAUSTPIN 2
#define PSENSORPIN A0





// Declare variables
int state;
bool case1Reached = false;
int start;
int exhaust;
const float minPressure = 0.0;            //NEW VARIABLES-RB
const float maxPress = 250.0;            //NEW VARIABLES-RB
const int analogMin = 0;            //NEW VARIABLES-RB
const int analogMax = 1023;            //NEW VARIABLES-RB
int sensorAD;   //???
float voltage;
const float pressureMin = 0.0;
const float pressureMax = 250.0;
int pressurepinValue = 0.0;            //NEW VARIABLES-RB
int setex = false;
int pressurepin;
float pressure;
float fpressure;
float maxPressure;
int maxFilltime;
int settleTime;
float initialPressure;
int measureTime;
float finalPressure;
float volume;
float hoseLength;
int minHose;
int minHose2;
int maxHose;
int maxHose2;
int calLow;   //A/D Low                              
int calHigh;  //A/D High    
float calLo;    //Pressure Low 
float calHi;    //Pressure High
int exhaustTime;
int exhaust2Time;
int b = 0;                                           //???
int e = 0;
int f = 0;
boolean newData = false;                               //???
String convert;
bool compFail;
bool leakFail;
bool elecFail;

                        
// Declare EEPROM locations
int addr = 0; //Location for MAX pressure
int addr1 = 4; //Location for MAX fill time
int addr2 = 8; //Location for settle time
int addr3 = 12; //Location for measure time
int addr4 = 16; //Location for acceptable rate
int addr5 = 20; //Location for exhaust time
int addr6 = 24; //Location for cal Low
int addr7 = 28; //Location for cal High
int addr8 = 32; //Location for cal Lo
int addr9 = 36; //Location for cal Hi

//Timer for fill time and 
PerfTimer t1(false);                    //This allows the start of the timer to be called on later
PerfTimer t2(false);



void LEDStatus()
  {
    if (state == 0) //Idle
      {
        digitalWrite(STATE0PIN, HIGH);
        digitalWrite(STATE1PIN, LOW);
        digitalWrite(STATE2PIN, LOW);
        digitalWrite(STATE3PIN, LOW);
        digitalWrite(STATE4PIN, LOW);
        digitalWrite(STATE5PIN, LOW);
        digitalWrite(STATE6PIN, LOW);     
      }
    else if (state == 1) //Fill
      {
        digitalWrite(STATE0PIN, LOW);
        digitalWrite(STATE1PIN, HIGH);
        digitalWrite(STATE2PIN, LOW);
        digitalWrite(STATE3PIN, LOW);
        digitalWrite(STATE4PIN, LOW);
        digitalWrite(STATE5PIN, LOW);
        digitalWrite(STATE6PIN, LOW);      
      }
    else if (state == 2) //Settle
      {
        digitalWrite(STATE0PIN, LOW);
        digitalWrite(STATE1PIN, LOW);
        digitalWrite(STATE2PIN, HIGH);
        digitalWrite(STATE3PIN, LOW);
        digitalWrite(STATE4PIN, LOW);
        digitalWrite(STATE5PIN, LOW);
        digitalWrite(STATE6PIN, LOW);    
      }
    else if (state == 3) //Measuring PSI
      {
        digitalWrite(STATE0PIN, LOW);
        digitalWrite(STATE1PIN, LOW);
        digitalWrite(STATE2PIN, LOW);
        digitalWrite(STATE3PIN, HIGH);
        digitalWrite(STATE4PIN, LOW);
        digitalWrite(STATE5PIN, LOW);
        digitalWrite(STATE6PIN, LOW);    
      }
    else if (state == 6) //Exhaust
      {
        digitalWrite(STATE0PIN, LOW);
        digitalWrite(STATE1PIN, LOW);
        digitalWrite(STATE2PIN, LOW);
        digitalWrite(STATE3PIN, LOW);
        digitalWrite(STATE4PIN, LOW);
        digitalWrite(STATE5PIN, LOW);
        digitalWrite(STATE6PIN, HIGH);    
      }
    else 
      {
        digitalWrite(STATE0PIN, LOW);
        digitalWrite(STATE1PIN, LOW);
        digitalWrite(STATE2PIN, LOW);
        digitalWrite(STATE3PIN, LOW);
        digitalWrite(STATE4PIN, LOW);
        digitalWrite(STATE5PIN, LOW);
        digitalWrite(STATE6PIN, LOW);    
      }               
  }

//Handles EEPROM writing
void EEPROMWritelong(int address, long value)
      {
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
      }

//Handles EEPROM reading
long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.          
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);


      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }

void CheckButtons()
{
  start = digitalRead(startPin);
  exhaust = digitalRead(exhaustPin);  
}

void MeasurePressure()
{
  int pressurepin = analogRead(PSENSORPIN);
  float voltage = pressurepin * (5.0 / 1023.0);
  pressurepinValue = (((voltage - 0.5) / 4.5) * (pressureMax - pressureMin) + pressureMin)/1.99501247;
  /*analogWrite(PSENSORPIN, HIGH);
  pressurepin = analogRead(PSENSORPIN);

  pressure = map(pressurepin, analogMin, analogMax, minPressure, maxPressure);
 */
  Serial.print("Pressure: ");
  Serial.print(pressurepinValue);
  Serial.println(" PSI");
  delay(100);



}

  

void InitializeOutputs()
{
  pinMode(STATE0PIN, OUTPUT);
  pinMode(STATE1PIN, OUTPUT);
  pinMode(STATE2PIN, OUTPUT);
  pinMode(STATE3PIN, OUTPUT);
  pinMode(STATE4PIN, OUTPUT);
  pinMode(STATE5PIN, OUTPUT);
  pinMode(STATE6PIN, OUTPUT);
  pinMode(BUZZERPIN, OUTPUT);
  pinMode(PASSPIN, OUTPUT);
  pinMode(COMPFAILPIN, OUTPUT);
  pinMode(LEAKFAILPIN, OUTPUT);
  pinMode(COMPPIN, OUTPUT);
  pinMode(EXHAUSTPIN, OUTPUT);
  pinMode(PSENSORPIN, INPUT);

  digitalWrite(STATE0PIN, HIGH);
  digitalWrite(STATE1PIN, HIGH);
  digitalWrite(STATE2PIN, HIGH);
  digitalWrite(STATE3PIN, HIGH);
  digitalWrite(STATE4PIN, HIGH);
  digitalWrite(STATE5PIN, HIGH);
  digitalWrite(STATE6PIN, HIGH); 
  delay(1000);  
  digitalWrite(STATE0PIN, LOW);
  digitalWrite(STATE1PIN, LOW);
  digitalWrite(STATE2PIN, LOW);
  digitalWrite(STATE3PIN, LOW);
  digitalWrite(STATE4PIN, LOW);
  digitalWrite(STATE5PIN, LOW);
  digitalWrite(STATE6PIN, LOW);
}

void Read_EEPROM()   
{  
  maxPressure = EEPROMReadlong(addr);
  maxPressure = maxPressure / 400;
  Serial.print("MAX Pressure: ");
  Serial.print(maxPressure);
  Serial.println(" PSI");
  delay(10);
  lcd.print("Max Pressure:");
  lcd.setCursor(0, 1);
  lcd.print(maxPressure);
  lcd.print(" PSI");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
 

  maxFilltime = EEPROMReadlong(addr1);
  Serial.print("MAX Fill Time: ");
  Serial.print(maxFilltime);
  Serial.println(" seconds");
  delay(10);
  lcd.print("Max Fill Time:");
  lcd.setCursor(0, 1);
  lcd.print("60");
  lcd.print(" Seconds");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  

  settleTime = 10;
  Serial.print("Settle Time: ");
  Serial.print(settleTime);
  Serial.println(" seconds");
  delay(10);
  lcd.print("Settle Time:");
  lcd.setCursor(0, 1);
  lcd.print(settleTime);
  lcd.print(" Seconds");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  
 
  measureTime = 5;
  Serial.print("Measure Time: ");
  Serial.print(measureTime);
  Serial.println(" seconds");
  delay(10);
  lcd.print("Measure Time:");
  lcd.setCursor(0, 1);
  lcd.print(measureTime);
  lcd.print(" Seconds");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);

  exhaust2Time = 60;
  exhaustTime = 59900;
  Serial.print("Exhaust Time: ");
  Serial.print("60");
  Serial.println(" seconds");
  delay(10);
  lcd.print("Exhaust Time:");
  lcd.setCursor(0, 1);
  lcd.print("60");
  lcd.print(" Seconds");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);


  volume = 22730000;                                                                                 //VOLUME CHANGED//HARDCODED//NEW VOLUME
  volume = volume / 1000;
  Serial.print("Volume: ");
  Serial.print(volume);
  Serial.println(" cm^3");
  delay(10);
  lcd.print("Volume:");
  lcd.setCursor(0, 1);
  lcd.print(volume);
  lcd.print(" cm3");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);

  calLo = EEPROMReadlong(addr8);
  calLo = calLo / 1000;
  Serial.print("Calibrated Low Pressure: ");
  Serial.print(calLo);
  Serial.println(" PSI");
  delay(10);
  lcd.print("Cal Lo Pressure:");
  lcd.setCursor(0, 1);
  lcd.print(calLo);
  lcd.print(" PSI");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);

  calHi = EEPROMReadlong(addr9);
  calHi = calHi / 400;                                                //1000 to 400-RB
  Serial.print("Calibrated High Pressure: ");
  Serial.print(calHi);
  Serial.println(" PSI");
  delay(10);
  lcd.print("Cal Hi Pressure:");
  lcd.setCursor(0, 1);
  lcd.print(calHi);
  lcd.print(" PSI");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);

}
                       
void WriteMaxPressure()
{
  EEPROMWritelong(addr,250); 
  Serial.print("NEW Max Pressure: ");
  Serial.println(" PSI");
}

void WriteMaxFillTime()
{
  EEPROMWritelong(addr1,60);
}

void WriteSettleTime()
{
  EEPROMWritelong(addr2,settleTime);
  Serial.print("NEW Settle Time: ");
  Serial.println(" seconds");
}

void WriteMeasureTime()
{
  EEPROMWritelong(addr3,measureTime);
  Serial.print("NEW Measure Time: ");
  Serial.println(" seconds");
}

void WriteExhaustTime()
{
  EEPROMWritelong(addr5,exhaustTime);
  Serial.print("NEW Exhaust Time: ");;
  Serial.println(" seconds");
}

void WriteCalLow()
{
  EEPROMWritelong(addr6,calLow);
  Serial.print("NEW Calibrated Low A/D: ");
  Serial.println(" bits");
}

void WriteCalHigh()
{
  EEPROMWritelong(addr7,calHigh);
  Serial.print("NEW Calibrated High A/D: ");
  Serial.println(" bits");
}

void WriteCalLo()
{
  EEPROMWritelong(addr8,calLo);
  Serial.print("NEW Calibrated Low Pressure: ");
  Serial.print(EEPROMReadlong(addr8)/1000);
  Serial.println(" PSI");
}

void WriteCalHi()
{
  EEPROMWritelong(addr9,calHi);
  Serial.print("NEW Calibrated High Pressure: ");
  Serial.print(EEPROMReadlong(addr9)/1000);
  Serial.println(" PSI");
}

void WriteVolume()
{

  Serial.print("NEW Volume: ");
                                                                    
  Serial.println(" cm3");
}


//LEDs
void idleBlink()
{
    digitalWrite(STATE0PIN, HIGH);
    lcd.print(".");
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    delay(500);
    digitalWrite(STATE0PIN, HIGH);
    lcd.print(".");
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    delay(500);
    digitalWrite(STATE0PIN, HIGH);
    lcd.print(".");
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    delay(500);
    digitalWrite(STATE0PIN, HIGH);
    lcd.print(".");
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    delay(500);
    digitalWrite(STATE0PIN, HIGH);
    lcd.print(".");
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    delay(500);
   
}

void allBlink()
{
    digitalWrite(STATE0PIN, HIGH);
    digitalWrite(STATE1PIN, HIGH);
    digitalWrite(STATE2PIN, HIGH);
    digitalWrite(STATE3PIN, HIGH);
    digitalWrite(STATE4PIN, HIGH);
    digitalWrite(STATE5PIN, HIGH);
    digitalWrite(STATE6PIN, HIGH);
    lcd.print(".");
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    digitalWrite(STATE1PIN, LOW);
    digitalWrite(STATE2PIN, LOW);
    digitalWrite(STATE3PIN, LOW);
    digitalWrite(STATE4PIN, LOW);
    digitalWrite(STATE5PIN, LOW);
    digitalWrite(STATE6PIN, LOW);
    delay(500);
    digitalWrite(STATE0PIN, HIGH);
    digitalWrite(STATE1PIN, HIGH);
    digitalWrite(STATE2PIN, HIGH);
    digitalWrite(STATE3PIN, HIGH);
    digitalWrite(STATE4PIN, HIGH);
    digitalWrite(STATE5PIN, HIGH);
    digitalWrite(STATE6PIN, HIGH);
    lcd.print(".");
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    digitalWrite(STATE1PIN, LOW);
    digitalWrite(STATE2PIN, LOW);
    digitalWrite(STATE3PIN, LOW);
    digitalWrite(STATE4PIN, LOW);
    digitalWrite(STATE5PIN, LOW);
    digitalWrite(STATE6PIN, LOW);
    delay(500);
    digitalWrite(STATE0PIN, HIGH);
    digitalWrite(STATE1PIN, HIGH);
    digitalWrite(STATE2PIN, HIGH);
    digitalWrite(STATE3PIN, HIGH);
    digitalWrite(STATE4PIN, HIGH);
    digitalWrite(STATE5PIN, HIGH);
    digitalWrite(STATE6PIN, HIGH);
    lcd.print(".");
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    digitalWrite(STATE1PIN, LOW);
    digitalWrite(STATE2PIN, LOW);
    digitalWrite(STATE3PIN, LOW);
    digitalWrite(STATE4PIN, LOW);
    digitalWrite(STATE5PIN, LOW);
    digitalWrite(STATE6PIN, LOW);
    delay(500);
    digitalWrite(STATE0PIN, HIGH);
    digitalWrite(STATE1PIN, HIGH);
    digitalWrite(STATE2PIN, HIGH);
    digitalWrite(STATE3PIN, HIGH);
    digitalWrite(STATE4PIN, HIGH);
    digitalWrite(STATE5PIN, HIGH);
    digitalWrite(STATE6PIN, HIGH);
    lcd.print(".");
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    digitalWrite(STATE1PIN, LOW);
    digitalWrite(STATE2PIN, LOW);
    digitalWrite(STATE3PIN, LOW);
    digitalWrite(STATE4PIN, LOW);
    digitalWrite(STATE5PIN, LOW);
    digitalWrite(STATE6PIN, LOW);
    delay(500);
    digitalWrite(STATE0PIN, HIGH);
    digitalWrite(STATE1PIN, HIGH);
    digitalWrite(STATE2PIN, HIGH);
    digitalWrite(STATE3PIN, HIGH);
    digitalWrite(STATE4PIN, HIGH);
    digitalWrite(STATE5PIN, HIGH);
    digitalWrite(STATE6PIN, HIGH);
    lcd.print(".");
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    digitalWrite(STATE1PIN, LOW);
    digitalWrite(STATE2PIN, LOW);
    digitalWrite(STATE3PIN, LOW);
    digitalWrite(STATE4PIN, LOW);
    digitalWrite(STATE5PIN, LOW);
    digitalWrite(STATE6PIN, LOW);
    delay(500);
}
//All LEDs on
void allOn()
{
    digitalWrite(STATE0PIN, HIGH);
    digitalWrite(STATE1PIN, HIGH);
    digitalWrite(STATE2PIN, HIGH);
    digitalWrite(STATE3PIN, HIGH);
    digitalWrite(STATE4PIN, HIGH);
    digitalWrite(STATE5PIN, HIGH);
    digitalWrite(STATE6PIN, HIGH);
}
//All LEDs off
void allOff()
{
    digitalWrite(STATE0PIN, LOW);
    digitalWrite(STATE1PIN, LOW);
    digitalWrite(STATE2PIN, LOW);
    digitalWrite(STATE3PIN, LOW);
    digitalWrite(STATE4PIN, LOW);
    digitalWrite(STATE5PIN, LOW);
    digitalWrite(STATE6PIN, LOW);
}


/*---------------------------------------------------------------------------------------------------------------------------------------------------------*/
                                                                  //Initial startup of the code
void setup() 
  {
    ads.begin(0x48);
    state = 0; 
    setex = false;
    InitializeOutputs();                                          //Initialize Outputs
    allOn();
    Serial.begin(9600);                                           //Baud rate
    Serial.println("<Tester Starting>");                          //Prints to the serial monitor and println gives a newline
    lcd.begin(16, 2);                                             //LCD screen size
    lcd.print("Hello :)");
    delay(500);                                                   //Time in milliseconds
    lcd.clear();                                                  //Clears the screen
    lcd.print("Software: v3.1");
    delay(500);
    lcd.clear();
    lcd.print("Coded by Roe");
    delay(500);
    lcd.clear();
    minHose2 = 2;                                                 //Air hose specs
    maxHose2 = 4;
    lcd.print("Hose Length: ");
    lcd.setCursor(0,1);                                           //Sets where the LCD prints
    lcd.print(minHose2);
    lcd.print("-");
    lcd.print(maxHose2);
    lcd.print("ft");
    delay(500);  
    Serial.println("Current stored values in EEPROM:");
    lcd.clear();
    lcd.print("Current Settings");
    delay(500);
    lcd.clear();
    Read_EEPROM();                                               //Pulls the values from the EEPROM.h file
    lcd.setCursor(0, 0);
    lcd.print("Working my magic");
    lcd.setCursor(0, 1);
    lcd.print("Hold on");
    lcd.clear();
   // lcd.print("Pre-Test");
   // lcd.setCursor(0,1);
   // lcd.print("Exhaust");
    //digitalWrite(STATE6PIN, HIGH);                                        //digitalWrite is sending current/signal to that pin
    //analogWrite(EXHAUSTPIN, 255);
    delay(100);
    lcd.clear();
    MeasurePressure();
    allOff();
    allBlink(); 

    
    delay(500);
    lcd.clear();
    allOff();
  }
                                                                 //Goes straight into main loop
/*---------------------------------------------------------------------------------------------------------------------------------------------------------*/

void loop()                                                                   //Starts the main loop
  {
    switch (state)
      {
        case 0:  //Idle
        state0:
        MeasurePressure();
        while (pressurepinValue != 0)
          {
            digitalWrite(STATE6PIN, HIGH);
            analogWrite(EXHAUSTPIN, 255);
            MeasurePressure();
            lcd.print("Pre-Test");
            lcd.setCursor(10,0);
            lcd.print("PSI:");
            lcd.print(pressurepinValue);
            lcd.setCursor(0,1);
            lcd.print("Exhaust           ");
          
          if (pressurepinValue <= 0)
            {
              analogWrite(EXHAUSTPIN, 0);
              digitalWrite(STATE6PIN, LOW);
              MeasurePressure();
              break;
            }
          }

        lcd.clear();
        //lcd.print("Ready");
        
          while (state == 0)                                                  //While idle
              { 
                analogWrite(EXHAUSTPIN, 0);                                   //Makes sure that the exhaust is turned off
                LEDStatus();                                                  //Turns on the correct LED for the case/state
                lcd.setCursor(0, 0);
                lcd.print("Ready");
                lcd.setCursor(8,0);
                lcd.print("PSI:");
                MeasurePressure();
                lcd.print(pressurepinValue);
                lcd.setCursor(0, 1);
                lcd.print("Press START");
                CheckButtons();
                if (start == LOW && exhaust == HIGH)                         //HIGH = off & LOW = on INVERSE LOGIC FOR IF & WHILE STATEMENTS
                  {
                    delay(5);
                    CheckButtons();                                          //Checks again to ensure button is pressed
                    if (start == LOW)                                        //If start is on
                      {
                        lcd.clear();
                        lcd.print("Starting");
                        delay(500);
                        state = 1;                                          //Jump to case 1
                        break;                                              //Breaks out of this loop
                      }
                    else
                      {
                        state = 0;  
                      }
                  }
                else if (start == HIGH && exhaust == LOW)                    //If exhaust is on
                  {
                    delay(10);
                    CheckButtons();                                          //Checks to ensure exhaust is pressed
                    if (exhaust == LOW)
                      {
                        state = 6;                                           //Jump to state 6
                        break;                                 
                      }
                    else
                      {
                        state = 0;  
                      }
                  }  
                delay(10);
                CheckButtons();                                              //Consitently checking for button inputs
                
               
              }
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/
        case 1:                                                                             //Compressor output ON
        state1:
          lcd.clear();
          lcd.print("Compressor ON"); 
          while (state == 1)                                                                //While compressor is on
              {  
                LEDStatus();                                                                //State 1 LED turned on
                CheckButtons();                                                             //Checking button inputs
                 for (int f = 0; f <= maxFilltime; f++)
                  {
                    digitalWrite(STATE1PIN, HIGH);
                    CheckButtons();
                    delay(500);
                    if (start == HIGH && exhaust ==LOW)
                      {
                        digitalWrite(COMPPIN, LOW);
                        lcd.clear();
                        lcd.print("Cancelled");
                        state = 6;
                        break;
                      }
                    MeasurePressure();
                    delay(400);
                    digitalWrite(STATE1PIN, LOW);
                    lcd.setCursor(0,1);
                    lcd.print(f+1);
                    lcd.print(" / ");
                    lcd.print(maxFilltime);
                    digitalWrite(COMPPIN, HIGH);
                    lcd.setCursor(8, 1);
                    lcd.print("PSI:");
                    lcd.print(pressurepinValue);
                  }
                 state = 2;
                 break;

                
                /*MeasurePressure();
                Serial.println(pressurepinValue)
                digitalWrite(COMPPIN, HIGH);                                                //Compressor ON
                t1.start();                                                                 //Starts the first timer
                delay(60000);                                                               //Timer duration 60 seconds//DELAYS THE ENTIRE SYSTEM
                t1.stop();                                                                  //Timer stops after delay of 60,000 milliseconds or 60 seconds
                Serial.println("Fill time: " + String(t1.totalTime/1000) + " seconds");     //Gives report of fill time to monitor
                if (t1.totalTime >= 59900)                                                  //If fill time = 60 seconds
                  {
                    digitalWrite(COMPPIN, LOW);                                             //Compressor turned off
                    state = 2;                                                              //State = 2 or settle state
                    lcd.clear();
                    lcd.print("Done");
                    delay(100);
                    break;                                                                  //Breaks out of case 1
                    }*/
                }
                  
              
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/  
        case 2:                                                                             //Settle Time
        state2:
          lcd.clear();
          lcd.print("Settle ");
          while (state == 2)
            {
              digitalWrite(COMPPIN, LOW);                                                   //Turns off compressor 
              LEDStatus();
              delay(1000);
              CheckButtons();
              while (start == HIGH && exhaust == HIGH)                                      //While no button inputs
                {
                  CheckButtons();
                  for (int b = 0; b <= settleTime; b++)                                     //Handles the settle time counter
                    {
                      digitalWrite(STATE2PIN, HIGH);                                        //State2pin turned ON
                      CheckButtons();                                                       //Checks inputs again
                      state = 2;                                                            //Ensures state is 2
                      MeasurePressure();                                                    //Continuously provides PSI readings
                      lcd.setCursor(0,0);
                      lcd.print("Settle PSI:");
                      lcd.print(pressurepinValue);                                               //Pressurepin is the PSI value from MeasurePressure function
                      delay(500); 
                      digitalWrite(STATE2PIN, HIGH);                                        //State2pin on again for redundancy
                      delay(500); 
                      lcd.setCursor(0, 1);
                      lcd.print(b+1);                                                       //Visual timer for the settle timer
                      lcd.print(" / ");
                      lcd.print(settleTime);                                                //Settle time is 10 seconds
                    }                                                                       //Once b+1 reaches the settle time of 10, the for loop is completed
                  state = 3;                                                                //State 3 entered right after for loop
                  break;  
                
              if (start == LOW && exhaust == LOW)                                           //If both start and exhaust are on system cancels//Only for system failure and user safety
                {
                  lcd.clear();
                  lcd.print("Cancelled");
                  delay(500);
                  state = 6;
                }
              if (start == HIGH && exhaust == LOW);                                         //If exhaust is on the settle cancels and state 6 is enetered
                {
                  lcd.clear();
                  lcd.print("Cancelled");
                  delay(500);
                  state = 6;
                    }
                }
            }
              
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/
        case 3:                                                                             //Measure Time              
        state3:
          lcd.clear();
          lcd.print("Measuring");
          delay(100);
          while (state == 3)                                                                //While measuring
            { 
              LEDStatus();                                                                  //State 3 LED on
              delay(500);
              MeasurePressure();                                                            //Pressure is continuously checked
              lcd.clear();
              CheckButtons();                                                               //Checks for button inputs
              if (start == HIGH && exhaust == HIGH)                                         //If no button inputs
              {
                MeasurePressure();                                                          //Pressure is measured
                lcd.print("Final PSI:");
                lcd.setCursor(0, 1);
                lcd.print(pressurepinValue);                                                     //Takes the PSI value from MeasurePressure()
                digitalWrite(PASSPIN, HIGH);                                                //Turns the pass LED on once final PSI is displayed
                delay(100);
              }
              if (start == HIGH && exhaust == LOW)                                          //If exhaust button is pushed
              {
                delay(100);
                lcd.clear();
                digitalWrite(PASSPIN, LOW);                                                 //Turns off the pass LED
                state = 6;                                                                  //Exhausts
                break;
              }
            }
                           
/*---------------------------------------------------------------------------------------------------------------------------------------------------------*/
  
       case 6:                                                                              //Exhaust
       state6:
        lcd.clear();
        digitalWrite(PASSPIN, LOW);
        lcd.print("Exhausting");
         while (state == 6)
            {   
              LEDStatus();
              for (int e = 0; e <= exhaust2Time; e++)
                {
                  digitalWrite(STATE6PIN, HIGH);
                  digitalWrite(EXHAUSTPIN, HIGH);
                  CheckButtons();
                  MeasurePressure();
                  lcd.setCursor(9,1);
                  lcd.print("PSI:");
                  lcd.print(pressurepinValue);
                  delay(500);
                  if (pressurepinValue <= pressureMin)
                    {
                      digitalWrite(EXHAUSTPIN, LOW);
                      lcd.clear();
                      lcd.print("Done");
                      state = 0;
                      break;
                    }
                  digitalWrite(STATE6PIN, HIGH);
                  delay(400);
                  lcd.setCursor(0, 1);
                  lcd.print(e+1);
                  lcd.print(" / ");
                  lcd.print(exhaust2Time);
                }
               digitalWrite(EXHAUSTPIN, LOW);
               state = 0;
               break;
            }
              /*t2.start();                                                                   
              LEDStatus();
              digitalWrite(COMPFAILPIN, LOW);
              digitalWrite(PASSPIN, LOW);
              digitalWrite(LEAKFAILPIN, LOW);   
              analogWrite(EXHAUSTPIN, 255);
              delay(60000);
              t2.stop();
              analogWrite(EXHAUSTPIN, 0);
              if (t2.totalTime >= 59900)
              {
                lcd.clear();
                state = 0;
                break;
              }
            }
      */
     /*---------------------------------------------------------------------------------------------------------------------------------------------------------*/                                                                        

    }                                                                                        //End Switch Case

}                                                                                            //End Main Loop
