/*

v3.18
-Based on the original leak tester software (v1.15)
-This software is meant to run a variety of compressors for 60 seconds at max output
-Settles for 10 seconds after 60 second fill period, then displays the settled PSI
-Exhaust set for 300 seconds but stops after all air is exhausted
-

*/

  
/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 
// Declare external libraries

#include <Arduino.h> 
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <Math.h>

  
/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 


Adafruit_ADS1115 ads;
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);    //(rs, enable, d4, d5, d6, d7) 

  
/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 
// Declare pins

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
int startPin = 35;             // the number of the start button pin
int exhaustPin = 36;           // the number of the exhaust button pin

  
/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 
// Declare variables

int state;
int start;
int exhaust;
const float minPressure = 0.0;
const float maxPress = 250.0;
const int analogMin = 0;
const int analogMax = 1023;
float voltage;
float voltage2;
const float pressureMin = 0.0;
const float pressureMax = 250.0;
float pressurepinValue = 0.0;            
float pressurepin;
float pressure;
float fpressure;
float maxPressure;
int maxFilltime;
int settleTime;
int measureTime;
float volume;
float hoseLength;
int minHose;
int minHose2;
int maxHose;
int maxHose2;
int exhaust2Time;
int b = 0;
int e = 0;
int f = 0;
bool compFail;
bool leakFail;
bool elecFail;
char buffer1[10];
char buffer2[10];
float exhaustpressure;
float pressurepinValues;

  
/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 
//Assigning LEDs to specific states

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

  
  
/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 
// Declare EEPROM locations

int addr = 0; //Location for MAX pressure
int addr1 = 4; //Location for MAX fill time
int addr2 = 8; //Location for settle time
int addr3 = 12; //Location for measure time
int addr5 = 20; //Location for exhaust time


/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 
//Changes where the variables are written to so EEPROM doesn't overload
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


/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 
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

/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 
//Checks switch

void CheckButtons()
{
  start = digitalRead(startPin);
  exhaust = digitalRead(exhaustPin);  
}

/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 
//Pressure calculation/conversion

void MeasurePressure()
{
  float pressurepin = analogRead(PSENSORPIN);
  voltage = analogRead(PSENSORPIN) * (5.0 / 1023.0);                                              //ADDED NEW LINE
  float voltage2 = pressurepin * (5.0 / 1023.0);
  pressurepinValue = (((voltage2 - 0.45) * (8000 - 0)/(3) + 0)/100)-(voltage + (voltage/2));
  pressurepinValues = (((((voltage2 - 0.5) / 4.5) * (pressureMax - pressureMin) + pressureMin)/1.99501247)- (voltage2/1.5) -1.8) + 3;
  dtostrf(pressurepinValue, 4, 1, buffer1);
  dtostrf(pressurepinValues, 4, 1, buffer2);
  Serial.println("Voltage: ");
  Serial.println(voltage);
  Serial.print("Pressure: ");
  Serial.print(buffer1);
  Serial.println(" PSI");
  delay(50);                      //delays ENTIRE system, number is in milliseconds
}


/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 
 //Assigns pins with output/input & flashes lights

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
  delay(1000);                        //delays ENTIRE system, number is in milliseconds
  digitalWrite(STATE0PIN, LOW);
  digitalWrite(STATE1PIN, LOW);
  digitalWrite(STATE2PIN, LOW);
  digitalWrite(STATE3PIN, LOW);
  digitalWrite(STATE4PIN, LOW);
  digitalWrite(STATE5PIN, LOW);
  digitalWrite(STATE6PIN, LOW);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 
//EEPROM reading for variables

void Read_EEPROM()   
{  
  maxPressure = EEPROMReadlong(addr);                       //Stored in EEPROM
  maxPressure = maxPressure / 400;                                //Converts the psi reading to proper psi
  Serial.print("MAX Pressure: ");
  Serial.print(maxPressure);
  Serial.println(" PSI");
  delay(10);                      //delays ENTIRE system, number is in milliseconds
  lcd.print("Max Pressure:");
  lcd.setCursor(0, 1);           //Sets where the LCD prints
  lcd.print(maxPressure);
  lcd.print(" PSI");
  delay(1000);                      //delays ENTIRE system, number is in milliseconds
  lcd.clear();                              //Clears LCD screen
  lcd.setCursor(0, 0);           //Sets where the LCD prints
 

  maxFilltime = EEPROMReadlong(addr1);              //Stored in EEPROM
  Serial.print("Fill Time: ");
  Serial.print(maxFilltime);
  Serial.println(" seconds");
  delay(10);                      //delays ENTIRE system, number is in milliseconds
  lcd.print("Fill Time:");
  lcd.setCursor(0, 1);           //Sets where the LCD prints
  lcd.print("60");
  lcd.print(" Seconds");
  delay(1000);                      //delays ENTIRE system, number is in milliseconds
  lcd.clear();                              //Clears LCD screen
  lcd.setCursor(0, 0);           //Sets where the LCD prints
  

  settleTime = 10;                            //Hardcoded value
  Serial.print("Settle Time: ");
  Serial.print(settleTime);
  Serial.println(" seconds");
  delay(10);                      //delays ENTIRE system, number is in milliseconds
  lcd.print("Settle Time:");
  lcd.setCursor(0, 1);           //Sets where the LCD prints
  lcd.print(settleTime);
  lcd.print(" Seconds");
  delay(1000);                      //delays ENTIRE system, number is in milliseconds
  lcd.clear();                              //Clears LCD screen
  lcd.setCursor(0, 0);           //Sets where the LCD prints
  
 
  measureTime = 5;                            //Hardcoded value
  Serial.print("Measure Time: ");
  Serial.print(measureTime);
  Serial.println(" seconds");
  delay(10);                      //delays ENTIRE system, number is in milliseconds
  lcd.print("Measure Time:");
  lcd.setCursor(0, 1);           //Sets where the LCD prints
  lcd.print(measureTime);
  lcd.print(" Seconds");
  delay(1000);                      //delays ENTIRE system, number is in milliseconds
  lcd.clear();                              //Clears LCD screen
  lcd.setCursor(0, 0);           //Sets where the LCD prints

  exhaust2Time = 900;                            //Hardcoded value
  Serial.print("Exhaust Time: ");
  Serial.print("900");
  Serial.println(" seconds");
  delay(10);                      //delays ENTIRE system, number is in milliseconds
  lcd.print("Exhaust Time:");
  lcd.setCursor(0, 1);           //Sets where the LCD prints
  lcd.print("900");
  lcd.print(" Seconds MAX");
  delay(1000);                      //delays ENTIRE system, number is in milliseconds
  lcd.clear();                              //Clears LCD screen
  lcd.setCursor(0, 0);           //Sets where the LCD prints


  volume = 22730000;
  volume = volume / 1000;
  Serial.print("Volume: ");
  Serial.print(volume);
  Serial.println(" cm^3");
  delay(10);                      //delays ENTIRE system, number is in milliseconds
  lcd.print("Volume:");
  lcd.setCursor(0, 1);           //Sets where the LCD prints
  lcd.print(volume);
  lcd.print(" cm3");
  delay(1000);                      //delays ENTIRE system, number is in milliseconds
  lcd.clear();                              //Clears LCD screen
  lcd.setCursor(0, 0);           //Sets where the LCD prints
}


/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 
//EEPROM writing

void WriteMaxPressure()
{
  EEPROMWritelong(addr,250); //Gives you the psi
}

void WriteMaxFillTime()
{
  EEPROMWritelong(addr1,60);  //Gives you the fill time
}


/*--------------------------------------------------------------------------------------------------------------------------------------------*/ 
//LEDs

void idleBlink()
{
    digitalWrite(STATE0PIN, HIGH);
    lcd.print(".");
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, LOW);
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, HIGH);
    lcd.print(".");
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, LOW);
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, HIGH);
    lcd.print(".");
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, LOW);
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, HIGH);
    lcd.print(".");
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, LOW);
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, HIGH);
    lcd.print(".");
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, LOW);
    delay(300);                      //delays ENTIRE system, number is in milliseconds
   
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
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, LOW);
    digitalWrite(STATE1PIN, LOW);
    digitalWrite(STATE2PIN, LOW);
    digitalWrite(STATE3PIN, LOW);
    digitalWrite(STATE4PIN, LOW);
    digitalWrite(STATE5PIN, LOW);
    digitalWrite(STATE6PIN, LOW);
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, HIGH);
    digitalWrite(STATE1PIN, HIGH);
    digitalWrite(STATE2PIN, HIGH);
    digitalWrite(STATE3PIN, HIGH);
    digitalWrite(STATE4PIN, HIGH);
    digitalWrite(STATE5PIN, HIGH);
    digitalWrite(STATE6PIN, HIGH);
    lcd.print(".");
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, LOW);
    digitalWrite(STATE1PIN, LOW);
    digitalWrite(STATE2PIN, LOW);
    digitalWrite(STATE3PIN, LOW);
    digitalWrite(STATE4PIN, LOW);
    digitalWrite(STATE5PIN, LOW);
    digitalWrite(STATE6PIN, LOW);
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, HIGH);
    digitalWrite(STATE1PIN, HIGH);
    digitalWrite(STATE2PIN, HIGH);
    digitalWrite(STATE3PIN, HIGH);
    digitalWrite(STATE4PIN, HIGH);
    digitalWrite(STATE5PIN, HIGH);
    digitalWrite(STATE6PIN, HIGH);
    lcd.print(".");
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, LOW);
    digitalWrite(STATE1PIN, LOW);
    digitalWrite(STATE2PIN, LOW);
    digitalWrite(STATE3PIN, LOW);
    digitalWrite(STATE4PIN, LOW);
    digitalWrite(STATE5PIN, LOW);
    digitalWrite(STATE6PIN, LOW);
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, HIGH);
    digitalWrite(STATE1PIN, HIGH);
    digitalWrite(STATE2PIN, HIGH);
    digitalWrite(STATE3PIN, HIGH);
    digitalWrite(STATE4PIN, HIGH);
    digitalWrite(STATE5PIN, HIGH);
    digitalWrite(STATE6PIN, HIGH);
    lcd.print(".");
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, LOW);
    digitalWrite(STATE1PIN, LOW);
    digitalWrite(STATE2PIN, LOW);
    digitalWrite(STATE3PIN, LOW);
    digitalWrite(STATE4PIN, LOW);
    digitalWrite(STATE5PIN, LOW);
    digitalWrite(STATE6PIN, LOW);
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, HIGH);
    digitalWrite(STATE1PIN, HIGH);
    digitalWrite(STATE2PIN, HIGH);
    digitalWrite(STATE3PIN, HIGH);
    digitalWrite(STATE4PIN, HIGH);
    digitalWrite(STATE5PIN, HIGH);
    digitalWrite(STATE6PIN, HIGH);
    lcd.print(".");
    delay(300);                      //delays ENTIRE system, number is in milliseconds
    digitalWrite(STATE0PIN, LOW);
    digitalWrite(STATE1PIN, LOW);
    digitalWrite(STATE2PIN, LOW);
    digitalWrite(STATE3PIN, LOW);
    digitalWrite(STATE4PIN, LOW);
    digitalWrite(STATE5PIN, LOW);
    digitalWrite(STATE6PIN, LOW);
    delay(300);                      //delays ENTIRE system, number is in milliseconds
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
    ads.begin(0x48);        //Analog to digital converter
    state = 0; 
    InitializeOutputs();                  //Initialize Outputs function call
    allOn();                          //All LEDs on
    Serial.begin(9600);                        //Baud rate
    Serial.println("<Tester Starting>");          //Prints to the serial monitor and println gives a newline
    lcd.begin(16, 2);                             //LCD screen size
    lcd.print("Hello");
    delay(500);                      //delays ENTIRE system, number is in milliseconds
    lcd.clear();                     //Clears the screen
    lcd.print("Software: v3.18");
    delay(500);                      //delays ENTIRE system, number is in milliseconds
    lcd.clear();                              //Clears LCD screen
    lcd.print("Coded by Roe");
    delay(500);                      //delays ENTIRE system, number is in milliseconds
    lcd.clear();                              //Clears LCD screen
    minHose2 = 2;              //Air hose specs
    maxHose2 = 4;
    lcd.print("Hose Length: ");
    lcd.setCursor(0,1);           //Sets where the LCD prints
    lcd.print(minHose2);
    lcd.print("-");
    lcd.print(maxHose2);
    lcd.print("ft");
    delay(500);                      //delays ENTIRE system, number is in milliseconds
    Serial.println("Current stored values in EEPROM:");
    lcd.clear();                              //Clears LCD screen
    lcd.print("Current Settings");
    delay(500);                      //delays ENTIRE system, number is in milliseconds
    lcd.clear();                              //Clears LCD screen
    Read_EEPROM();                  //Pulls the values from the EEPROM.h file
    lcd.setCursor(0, 0);           //Sets where the LCD prints
    lcd.print("Working my magic");
    lcd.setCursor(0, 1);           //Sets where the LCD prints
    lcd.print("Hold on");
    lcd.clear();                              //Clears LCD screen
    delay(100);                      //delays ENTIRE system, number is in milliseconds
    lcd.clear();                              //Clears LCD screen
    MeasurePressure();                //Measuring psi and voltage
    allOff();                         //Lights off function call
    allBlink();                     //Blinks lights

    
    delay(500);                      //delays ENTIRE system, number is in milliseconds
    lcd.clear();                              //Clears LCD screen
    allOff();                         //Lights off function call
  }
  
  //Goes straight into main loop

  
/*---------------------------------------------------------------------------------------------------------------------------------------------------------*/


void loop()                       //Starts the main loop
  {
    switch (state)                    //Allows program to switch between "modes"
      {
        case 0:  //Idle
        state0:
        MeasurePressure();                //Measuring psi and voltage
        while (voltage > 0.0)              //These use voltage instead of pressurepinValue
          {
            digitalWrite(STATE6PIN, HIGH);
            analogWrite(EXHAUSTPIN, 255);               //Analog write uses numbers as the current//255 means on or high
            MeasurePressure();                //Measuring psi and voltage
            Serial.println(voltage);
            lcd.print("Pre-Test");
            lcd.setCursor(10,0);               //Sets where the LCD prints
            lcd.print("PSI:");
            lcd.print(buffer1);
            lcd.setCursor(0,1);                 //Sets where the LCD prints
            lcd.print("Exhaust           ");
          
          if ((voltage) <= 0.45)                          //Added a new conversion for exhausting psi completely
            {
              analogWrite(EXHAUSTPIN, 255);
              digitalWrite(STATE6PIN, HIGH);
              delay(100);                      //delays ENTIRE system, number is in milliseconds
              analogWrite(EXHAUSTPIN, 0);
              digitalWrite(STATE6PIN, LOW);
              MeasurePressure();                //Measuring psi and voltage
              break;
            }
          }

        lcd.clear();                              //Clears LCD screen
        
          while (state == 0)                                                  //While idle//Continuously checks for any inputs
              { 
                analogWrite(EXHAUSTPIN, 0);                                   //Makes sure that the exhaust is turned off
                LEDStatus();                                                  //Turns on the correct LED for the case/state
                lcd.setCursor(0, 0);                          //Sets where the LCD prints
                lcd.print("Ready");
                lcd.setCursor(8,0);                         //Sets where the LCD prints
                lcd.print("PSI: .");
                MeasurePressure();                    //Measuring psi and voltage
                Serial.println(voltage);                                      //Prints voltage
                lcd.print(buffer1 + 3);                                        //Gives you the idle psi//3 is so the display value isn't negative
                lcd.setCursor(0, 1);                    //Sets where the LCD prints
                lcd.print("Press START");
                CheckButtons();
                if (start == LOW && exhaust == HIGH)                         //HIGH = off & LOW = on INVERSE LOGIC FOR IF & WHILE STATEMENTS
                  {
                    delay(5);                      //delays ENTIRE system, number is in milliseconds
                    CheckButtons();                                          //Checks again to ensure button is pressed
                    if (start == LOW)                                        //If start is on
                      {
                        lcd.clear();                              //Clears LCD screen
                        lcd.print("Starting");
                        delay(500);                      //delays ENTIRE system, number is in milliseconds
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
                    delay(10);                      //delays ENTIRE system, number is in milliseconds
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
                delay(10);                      //delays ENTIRE system, number is in milliseconds
                CheckButtons();                                              //Consitently checking for button inputs
                
               
              }

              
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/

        
        case 1:                          //Compressor output ON
        state1:
          lcd.clear();                              //Clears LCD screen
          lcd.print("Compressor ON"); 
          while (state == 1)                                //While compressor is on
              {  
                LEDStatus();                              //State 1 LED turned on
                CheckButtons();                                          //Checking button inputs
                 for (int f = 0; f <= maxFilltime; f++)         //Sets up the counter for fill time
                  {
                    digitalWrite(STATE1PIN, HIGH);
                    delay(100);                      //delays ENTIRE system, number is in milliseconds
                    CheckButtons();
                    if (start == HIGH && exhaust == LOW)     //If exhaust is pushed, test stops
                      {
                        digitalWrite(COMPPIN, LOW);
                        lcd.clear();                              //Clears LCD screen
                        lcd.print("Cancelled");
                        state = 6;
                        break;                  //Loop gets broken
                      }
                    MeasurePressure();                //Measuring psi and voltage
                    delay(200);                         //Delays are spread out to increase reaction to input from switch
                    CheckButtons();
                    digitalWrite(STATE1PIN, LOW);
                    lcd.setCursor(0,1);           //Sets where the LCD prints
                    delay(200);                      //delays ENTIRE system, number is in milliseconds
                    lcd.print(f+1);
                    lcd.print("/");
                    delay(200);                      //delays ENTIRE system, number is in milliseconds
                    lcd.print(maxFilltime);
                    digitalWrite(COMPPIN, HIGH);
                    lcd.setCursor(7, 1);                     //Sets where the LCD prints
                    lcd.print("PSI:");
                    lcd.print(buffer1);                    //Prints the psi value with a float
                    CheckButtons();
                  }
                 state = 2;
                 break;

                
              
                }
                  
              
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/  
        
        
        case 2:                 //Settle Time
        state2:
          lcd.clear();                              //Clears LCD screen
          lcd.print("Settle ");
          while (state == 2)
            {
              digitalWrite(COMPPIN, LOW);             //Turns off compressor 
              LEDStatus();
              delay(1000);                      //delays ENTIRE system, number is in milliseconds
              CheckButtons();
              while (start == HIGH && exhaust == HIGH)           //While no button inputs
                {
                  CheckButtons();
                  for (int b = 0; b <= settleTime; b++)          //Handles the settle time counter
                    {
                      digitalWrite(STATE2PIN, HIGH);         //State2pin turned ON
                      CheckButtons();                       //Checks inputs again
                      state = 2;                            //Ensures state is 2
                      MeasurePressure();                //Measuring psi and voltage 
                      lcd.setCursor(0,0);           //Sets where the LCD prints
                      lcd.print("Settle PSI:");
                      lcd.print(buffer1);                    //Prints the psi value with a float
                      delay(500);                       //delays ENTIRE system, number is in milliseconds
                      digitalWrite(STATE2PIN, HIGH);                  //State2pin on again for redundancy
                      delay(500);                       //delays ENTIRE system, number is in milliseconds
                      lcd.setCursor(0, 1);           //Sets where the LCD prints
                      lcd.print(b+1);          //Visual timer for the settle timer
                      lcd.print(" / ");
                      lcd.print(settleTime);       //Settle time is 10 seconds
                    }                             //Once b+1 reaches the settle time of 10, the for loop is completed

                  state = 3;               //State 3 entered right after for loop
                  break;  
                
              if (start == LOW && exhaust == LOW)           //If both start and exhaust are on system cancels//Only for system failure and user safety
                {
                  lcd.clear();                              //Clears LCD screen
                  lcd.print("Cancelled");
                  delay(500);                      //delays ENTIRE system, number is in milliseconds
                  state = 6;
                }
              if (start == HIGH && exhaust == LOW);         //If exhaust is on the settle cancels and state 6 is enetered
                {
                  lcd.clear();                              //Clears LCD screen
                  lcd.print("Cancelled");
                  delay(500);                      //delays ENTIRE system, number is in milliseconds
                  state = 6;
                    }
                }
            }

              
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/
        
        
        case 3:                     //Measure Time              
        state3:
          lcd.clear();                              //Clears LCD screen
          lcd.print("Measuring");
          delay(100);                      //delays ENTIRE system, number is in milliseconds
          while (state == 3)                   //While measuring
            { 
              LEDStatus();                //State 3 LED on
              CheckButtons();
              delay(500);                      //delays ENTIRE system, number is in milliseconds
              MeasurePressure();                //Measuring psi and voltage
              lcd.clear();                              //Clears LCD screen
              CheckButtons();                       //Checks for button inputs
              if (start == HIGH && exhaust == HIGH)          //If no button inputs
              {
                MeasurePressure();                //Measuring psi and voltage
                lcd.print("Final PSI:");
                lcd.setCursor(0, 1);           //Sets where the LCD prints
                lcd.print(buffer1);                    //Prints the psi value with a float
                digitalWrite(PASSPIN, HIGH);        //Turns the pass LED on once final PSI is displayed
                delay(100);                      //delays ENTIRE system, number is in milliseconds
              }
              if (start == HIGH && exhaust == LOW)           //If exhaust button is pushed
              {
                delay(100);                      //delays ENTIRE system, number is in milliseconds
                lcd.clear();                              //Clears LCD screen
                digitalWrite(PASSPIN, LOW);              //Turns off the pass LED
                state = 6;                         //Exhausts
                break;
              }
            }

                           
/*---------------------------------------------------------------------------------------------------------------------------------------------------------*/

  
       case 6:                       //Exhaust
       state6:
        lcd.clear();                              //Clears LCD screen
        digitalWrite(PASSPIN, LOW);
        lcd.print("Exhausting");
         while (state == 6)
            {   
              LEDStatus();
              for (int e = 0; e <= exhaust2Time; e++)                //Timer for exhaust
                {
                  digitalWrite(STATE6PIN, HIGH);
                  digitalWrite(EXHAUSTPIN, HIGH);
                  CheckButtons();                   //Looks for inputs
                  MeasurePressure();                //Measuring psi and voltage
                  delay(200);                      //delays ENTIRE system, number is in milliseconds
                  lcd.setCursor(8,1);           //Sets where the LCD prints
                  lcd.print("PSI:");
                  delay(200);                      //delays ENTIRE system, number is in milliseconds
                  lcd.print(buffer2);                //Added a new conversion for exhausting psi completely//TOOK OUT PRESSUREPINVALUE
                  delay(200);                      //delays ENTIRE system, number is in milliseconds
                  if (voltage <= 0.45)                        //This for loop will count to 300 before exiting unless it detects 0 psi
                    {
                      Serial.println(voltage);
                      digitalWrite(EXHAUSTPIN, HIGH);
                      delay(1000);                      //delays ENTIRE system, number is in milliseconds
                      digitalWrite(EXHAUSTPIN, LOW);
                      lcd.clear();                              //Clears LCD screen
                      lcd.print("Done");
                      delay(1000);                      //delays ENTIRE system, number is in milliseconds
                      lcd.clear();                              //Clears LCD screen
                      state = 0;
                      break;                  //Breaks out of loop
                    }
                  digitalWrite(STATE6PIN, HIGH);
                  delay(300);                      //delays ENTIRE system, number is in milliseconds
                  lcd.setCursor(0, 1);           //Sets where the LCD prints
                  lcd.print(e+1);                     //Adds one to the counter every second
                  lcd.print(" s");                    //For seconds
                  
                }
               
               digitalWrite(EXHAUSTPIN, LOW);
               state = 0;                                       //System gets shut down and returns the state back to IDLE
               break;
            }
              
     /*---------------------------------------------------------------------------------------------------------------------------------------------------------*/                                                                        

    }          //End Switch Case

}         //End Main Loop
