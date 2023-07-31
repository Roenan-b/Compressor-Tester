/**
v1.09C
Fixed leak rate calculation
Increased compressor ON time for Cartee 16192
Increased exhaust pulse if overshooting 100psi

v1.13C
-Same as 1.09C but changed exhaust valve pulse timing and added gradual valve time increase
-Added leak rate value displayed on PASS\FAIL screen

v1.14C
-Fixed scenario where wouldn't fail for low pressure if it had already overshot
-Added a second try to run compressor when initially fails for no connection

v1.15C
-Same as 1.14C but changed the min pressure to 23 below max to accomodate 77psi

*/
// Declare external libraries
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
int start;
int exhaust;
int sensorAD;                               //???
int16_t adc0;                               //???
float voltage;
float pressure;
float maxPressure;
int maxFilltime;
int fillCounter;
int settleTime;
float initialPressure;
int measureTime;
float finalPressure;
float pressureDiff;
float leakRate;
float storedLeakrate;
float volume;
float acceptableRate;
float hoseLength;
int minHose;
int minHose2;
int maxHose;
int maxHose2;
int calLow;   //A/D Low                              //???
int calHigh;  //A/D High                             //???
float calLo;    //Pressure Low                       //???
float calHi;    //Pressure High                      //???
int exhaustTime;
int a = 0;                                           //???
int b = 0;                                           //???
int q;                                               //???
int done = 0;                                        //???
int overShoot;                                       //???
const byte numChars = 54;                               //???
char receivedChars[numChars];                               //???
boolean newData = false;                               //???
String convert;
bool compFail;
bool leakFail;
bool elecFail;
int increase = 0;
                        
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


void setup() 
  {
    ads.begin(0x48);
    state = 0;
    InitializeOutputs();                              //Initialize Outputs
    allOn();
    Serial.begin(9600);
    Serial.println("<Tester Starting>");
    lcd.begin(16, 2);
    lcd.print("Hello Matey :)");
    delay(1500);
    lcd.clear();
    lcd.print("Software: v2.1");
    delay(1500);
    lcd.clear();
    lcd.print("Coded by Roe");
    delay(2000);
    lcd.clear();
    volume = 1000;                                                                                 //VOLUME CHANGED//HARDCODED                    //CHANGED TO 1000
    volume = volume / 1000;
    hoseLength = ((volume - 8) / 0.656693) / 2.54;      //Total volume minus other parts volume divided by pi times radius squared converted to inches
    //minHose = hoseLength - 1;                           //FIGURE OUT IF IT'S NEEDED
    minHose2 = 2;
    //maxHose = hoseLength + 1;
    maxHose2 = 4;
    lcd.print("Hose Length: ");           //MESSED WITH THE HOSE STUFF
    lcd.setCursor(0,1);
    lcd.print(minHose2);
    lcd.print("-");
    lcd.print(maxHose2);
    lcd.print("ft");
    delay(5000);  
    Serial.println("Current stored values in EEPROM:");     //DON'T NEED THIS (MAYBE)
    lcd.clear();
    lcd.print("Current Settings");
    delay(500);
    lcd.clear();
    Read_EEPROM();
    lcd.setCursor(0, 0);
    lcd.print("Working my magic");
    lcd.setCursor(0, 1);
    lcd.print("Hold on");
    delay(1000);
    lcd.clear();
    MeasurePressure();
    PrintValues();
    Serial.println();
    allOff();
    allBlink();
    for (int c = 0; c < 5000; c++)
    {
      recvWithEndMarker();
    }
  /*  allOn();
    delay(500);
    Serial.println("<Starting Leak Tester With These Settings>");
    delay(10);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Starting test");
    lcd.setCursor(0, 1);
    lcd.print("w these settings");
    delay(500);
    lcd.clear();
    lcd.setCursor(0, 0);    */   
    Read_EEPROM();                                    //Read values from EEPROM
    
    delay(500);
    lcd.clear();
    allOff();
  }

void loop() 
  {
    
   
    switch (state)
      {
        case 0:  //Idle
        state0:
        lcd.clear();
        lcd.print("Exhausting");
        analogWrite(EXHAUSTPIN, 255); 
        delay(100);
        lcd.clear();
        lcd.print("Ready");
        lcd.setCursor(0, 1);
        lcd.print("Top Hose: 8-10in");
        
        
          while (state == 0)
              { 
                analogWrite(EXHAUSTPIN, 0);
                LEDStatus();
                //lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Ready");
                lcd.setCursor(0, 1);
                lcd.print("Top Hose: 8-10in");
                CheckButtons();
                if (start == LOW && exhaust == HIGH)
                  {
                    delay(5);
                    CheckButtons();
                    if (start == LOW)
                      {
                        lcd.clear();
                        lcd.print("Starting");
                        delay(500);
                        state = 1;
                        break;                                 
                      }
                    else
                      {
                        state = 0;  
                      }
                  }
                else if (start == HIGH && exhaust == LOW)
                  {
                    delay(10);
                    CheckButtons();
                    if (exhaust == LOW)
                      {
                        state = 6;
                        break;                                 
                      }
                    else
                      {
                        state = 0;  
                      }
                  }  
                delay(1000);
                digitalWrite(STATE0PIN, LOW);
                CheckButtons();
                if (start == LOW && exhaust == HIGH)
                  {
                    delay(10);
                    CheckButtons();
                    if (start == LOW)
                      {
                        lcd.clear();
                        lcd.print("Starting");
                        delay(500);
                        state = 1;
                        break;                                 
                      }
                    else
                      {
                        state = 0;  
                      }
                  }
                else if (start == HIGH && exhaust == LOW)
                  {
                    delay(10);
                    CheckButtons();
                    if (exhaust == LOW)
                      {
                        state = 6;
                        break;                                 
                      }
                    else
                      {
                        state = 0;  
                      }
                  }
                //lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("     ");                 
                delay(500);
                //lcd.setCursor(0, 1);
                //lcd.print("Hose: ");
                //lcd.print(minHose2 + 1);
                //lcd.print("-");
                //lcd.print(maxHose2 + 1);
                //lcd.print("in");
                a = 0;
                overShoot = 0;
                compFail = false;
                analogWrite(EXHAUSTPIN, 0);
                CheckButtons();
                
                if (start == LOW && exhaust == HIGH)
                  {
                    delay(10);
                    CheckButtons();
                    if (start == LOW)
                      {
                        lcd.clear();
                        lcd.print("Starting");
                        delay(500);
                        state = 1;
                        break;                                 
                      }
                    else
                      {
                        state = 0;  
                      }
                  }
                else if (start == HIGH && exhaust == LOW)
                  {
                    delay(10);
                    CheckButtons();
                    if (exhaust == LOW)
                      {
                        state = 6;
                        break;                                 
                      }
                    else
                      {
                        state = 0;  
                      }
                  }  
                else
                  {
                    state = 0;  
                  }
              }
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/
        case 1:  //Compressor output ON
        state1:
          lcd.clear();
          lcd.print("Compressor ON"); 
          increase = 22; 
          while (state == 1)
              {  
                LEDStatus();
                delay(500);
                CheckButtons();
                if (start == HIGH && exhaust == HIGH && overShoot == 0)
                  {
                    digitalWrite(COMPPIN, HIGH);
                    CheckButtons();
                    delay(425 * maxFilltime);       //350 for ViAir 16092  425 for Cartee 16092
                    digitalWrite(COMPPIN, LOW);                      
                  } 
                else if (start == LOW || exhaust == LOW)
                    {
                      lcd.clear();
                      lcd.print("Cancel");
                      delay(10);
                      if (start == LOW || exhaust == LOW);
                        {
                          state = 6;
                          break;
                        }
                    } 
                if (start == HIGH && exhaust == HIGH)               //CHANGE THIS?  OR REFORMAT
                    {
                      delay(500);
                      MeasurePressure();
                      PrintValues();
                      lcd.setCursor(9,1);
                      lcd.print(pressure, 4);
                      if (pressure < 5)
                        {
                          elecFail = true;
                          compFail = false;
                          leakFail = false;
                          state = 5;
                        }
                      else if (pressure >= 5 && pressure < maxPressure - 23 && overShoot == 0)          //If pressure is too low
                        {
                          compFail = true;
                          elecFail = false;
                          leakFail = false;
                          state = 5;                                                   //If fill time is reached, go to indicate a compressor fill failure
                        }  
                      else if (pressure <= maxPressure && pressure >= (maxPressure - 23))   //If pressure is in the acceptable range
                        {
                          compFail = false;
                          leakFail = false;
                          elecFail = false;
                          state = 2;                                                   //Go to the next step
                        }
                      else if (pressure > maxPressure)                                 //If pressure is too high
                        {
                          compFail = false;
                          leakFail = false;
                          elecFail = false;
                          lcd.clear();
                          lcd.print("Pressure Too Hi");
                          delay(500);
                          lcd.clear();
                          lcd.print("Exhausting");
                          analogWrite(EXHAUSTPIN, 255);                                //Pulse exhaust valve to bring pressure down under max pressure
                          delay(increase);  
                          analogWrite(EXHAUSTPIN, 0);
                          MeasurePressure();
                          lcd.setCursor(9,1);
                          lcd.print(pressure, 4);
                          delay(500);
                          overShoot = 1; 
                          state = 1;
                          increase = increase + 1;                                 
                        }  
                    }
                else
                  {
                    state = 1;    
                  }
                
                delay(500);     //Allow time for button release
                break;
              }
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/  
        case 2:  //Settle Time
        state2:
          lcd.clear();
          lcd.print("Settle ");
          while (state == 2)
            {
              digitalWrite(COMPPIN, LOW);  
              LEDStatus();
              delay(1000);
              CheckButtons();
              while (start == HIGH && exhaust == HIGH)
                {
                  CheckButtons();
                  for (int b = 0; b <= settleTime; b++)
                    {
                      digitalWrite(STATE2PIN, LOW);
                      CheckButtons();
                      state = 2;
                      MeasurePressure();
                      lcd.setCursor(0,0);
                      lcd.print("Settle P:");
                      lcd.print(pressure, 4);
                      delay(500); 
                      digitalWrite(STATE2PIN, HIGH);
                      delay(500);
                      MeasurePressure();
                      lcd.setCursor(0,0);
                      lcd.print("Settle P:");
                      lcd.print(pressure, 4);
                      delay(500); 
                      lcd.setCursor(0, 1);
                      lcd.print(b+1);
                      lcd.print(" / ");
                      lcd.print(settleTime);                                
                    }
                  state = 3;
                  break;  
                }
              if (start == LOW || exhaust == LOW)
                {
                  lcd.clear();
                  lcd.print("Cancel");
                  delay(500);
                  delay(10);
                  if (start == LOW || exhaust == LOW);
                    {
                      state = 6;
                    }
                }
              
              if (state = 3)
                {
                  break;
                }
            delay(500);     //Allow time for button release
            break;     
            }   
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------*/
        case 3:   //Measure Time              
        state3:
          lcd.clear();
          lcd.print("Measuring");
          while (state == 3)
            { 
              LEDStatus();
              delay(500);
              MeasurePressure();
              lcd.setCursor(9,1);
              lcd.print(pressure, 4);
              initialPressure = pressure;
              Serial.println(initialPressure);
              lcd.clear();
              lcd.print("InitPres:");
              lcd.print(initialPressure, 4); 
              if (initialPressure > maxPressure - 24)
                {  
                  CheckButtons();
                  while (start == HIGH && exhaust == HIGH)
                    {
                      CheckButtons();
                      Serial.println(measureTime);
                      for (int c = 0; c <= measureTime; c++)
                        {
                          digitalWrite(STATE3PIN, LOW);
                          CheckButtons();
                          MeasurePressure();
                          lcd.setCursor(9,1);
                          lcd.print(pressure, 4);
                          delay(500);
                          digitalWrite(STATE3PIN, HIGH);
                          MeasurePressure();
                          lcd.setCursor(9,1);
                          lcd.print(pressure, 4);
                          delay(500);
                          lcd.setCursor(0, 1);
                          lcd.print(c+1);
                          lcd.print(" / ");
                          lcd.print(measureTime);                                    
                        }
                      MeasurePressure();
                      lcd.setCursor(9,1);
                      lcd.print(pressure, 4);
                      finalPressure = pressure;
                      Serial.println(finalPressure);
                      lcd.setCursor(0,1);
                      lcd.print("FinalPres:");
                      lcd.print(finalPressure, 4);
                      delay(1000);   
                      state = 4;
                      break;  
                    }
                  if (start == LOW || exhaust == LOW)
                    {
                      lcd.clear();
                      lcd.print("Cancel");
                      delay(500);
                      delay(10);
                      if (start == LOW || exhaust == LOW);
                        {
                          state = 6;
                        }
                    }
              }
            else if (initialPressure <= maxPressure - 24)
              {
                leakFail = true;
                compFail = false;
                elecFail = false;
                state = 5;
                break;
              }
            else
              {
                state = 5;
                break;                                    
              }
              if (state = 4)
                {
                  break;
                }
            delay(500);     //Allow time for button release
            break;
            }
      
/*----------------------------------------------------------------------------------------------*/
        case 5:   //Pass or Fail
        state5:
          lcd.clear();
          lcd.print("Wait");                                          //CHANGED STATE NAME
          while (state == 5)
            { 
              LEDStatus();   
              CheckButtons();
              while (start == HIGH && exhaust == HIGH)
                {
                      CheckButtons();
                      if (leakFail == true && done == 0)
                        {
                          lcd.clear();
                          lcd.print("FAIL Losing Pres");
                          lcd.setCursor(0,1);
                          lcd.print("Large Leak");
                          digitalWrite(LEAKFAILPIN, HIGH);
                          digitalWrite(BUZZERPIN, HIGH);
                          delay(200);                      //Change delay to constant button check
                          digitalWrite(BUZZERPIN, LOW);
                          delay(200);
                          digitalWrite(BUZZERPIN, HIGH);
                          delay(200);                      //Change delay to constant button check
                          digitalWrite(BUZZERPIN, LOW);
                          done = 1;
                        }                        
                      else if (compFail == true && done == 0)
                        {
                          lcd.clear();
                          lcd.print("FAIL Low Pressur");
                          lcd.setCursor(0,1);
                          lcd.print("Bad Compressor");
                          digitalWrite(COMPFAILPIN, HIGH);
                          digitalWrite(BUZZERPIN, HIGH);
                          delay(200);                      //Change delay to constant button check
                          digitalWrite(BUZZERPIN, LOW);
                          delay(200);
                          digitalWrite(BUZZERPIN, HIGH);
                          delay(200);
                          digitalWrite(BUZZERPIN, LOW);
                          done = 1;
                        }
                      else if (elecFail == true && done == 0)
                        {
                          lcd.clear();
                          lcd.print("FAIL No Pressure");
                          lcd.setCursor(0,1);
                          lcd.print("Bad Connection");
                          digitalWrite(COMPFAILPIN, HIGH);
                          digitalWrite(BUZZERPIN, HIGH);
                          delay(200);                      //Change delay to constant button check
                          digitalWrite(BUZZERPIN, LOW);
                          delay(200);
                          digitalWrite(BUZZERPIN, HIGH);
                          delay(200);
                          digitalWrite(BUZZERPIN, LOW);
                          done = 1;
                        }
                      else if (((leakRate <= acceptableRate) && (leakRate >= (acceptableRate * -1))) && done == 0)
                        {
                          lcd.clear();
                          lcd.print("PASS!  ");
                          //lcd.print(leakRate);
                          lcd.setCursor(0,1);
                          lcd.print("Leak and Comp OK");
                          digitalWrite(PASSPIN, HIGH);
                          delay(1000);                      //Change delay to constant button check
                          done = 1;
                        }
                      else if (((leakRate > acceptableRate) || (leakRate < (acceptableRate * -1))) && done == 0)
                        {
                          lcd.clear();
                          lcd.print("FAIL!  ");
                          lcd.print(leakRate);
                          lcd.setCursor(0,1);
                          lcd.print("Leak Rate Too Hi");
                          digitalWrite(LEAKFAILPIN, HIGH);
                          digitalWrite(BUZZERPIN, HIGH);
                          delay(200);                      //Change delay to constant button check
                          digitalWrite(BUZZERPIN, LOW);
                          delay(200);
                          digitalWrite(BUZZERPIN, HIGH);
                          delay(200);                      //Change delay to constant button check
                          digitalWrite(BUZZERPIN, LOW);
                          done = 1;
                        }
                      else if (leakFail == true && done == 1)
                      {
                          delay(500);
                          digitalWrite(LEAKFAILPIN, LOW);
                          digitalWrite(PASSPIN, LOW);
                          digitalWrite(COMPFAILPIN, LOW);
                          delay(500);
                          digitalWrite(LEAKFAILPIN, HIGH);
                          digitalWrite(PASSPIN, LOW);
                          digitalWrite(COMPFAILPIN, LOW);
                      }
                      else if (compFail == true && done == 1)
                        {
                          delay(500);
                          digitalWrite(COMPFAILPIN, LOW);
                          digitalWrite(PASSPIN, LOW);
                          digitalWrite(LEAKFAILPIN, LOW);
                          delay(500);
                          digitalWrite(COMPFAILPIN, HIGH);
                          digitalWrite(PASSPIN, LOW);
                          digitalWrite(LEAKFAILPIN, LOW);
                        }
                      else if (elecFail == true && done == 1)
                        {
                          delay(500);
                          digitalWrite(COMPFAILPIN, LOW);
                          digitalWrite(PASSPIN, LOW);
                          digitalWrite(LEAKFAILPIN, LOW);
                          delay(500);
                          digitalWrite(COMPFAILPIN, HIGH);
                          digitalWrite(PASSPIN, LOW);
                          digitalWrite(LEAKFAILPIN, LOW);
                        }  
                      else if (((leakRate <= acceptableRate) && (leakRate >= (acceptableRate * -1))) && done == 1)
                        {
                          digitalWrite(PASSPIN, HIGH);
                          digitalWrite(COMPFAILPIN, LOW);
                          digitalWrite(LEAKFAILPIN, LOW);
                        }
                      else if (((leakRate > acceptableRate) || (leakRate < (acceptableRate * -1))) && done == 1)
                        {
                          delay(500);
                          digitalWrite(LEAKFAILPIN, LOW);
                          digitalWrite(PASSPIN, LOW);
                          digitalWrite(COMPFAILPIN, LOW);
                          delay(500);
                          digitalWrite(LEAKFAILPIN, HIGH);
                          digitalWrite(PASSPIN, LOW);
                          digitalWrite(COMPFAILPIN, LOW);
                        }    
                }
              if (start == LOW || exhaust == LOW)
                {
                  lcd.clear();
                  lcd.print("Test Finished");
                  delay(500);
                  delay(10);
                  if (start == LOW || exhaust == LOW)  
                    {
                      done = 0;
                      state = 6;
                      break;
                    }
                }
            }
      
     
/*------------------------------------------------------------------------------------------*/  

       case 6:    //Exhaust
       state6:
        lcd.clear();
        lcd.print("Exhausting");
         while (state == 6)
            {   
              LEDStatus();
              digitalWrite(COMPFAILPIN, LOW);
              digitalWrite(PASSPIN, LOW);
              digitalWrite(LEAKFAILPIN, LOW);   
              analogWrite(EXHAUSTPIN, 255);
              delay(exhaustTime * 1000);
              analogWrite(EXHAUSTPIN, 0);
              delay(100);
              state = 0;
              break;
            }
      
     
/*------------------------------------------------------------------------------------------*/                                                                           

    }                                                                                        //End Switch Case

}                                                                                            //End Main Loop

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
    else if (state == 4) //Display PSI
      {
        digitalWrite(STATE0PIN, LOW);
        digitalWrite(STATE1PIN, LOW);
        digitalWrite(STATE2PIN, LOW);
        digitalWrite(STATE3PIN, LOW);
        digitalWrite(STATE4PIN, HIGH);
        digitalWrite(STATE5PIN, LOW);
        digitalWrite(STATE6PIN, LOW);    
      }
    else if (state == 5) //Pass or Fail                   //MIGHT REMOVE
      {
        digitalWrite(STATE0PIN, LOW);
        digitalWrite(STATE1PIN, LOW);
        digitalWrite(STATE2PIN, LOW);
        digitalWrite(STATE3PIN, LOW);
        digitalWrite(STATE4PIN, LOW);
        digitalWrite(STATE5PIN, HIGH);
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
void EEPROMWritelong(int address, long value)                                                      //DATA SECTION//WHAT IS ALL THIS//
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

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }

void CheckButtons()
{
  start = digitalRead(startPin);
  exhaust = digitalRead(exhaustPin);  
}

void MeasurePressure()
{
        /*delay(5);
        value = ads1115.readADC_SingleEnded(0);
        sum = value;
        Serial.print("1:");
        Serial.println(value);
        Serial.println(sum);
        delay(10);
        value = ads1115.readADC_SingleEnded(0);
        sum2 = sum + value;        
        Serial.print("2:");
        Serial.println(value);
        Serial.println(sum2);
        delay(10);
        value = ads1115.readADC_SingleEnded(0);
        sum3 = sum2 + value;        
        Serial.print("3:");
        Serial.println(value);
        Serial.println(sum3);
        delay(10);
        value = ads1115.readADC_SingleEnded(0);
        sum4 = sum3 + value;
        Serial.print("4:");
        Serial.println(value);
        Serial.println(sum4);
        delay(10);
        value = ads1115.readADC_SingleEnded(0);
        sum5 = sum4 + value;        
        Serial.print("5:");
        Serial.println(value);
        Serial.println(sum5);
        delay(10);
        sample_count++;*/
  while (sample_count < NUM_SAMPLES)
      {
        sum += ads.readADC_SingleEnded(0);
        sample_count++;
        delay(10);
      }   

  //sensorAD = (float)sum / (float)NUM_SAMPLES;
  //sensorAD = analogRead(A0);
  //adc0 = ads1115.readADC_SingleEnded(0);
  //delay(10);
  //Serial.println(adc0);
  //delay(10);
  //Serial.println(sum / 10);
  //delay(10);
  //Serial.println(calLow);
  //delay(10);
  //Serial.println(calHigh);
  //delay(10);
  //float temp = (float)analogRead(A0) - (float)calLow;
  float total = sum / 10;
  float temp = (float)total - (float)calLow;
  float temp2 = (float)calHigh - (float)calLow;                  
  pressure = (temp * 100) / temp2;
  delay(10);
  Serial.println(pressure, 7);
  delay(10); 
  //voltage = (analogRead(A0) * 5.015) / 1024.0;
  voltage = (total * 0.1875) / 1000;  
  delay(10);
  Serial.println(voltage, 7);
  delay(10);
  sum = 0;
  sample_count = 0;
}

  
void CalculateLeakRate()                                                                  //MIGHT BE ABLE TO REMOVE
{
  float temp3 = (float(initialPressure) + 14.7);
  float temp4 = (float(finalPressure) + 14.7);  
  float temp5 = (float(temp3) * volume) / 14.7;
  float temp6 = (float(temp4) * volume) / 14.7;
  float temp7 = float(measureTime) * 1.645;
  float temp8 = temp7 / 60;
  leakRate = (temp5 - temp6) / temp8;
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

void PrintValues()
{
  Serial.print("Raw AD: ");
  Serial.println(ads.readADC_SingleEnded(0));
  //Serial.println(analogRead(A0));
  Serial.print("Voltage Reading: ");
  Serial.print(voltage, 7);
  Serial.println("V");
  Serial.print("Pressure Reading: ");
  Serial.print(pressure, 7);
  Serial.println("psi");
  delay(10);
}

void Read_EEPROM()
{
  maxPressure = EEPROMReadlong(addr);
  maxPressure = maxPressure / 1000;
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
  lcd.print(maxFilltime);
  lcd.print(" Seconds");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  
  settleTime = EEPROMReadlong(addr2);
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
  
  measureTime = EEPROMReadlong(addr3);
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
  
  /*acceptableRate = EEPROMReadlong(addr4);
  acceptableRate = acceptableRate / 1000;
  Serial.print("Acceptable Leak Rate: ");
  Serial.print(acceptableRate);
  Serial.println(" SCCM");
  delay(10);
  lcd.print("Max Leak Rate:");
  lcd.setCursor(0, 1);
  lcd.print(acceptableRate);
  lcd.print(" SCCM");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);

  */
  
  exhaustTime = EEPROMReadlong(addr5);
  Serial.print("Exhaust Time: ");
  Serial.print(exhaustTime);
  Serial.println(" seconds");
  delay(10);
  lcd.print("Exhaust Time:");
  lcd.setCursor(0, 1);
  lcd.print(exhaustTime);
  lcd.print(" Seconds");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);

  calLow = EEPROMReadlong(addr6);
  Serial.print("Calibrated Low A/D: ");
  Serial.print(calLow);
  Serial.println(" bits");
  delay(10);
  lcd.print("Cal Low A/D:");
  lcd.setCursor(0, 1);
  lcd.print(calLow);
  lcd.print(" Bits");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);

  calHigh = EEPROMReadlong(addr7);;
  Serial.print("Calibrated High A/D: ");
  Serial.print(calHigh);
  Serial.println(" bits");
  delay(10);
  lcd.print("Cal High A/D:");
  lcd.setCursor(0, 1);
  lcd.print(calHigh);
  lcd.print(" Bits");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);

  volume = 1000;                                                                                 //VOLUME CHANGED//HARDCODED
  volume = volume / 1000;
  Serial.print("Volume: ");
  Serial.print(volume);
  Serial.println(" cm3");
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
  calHi = calHi / 1000;
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

//storedLeakrate = EEPROMReadlong(addr11);
  storedLeakrate = storedLeakrate / 1000;
  Serial.print("Last Leak Rate: ");
  Serial.print(storedLeakrate, 4);
  Serial.println(" SCCM");
  delay(10);
  lcd.print("Last Leak Rate:");
  lcd.setCursor(0, 1);
  lcd.print(storedLeakrate, 4);
  lcd.print(" SCCM");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
}

void WriteMaxPressure()
{
  EEPROMWritelong(addr,maxPressure);
  Serial.print("NEW Max Pressure: ");
  Serial.print(EEPROMReadlong(addr)/1000);
  Serial.println(" PSI");
}

void WriteMaxFillTime()
{
  EEPROMWritelong(addr1,maxFilltime);
  Serial.print("NEW Max Fill Time: ");
  Serial.print(EEPROMReadlong(addr1));
  Serial.println(" seconds");
}

void WriteSettleTime()
{
  EEPROMWritelong(addr2,settleTime);
  Serial.print("NEW Settle Time: ");
  Serial.print(EEPROMReadlong(addr2));
  Serial.println(" seconds");
}

void WriteMeasureTime()
{
  EEPROMWritelong(addr3,measureTime);
  Serial.print("NEW Measure Time: ");
  Serial.print(EEPROMReadlong(addr3));
  Serial.println(" seconds");
}

void WriteAcceptableRate()
{
  EEPROMWritelong(addr4,float(acceptableRate));
  Serial.print("NEW Acceptable Leak Rate: ");
  Serial.print(EEPROMReadlong(addr4)/1000);
  Serial.println(" SCCM");
}

void WriteExhaustTime()
{
  EEPROMWritelong(addr5,exhaustTime);
  Serial.print("NEW Exhaust Time: ");
  Serial.print(EEPROMReadlong(addr5));
  Serial.println(" seconds");
}

void WriteCalLow()
{
  EEPROMWritelong(addr6,calLow);
  Serial.print("NEW Calibrated Low A/D: ");
  Serial.print(EEPROMReadlong(addr6));
  Serial.println(" bits");
}

void WriteCalHigh()
{
  EEPROMWritelong(addr7,calHigh);
  Serial.print("NEW Calibrated High A/D: ");
  Serial.print(EEPROMReadlong(addr7));
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
                                                                      //TOOK OUT EEPROM.H ADDRESS
  Serial.println(" cm3");
}

/*void WriteLeakRate()
{
  EEPROMWritelong(addr11,storedLeakrate);
  Serial.print("Last Leak Rate: ");
  Serial.print(EEPROMReadlong(addr11));
  Serial.println(" SCCM");
}
*/
void recvWithEndMarker() 
{
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && newData == false) 
    {
        rc = Serial.read();

        if (rc != endMarker) 
        {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else 
        {
            receivedChars[ndx] = '0'; // terminate the string
            ndx = 0;
            newData = true;
            Serial.print("String received!");
            Serial.println();
            convertData();            
            Serial.println();
        }
    }
}

void convertData()                                                                                    //MAYBE DELTE THIS
{
  convert = ((receivedChars[1] - '0') * 100 + (receivedChars[2] - '0') * 10 + (receivedChars[3] - '0'));
  long maxPressure;
  maxPressure = maxPressure * 1000;
  WriteMaxPressure();
  convert = ((receivedChars[5] - '0') * 100 + (receivedChars[6] - '0') * 10 + (receivedChars[7] - '0'));
  int maxFilltime;
  WriteMaxFillTime();
  convert = ((receivedChars[9] - '0') * 100 + (receivedChars[10] - '0') * 10 + (receivedChars[11] - '0'));
  int settleTime;
  WriteSettleTime();
  convert = ((receivedChars[13] - '0') * 100 + (receivedChars[14] - '0') * 10 + (receivedChars[15] - '0'));
  int measureTime;
  WriteMeasureTime();
  convert = ((receivedChars[17] - '0') + 0.1 * (receivedChars[19] - '0') + 0.01 * (receivedChars[20] - '0'));   //Character 36 is decimal point
  float acceptableRate;
  acceptableRate = acceptableRate * 1000;
  Serial.println(acceptableRate / 1000);
  WriteAcceptableRate();
  convert = ((receivedChars[22] - '0') * 100 + (receivedChars[23] - '0') * 10 + (receivedChars[24] - '0'));
  int exhaustTime;
  WriteExhaustTime();
  convert = ((receivedChars[26] - '0') * 10000 + (receivedChars[27] - '0') * 1000 + (receivedChars[28] - '0') * 100 + (receivedChars[29] - '0') * 10 + (receivedChars[30] - '0'));
  int calLow;
  Serial.println(calLow);
  WriteCalLow();
  convert = ((receivedChars[32] - '0') * 10000 + (receivedChars[33] - '0') * 1000 + (receivedChars[34] - '0') * 100 + (receivedChars[35] - '0') * 10 + (receivedChars[36] - '0'));
  //int CalMid;
//  WriteCalMid();
  convert = ((receivedChars[38] - '0') + 0.1 * (receivedChars[40] - '0') + 0.01 * (receivedChars[41] - '0'));
  float calHigh;
  Serial.println(calHigh);
  WriteCalHigh();
  convert = ((receivedChars[38] - '0') + 0.1*(receivedChars[40] - '0') + 0.01*(receivedChars[41] - '0'));
  float calLo;
  calLo = calLo * 1000;
  Serial.println(calLo/1000);
  WriteCalLo();
  convert = (100*(receivedChars[43] - '0') +10*(receivedChars[44] - '0') + (receivedChars[45] - '0') + 0.1*(receivedChars[47] - '0') + 0.01*(receivedChars[48] - '0'));
  float calHi;
  calHi = calHi * 1000;
  Serial.println(calHi/1000);
  WriteCalHi();
  convert = (100*(receivedChars[50] - '0') +10*(receivedChars[51] - '0') + (receivedChars[52] - '0'));
  float volume;
  volume = volume * 1000;
  Serial.println(volume/1000);
  WriteVolume();
  newData = false;   
}

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
    /*digitalWrite(STATE0PIN, HIGH);
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    delay(500);
    digitalWrite(STATE0PIN, HIGH);
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    delay(500);
    digitalWrite(STATE0PIN, HIGH);
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    delay(500);
    digitalWrite(STATE0PIN, HIGH);
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    delay(500);
    digitalWrite(STATE0PIN, HIGH);
    delay(500);
    digitalWrite(STATE0PIN, LOW);
    delay(500);*/
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
