/*********************************************************************************************************************************** 
 
Weather Station Firmware Version 1.0.0 
Written by: Muhammad Alli 
Date created: 05-12-2018 
Date modified last: 24-10-2019  
 
The purpose of this program is to obtain, log and display weather sensor data to a mobile interface. This was achieved through 
the use of a Particle Photon microcontroller, 3 sensors, a micro SD card reader, a micro SD card and a Bluetooth Low Energy module: 
   -BME280 sensor (Temperature, pressure, humidity) 
   -Davis 6410 sensor (Wind speed and direction) 
   -BGT WS-601ABS2 sensor (Rain) 
   -Robotdyn SD card module (micro SD card reader) 
   -Any 32GB, or smaller, micro SD card 
   -JDY08 BLE module 
The microcontroller interrogates the sensors and transmits the sensor data, via  BLE, to a mobile application created on Blynk. 
The toggling of a push button on the mobile application controls the logging of timestamped weather data onto the micro SD card.

***********************************************************************************************************************************/ 

 
#include <BlynkSimpleSerialBLE.h>                           //To enable Bluetooth communication - blynk Version 0.5.4
#include <CE_BME280.h>                                      //Version 1.0.1
#include <Particle.h>                                         
#include <Wire.h>  
#include "SdCardLogHandlerRK.h"                             //Version 0.1.0
#define Addr 0x77                                           //BME280 address (119 decimal) 
 
//For Blynk communication 
char auth[] = "c33adb3e3e8344b1bed36ec3c7c2317b";           //Unique authorisation token to allow communication between hardware  and mobile app 
void BlynkComms(void);                                      //Communicates hardware data to the mobile app 
BlynkTimer timer1;                                          //Create timer called timer1 to control push data to Blynk 
 
//The controlling subroutine 
void AllSubroutines(void); 
 
//Subroutines, timers and variables for wind sensor operation 
void WindsensorSetup(void);                                 //Subroutine to initialise the wind sensor 
void Revolution(void);                                      //Interrupt subroutine ISR 
void WindDirection(void);                                   //To get wind direction data 
void Windsensor(void);                                      //Subroutine to get wind sensor data 
int WindspeedPin=D2;                                        //Digital read pin to detect revolutions of wind speed sensor  
int DebounceTime=0; 
float SampleTime1=4000;                                     //Sample time in milliseconds (4000 = 4 seconds) 
float Speed=0;                                              //Wind speed in km/hr  
volatile int RevCount=0;                                    //Since the main program and ISR are sharing this variable 
int WindDirectionPin=A1;                                    //Analog read pin for wind direction 
int DetectPin1=A7;                                          //Analog pin used to detect the connection status of the wind sensor 
float AnalogValue=0;                                        //Voltage of the potentiometer from the wind direction sensor 
int RawValue=0; int DirectionNumber = 0; char Direction[15]; 
const char DirectionDescription[][15] =  
{ "None" 
, "North" 
, "North East" 
, "East" 
, "South East" , "South" 
, "South West" , "West" 
, "North West" 
, "Err: Batt Low" 
}; 
String WindsensorState="";                                  //Store the connection status of the wind sensor 
byte WindsensorConnected=0;                                 //Variable used to control the execution of the wind sensor setup  subroutine 
Timer timerWindSpeed(SampleTime1, WindSpeed);               //Use of timers to calculate wind speed, every 5 seconds. It has more precise timing than milli(s) 
 
 
//Subroutines and variables for BME280 operation  
void GetBME280SensorData(void);                              
float Temperature; float Pressure; float Humidity; 
byte BME280SensorConnected=0;                               //To control the execution of the setup code for the BME280 after its reconnection to the photon 
String BME280SensorState("");                               //To store the connection status of the sensor 
CE_BME280 bme;                                              //For I2C communication 
 
 
//Subroutines and variables responsible for battery status monitoring  
void BatteryStatus(void); float BatteryVoltage; 
float BatteryVoltageComparison=5;                           //Used to prevent battery fluctuation  
String BatteryVoltageLow="<3.36"; 
int BatteryPin=A0; 
signed long ChargeCounter=4501;                             //The variable that initiates timed charging and is responsible for the charging period  
double ChargeEndTime=0;                                     //The time the battery status must transition to "battery charged"              
long ChargeStartTime=0;                                     //The time when the battery reaches 4.2V 
int STAT1 = D3; int STAT2 = D4; 
WidgetLED ChargeLed(V12);                    
byte BatteryChargedNotification=1;                          //To allow battery charged notification to be executed once.   
byte BatteryChargingNotification=1;                         //To allow battery charging notification to be executed once. 
byte BatteryLowNotification=1;                              //To allow battery low notification to be executed once 
 
 
//Subroutine and variables to obtain and maintaining RTC time void GetTime(void); 
int  Day, Month, Year, Hours, Minutes, Seconds; 
double t=0;                                                 //Variable used to store the Unix time 
double RTCResetTime=43200;                                  //43200 seconds is equal to 12 hours 
double RTCStartTime=1;                               
double RTCEndTime=1;                           
byte RTCReset=0; 
byte RTCTimeAdjust=1;                                       //Controls the execution of calibrating the photon's RTC time after 12 hours have passed 
byte AdjustmentAmount=1;                                    //Reduce the RTC time by 1 second after 12 hours have passed  


//Subroutines and variables for the rain sensor operation 
void RainSensor(void); void Rain(void); 
void Tipping(void);                                         //Interrupt subroutine ISR 
void RainSensorSetup(void); int RainDetectorPin=D5;                  
int RainSensorPin=D6;                    
int DebounceTime1; 
int SampleTime2=900000;                                     //equivalent to 15 minutes 
float RainFall=0; byte RainSensorConnected=0; String RainSensorState; 
volatile int TipCount=0;                                    //Since the main program and ISR are sharing this variable 
Timer timer2(SampleTime2, Rain); 
 
 
// Data logging 
void SaveToSDCard(void);                                    //Control data logging 
byte DataLogger=0; byte CheckSDCard=0; byte SDCount=0; int DetectPin2=A6; 
void CheckDataLoggerPB(void);                               //Subroutine to maintain the state of the data logging push button on the app 
const int SD_CHIP_SELECT = A2; 
SdFat sd; 
SdCardPrintHandler printToCard(sd, SD_CHIP_SELECT, SD_SCK_HZ(4 * MHZ)); size_t counter = 0; 
 
 
//Subroutine responsible for displaying information on the serial monitor 
void DisplayOnSerialMonitor(void); 
 
 
//Subroutine for power save mode 
void PowerSaveMode(void); 
byte SleepTime=1;                                           //The period, in seconds, that the photon remains in stop mode 
int PowerSave1=0; int PowerSave2=0; 
 
 
SYSTEM_MODE(MANUAL);                                        //Switches off the Wi-Fi module on the photon upon start up 
  
void setup() { 
     
     
    Serial.begin(9600);                                     //Initialise serial communication for the serial monitor 
    Serial1.begin(115200);                                  //Initialise serial communication on the TX and RX pins with 115200 baudrate for JDY-08 BLE module   
    Blynk.begin(Serial1,auth);                              //Connect the Blynk application to the photon via Bluetooth      
    //Set GPIOs as inputs with internal pull/down resistors     
    pinMode(WindspeedPin,INPUT_PULLUP);                           
    pinMode(DetectPin1, INPUT_PULLDOWN);                         
    pinMode(DetectPin2, INPUT_PULLUP);                           
    pinMode(RainSensorPin,INPUT_PULLDOWN);                       
    pinMode(RainDetectorPin,INPUT_PULLDOWN);                 
 
    pinMode(STAT1, INPUT_PULLUP);                              
    pinMode(STAT2, INPUT_PULLUP);                         
     
    Blynk.notify("Weather Station: Online");      
    //Serial.println("Weather Station: Online"); 
    requestTime();                                          //Requests Blynk RTC time upon start-up of weather station     
    timer1.setInterval(3000L, AllSubroutines);              //Set timer1 to execute the "AllSubroutines" subroutine every 3 seconds      
     
} 
 
//Maintains communication between weather station app and hardware  
void loop() { 
     
    Blynk.run();                                            //Maintain connection to Blynk      
    timer1.run();                                           //Keep timer1 running 
     
} 
 
//Executes periodically when timer1 reaches 3 seconds 
void AllSubroutines(){ 
     
    GetTime();                                              //To set and maintain the photon's RTC time 
    Windsensor(); 
    RainSensor(); 
    GetBME280SensorData();                                    
    //DisplayOnSerialMonitor(); 
    SaveToSDCard();                                         //Data logging 
    BlynkComms();                                           //Called last to allow variables to be updated before they are transmitted to the mobile app 
    PowerSaveMode();                                        //Checks if stop mode should be activated or not 
     
} 
 
//Setting up the timer and initialisation of the interrupt responsible for wind sensor measurements 
void WindsensorSetup(){ 
      
    Speed=0;     RevCount=0;    
    timerWindSpeed.reset();                                 //The stopped timer will reset and with the next line of code it will start 
    timerWindSpeed.start();                                 //Initialize timer for wind speed calculation 
    attachInterrupt(WindspeedPin, Revolution, FALLING);     //Setup an interrupt that is triggered on the falling edge of the signal every time the reed switch closes   
    WindsensorConnected=1; 
    PowerSave1=0;                                           //Switch sleep mode off 
 
} 
 
//Primary wind sensor subroutine 
void Windsensor(){ 
          
    if(digitalRead(DetectPin1) == HIGH){                    //To detect if the wind sensor is connected to the weather station 
         
        if (WindsensorConnected == 0){                      //Setup wind sensor only the first time after it is detected 
         
            WindsensorSetup(); 
            WindsensorState="Connected";                                   
            //Serial.println("Windsensor is connected and setup is complete");     
     
        }          
        if(BatteryVoltage<=3.36){ 
             
            DirectionNumber=9; 
                     }         else{          
            if(Speed==0){                                   //If there is no wind speed then the wind direction must be blocked 
         
                DirectionNumber=0; 
                     } 
            else{                                                
         
                WindDirection();                            //Determine wind direction 
                 
            }         } 
    } 
    else if(digitalRead(DetectPin1) == LOW){                //If the sensor is not connected disable the timer and interrupt once  
         
        timerWindSpeed.stop();         
        detachInterrupt(WindspeedPin);         
        WindsensorConnected=0; 
        WindsensorState="Disconnected"; 
        RevCount=0; 
        Speed=0; 
        DirectionNumber=0; 
        //Serial.println("Windsensor is not connected"); 
        PowerSave1=1; 
         
    }      
    strcpy(Direction, DirectionDescription[DirectionNumber]);   //Store the wind direction measurement into variable "direction" 
    //Serial.printlnf("Wind direction is: %s", Direction); 
     
     
} 
 
//Interrupt driven subroutine that executes when a pulse is detected 
void Revolution(){ 
 
    //Software debounce solves reed switch false readings. 
    if(millis()-DebounceTime > 15){  
     
        RevCount=RevCount+1; 
        DebounceTime=millis(); 
     
    } 
 
} 
 
//Obtain wind speed  
void WindSpeed(){ 
      
    Speed=(3.621015*RevCount)/(SampleTime1/1000);           //Speed in km/hr. Formula adapted from technical manual: Davis says 1600 rev/hr ---> 1 mile/hour     
    RevCount=0; 
    timerWindSpeed.reset();                                 //Resets the timer to start from 0  
     
} 
  
//Obtaining wind direction 
void WindDirection(){ 
 
    RawValue = analogRead(WindDirectionPin); DirectionNumber = map(RawValue, 0, 4096, 1, 9); 
 
} 
 
 
//Temperature, humidity and pressure function 
void GetBME280SensorData(){ 
     
    //Check if the BME280 sensor is connected to the weather station 
    if(!bme.begin(Addr)){      	 
         
        BME280SensorState="Disconnected"; 	 	 	        //BME280 sensor is disconnected 
        BME280SensorConnected=0;    	 	 	        	//To prevent this 'if' statement from executing continuously 
        //Serial.println("\nBME280 sensor not connected");     
         
        //Since sensor is disconnected, zero all measurements to prevent false readings  
        Temperature = 0; 
        Humidity = 0; 
        Pressure = 0;        
 
    } 
    else if(bme.begin(Addr)){     	 	 	 	            //BME280 sensor is connected 
         
        //Allow the sensors readings to stabilise before considering the measurements         
        if(BME280SensorConnected==0){                                         
             
            BME280SensorState="Initialising...";            //To give the BME280 time to produce stable readings  
            BME280SensorConnected=1;     	 	 	        //To prevent this 'if' statement from executing continuously                        
            //Serial.println("BME280 sensor is initialising...");  
        } 
        else if(BME280SensorConnected==1){ 
             
            BME280SensorState="Connected"; 
            BME280SensorConnected=2;     	 	 	        //To prevent this 'if' statement from executing continuously                        
            //Serial.println("BME280 sensor is connected and setup is complete");  
        } 
        if(BME280SensorConnected==2){ 
     
            delay(30);     	 	 	 	 	                //To allow BME280 sensor time to initialise 
            Pressure = (bme.readPressure()/100);     	    //Pressure measurements in hPa 
            Temperature = bme.readTemperature(); 
            Humidity = bme.readHumidity(); 
         
        }     
    } 
 
}   
 
//Primary rain sensor subroutine 
void RainSensor(){ 
      
   if(digitalRead(RainDetectorPin)==HIGH){ 
     
        if(RainSensorConnected==0){ 
             
            RainSensorSetup();                                                     
            //Serial.println("Rain sensor is connected and setup is complete");              
            RainSensorState="Connected"; 
             
        } 
         
    } 
    //If the rain sensor is not connected disable the interrupt and timer      
    else{          
        detachInterrupt(RainSensorPin);        
        timer2.stop();         
        TipCount=0; 
        RainFall=0; 
        RainSensorConnected=0; 
        RainSensorState="Disconnected"; 
        //Serial.println("Rain sensor is not connected"); 
        PowerSave2=1;  
    } 
     
} 
 
//Rain sensor setup subroutine 
void RainSensorSetup(){          
    TipCount=0; 
    attachInterrupt(RainSensorPin, Tipping, RISING);       
    timer2.reset();     
    timer2.start();  
    RainSensorConnected=1;     
    PowerSave2=0;   
     
} 
  
//Interrupt driven subroutine that executes when a pulse is detected  
void Tipping(){ 
       
    //Software debounce 
    if(millis()-DebounceTime1 >15){  
     
        TipCount=TipCount+1; 
        DebounceTime1=millis(); 
         
    }   
} 
 
//Executes every 15 minutes to obtain rainfall measurements 
void Rain(){ 
     
    RainFall=((TipCount*0.2)*(3600000/SampleTime2));        //Rainfall in mm/hr      
    TipCount=0;     
    timer2.reset();     
    timer2.start(); 
 
} 
 
//Responsible for sending weather, sensor connection status and battery status data to the Blynk mobile application  
void BlynkComms(){ 
     
    //BME280 sensor data 
    Blynk.virtualWrite(V0, Temperature); 
    Blynk.virtualWrite(V1, Humidity); 
    Blynk.virtualWrite(V2, Pressure); 
    Blynk.virtualWrite(V8, BME280SensorState);              //Sensor connection status      
    //Wind sensor data 
    Blynk.virtualWrite(V5, Direction);  
    Blynk.virtualWrite(V6, Speed); 
    Blynk.virtualWrite(V9, WindsensorState);                 
    //Rainfall sensor data 
    Blynk.virtualWrite(V15, RainFall); 
    Blynk.virtualWrite(V16, RainSensorState);                 
    //Battery monitoring subroutine  
    BatteryStatus(); 
     
}    
  
void BatteryStatus(){ 
     
    BatteryVoltage=(((analogRead(BatteryPin)*4.2)/3510))+0.015;   	//Calibrated battery voltage      
    //Non charge related battery conditions 
    if((digitalRead(STAT1)==HIGH && digitalRead(STAT2)==HIGH) || (digitalRead(STAT1)==LOW && digitalRead(STAT2)==LOW)){         
        Blynk.setProperty(V13, "label", "Battery voltage"); 
        ChargeCounter=4501;                                                   
         
 
        BatteryChargedNotification=1; 
        BatteryChargingNotification=1; 
         
        if(BatteryVoltage <= BatteryVoltageComparison){ 
             
            BatteryVoltageComparison=BatteryVoltage; 
            //Serial.println("BatteryVoltageComparison=BatteryVoltage");  
        } 
        else if(BatteryVoltage > BatteryVoltageComparison){ 
         
            BatteryVoltage=BatteryVoltageComparison; 
            //Serial.println("BatteryVoltage=BatteryVoltageComparison");  
             
        } 
         
        //Check battery voltage        
        if(BatteryVoltage<=3.36){
            
            Blynk.setProperty(V12, "color", "#F00606");                     //Red LED  
            Blynk.setProperty(V12, "label", "Battery low");                 //Set the widget to display that the battery is low 
            //Serial.println("Battery critically low. Please connect charger immediately!"); 
            Blynk.notify("Battery critically low. Please connect charger immediately!"); 
            
        } 
        else if(BatteryVoltage<=3.4 && BatteryLowNotification==1){          //Low voltage condition at 3.4V    
        
            Blynk.setProperty(V12, "color", "#F00606");                     //Red LED 
            Blynk.setProperty(V12, "label", "Battery low");                 //Set the widget to display that the battery is low 
            //Serial.println("Battery is low. Charge battery"); 
            Blynk.notify("Battery is Low. Please connect charger!"); 
            BatteryLowNotification=0; 
            
        } 
         else if(BatteryVoltage>3.40){         
             
            Blynk.setProperty(V12, "color", "#23C48E");                     //Green LED       
            Blynk.setProperty(V12, "label", "Battery normal");              //Set the widget to display that the battery is normal 
            //Serial.println("Battery is Normal");  
         
        } 
    } 
    if(digitalRead(STAT1)==LOW && digitalRead(STAT2)==HIGH && ChargeCounter>0){      //Battery charging condition 
         
        BatteryVoltageComparison=5;                                                  //To ensure that this variable will be set   with the new battery voltage under normal battery operating conditions 
        BatteryChargedNotification=1; 
        BatteryLowNotification=1;                                                            
         
        if(BatteryChargingNotification==1){ 
          
            Blynk.notify("Battery charging"); 
            BatteryChargingNotification=0; 
         
        } 
         
        Blynk.setProperty(V12, "color", "#DFED00");                         //Yellow LED   
        Blynk.setProperty(V12, "label", "Battery charging");                //Set the widget to display that the battery is   charging 
        Blynk.setProperty(V13, "label", "Battery charge voltage"); 
          
        if(BatteryVoltage>=4.2 && (PowerSave1==0 || PowerSave2==0)){        //To ensure that the changing of the charge status by timing is only initiated when the power saving feature is not active             
            
            t=Time.now();  	 	                	 	 	                //Setting “t” to be the current time from the photon's RTC 
 
             
            if(ChargeCounter==4501){                                    
                 
                ChargeStartTime=t;                               
                ChargeCounter=4500;                                         //4500 seconds is equivalent to 1 hour and 15 minutes. 
                ChargeEndTime=ChargeStartTime+ChargeCounter;                //Determine the end of the charging period 
                             }             else{                      
                if(ChargeEndTime<=t){                      
                    ChargeCounter=-1; 
                     
                } 
                 
            } 
 
        } 
         
    } 
    if((digitalRead(STAT1)==HIGH && digitalRead(STAT2)==LOW) || (ChargeCounter<=0 && digitalRead(STAT1)==LOW && digitalRead(STAT2)==HIGH)){      // Battery charged conditions  
     
        //Resets the “ChargeCounter” if fully charged state was achieved by STAT pins             
        if(digitalRead(STAT1)==HIGH && digitalRead(STAT2)==LOW){     
            
            ChargeCounter=4501; 
         
        }    
         
        Blynk.setProperty(V12, "color", "#1A6BF4");                         //Blue LED on 
        Blynk.setProperty(V12, "label", "Battery charged");                 //Set the widget to state that the battery is charged 
        Blynk.setProperty(V13, "label", "Battery voltage");           
        //Serial.println("Battery is Charged"); 
 
        BatteryChargingNotification=1; 
         
        //Executes once 
        if(BatteryChargedNotification==1){ 
             
            Blynk.notify("Battery charged");                                 
            BatteryChargedNotification=0; 
 
} 
 
    } 
    //Updates the battery voltage on the display widget     
    if(BatteryVoltage<=3.36){ 
         
        Blynk.virtualWrite(V13, BatteryVoltageLow);                         //Display "<3.36V" in battery voltage display widget 
        //Serial.println("Battery voltage is <3.36V"); 
    }     
    else{ 
     
        Blynk.virtualWrite(V13, BatteryVoltage); 
        //Serial.printlnf("Battery voltage is %0.3f", BatteryVoltage); 
             
    } 
     
} 
 
 
//Executes when the data logger PB is toggled on the app  
BLYNK_WRITE(V14){                        
     
    DataLogger = param.asInt();                                             //The state of the PB    
 
    //Activate data logging     
    if(DataLogger==HIGH){              
         
         
        CheckSDCard=1; 
             }             else{ 
                    
        Blynk.notify("Data logger: Disabled"); 
        //Serial.printlnf("Save data to SD card: Disabled"); 
         
    } 
 
} 
 
//Primary subroutine that enables/disables data logging to the SD card 
void SaveToSDCard(){ 
     
    //Execute when the data logger feature is activated and the battery voltage is not critically low     
    if(DataLogger==1 && BatteryVoltage>3.36){        
     
        //Checks if the SD card is connected         
        if (digitalRead(DetectPin2)==HIGH){                         
             
             
            //Separate the data batches stored on the SD card              
            if(CheckSDCard==1){ 
             
                printToCard.println("\nNew batch of weather data,Date,Time,Temperature(C),Humidity(%),Pressure(hPa),Wind speed(km/hr),Wind direction,Rain fall(mm/hrr)");    //This will run once only when the SD is first activated 
                //Serial.println("\nNew batch of weather data"); 
                 
                if(!printToCard.getLastBeginResult()){ 
                     
                    Blynk.notify("Data logger initializing, please wait..."); 
                    Serial.println("Data logger initializing, please wait..."); 
                    CheckSDCard=1; 
                    SDCount++;                         
                    if(SDCount==10){                                            //If the data logger fails after 10 attempts then a fault indication is produced 
                        
                       Blynk.notify("Data logger failed"); 
                       Blynk.virtualWrite(V14,0);                               //Switch PB state to "Off” 
                       DataLogger=0;                                            //Disable the data logger 
                       SDCount=0;   
                            
                    }                 
                }                 
                else{ 
                     
                    Blynk.notify("Data logger: Enabled");              
                    Serial.println("Data logger: Enabled"); 
                    CheckSDCard=0; 
                    SDCount=0; 
                     
                }             } 
            else if(CheckSDCard==0){                                            //Log timestamped data to SD card 
 
                t=Time.now();  
                Seconds=Time.second(t); 
                Minutes=Time.minute(t); 
                Hours=Time.hour(t); 
                Day=Time.day(t); 
                Month=Time.month(t); 
                Year=Time.year(t); 
                 
                printToCard.printlnf(" ,%i/%i/%i,%i:%i:%i,%0.2f,%0.2f,%0.2f,%0.2f,%s,%0.2f", Day, Month, Year, Hours, Minutes, Seconds, Temperature, Humidity, Pressure, Speed, Direction, RainFall); 
                //Serial.printlnf("%i/%i/%i  %i:%i:%i  Temperature: %0.2f C, Humidity: %0.2f %, Pressure: %0.2f hPa, Windspeed: %0.2f km/hr, Wind direction: %s, Rainfall: %0.2f mm", Day, Month, Year, Hours, Minutes, Seconds,Temperature, Humidity, Pressure, Speed, Direction, RainFall); 
                 
            } 
        } 
         
        //Executes if the data logging is active/activated but no SD card is present in the SD card reader         
        else{    
         
            //Serial.printlnf("No SD card present");                
            Blynk.notify("No SD card present"); 
            Blynk.virtualWrite(V14,0);                                           
            DataLogger=0;                                                        
            SDCount=0;                                                                                
             
        } 
    } 
    //Disable data logger from activating when battery is critically low     
    else if(DataLogger==1 && BatteryVoltage<=3.36){        
             
        printToCard.println("Data logger de-activated. Battery is critically low. "); 
        Blynk.notify("Data logger de-activated. Battery is critically low. "); 
        Blynk.virtualWrite(V14,0); 
        DataLogger=0; 
             
    } 
     
} 
 
 
//Checks if sleep mode should be enabled 
void PowerSaveMode(){ 
     
    if(PowerSave1==1 & PowerSave2==1){                                          //Power saving mode activated when both the rain sensor and the wind sensor are disconnected from the station 
         
        System.sleep(DetectPin1,RISING,SleepTime);       
        CheckDataLoggerPB();                                                    //Update the state of the data logger PB after returning from stop mode 
 
    } 
     
} 
 
//obtain, set and maintain the photon's RTC time 
void GetTime(){ 
     
     
    if(RTCReset==0 && RTCEndTime<=Time.now()){ 
         
        requestTime();                 
        if(float(((Time.now()-RTCStartTime)/RTCTimeAdjust))>=RTCResetTime){     //The Photon's RTC is corrected every 12 hours  
 
            t=(Time.now()-AdjustmentAmount);                                    //Every 12 hours the time is adjusted, ONCE, by 1 second  
            Time.setTime(t);             RTCTimeAdjust++; 
             
        }  
    } 
    if(RTCReset==1 && RTCEndTime<=Time.now()){                                  //Executes once only, after the photon's RTC is reset by Blynk 
 
        RTCEndTime=RTCStartTime+RTCResetTime;                                   //The time at which the photon's RTC needs to be reset by Blynk 
        RTCReset=0;  
 
    } 
 
} 
 
//Requests Blynk Unix time 
void requestTime(){ 
     
    Blynk.sendInternal("rtc", "sync"); 
 
} 
 
//Obtaining and converting Unix time to normal date and time format  
BLYNK_WRITE(InternalPinRTC) { 
 
    t = param.asLong();                                                         //Unix time returned by Blynk    
    RTCStartTime=t;                                           
    Time.setTime(t);                                                            //Set the photon's RTC according to Blynk's RTC time      
    RTCReset=1;                                                                 //Controls the setting of the RTCEndTime  
    //Serial.println("Photon set to Blynk RTC time"); 
    RTCTimeAdjust=1;                                                            //Used to adjust the photon's RTC time when 12 hours of operation is reached 
 
     
} 
 
//Executes when the weather station hardware connects to the app   
BLYNK_CONNECTED(){ 
     
     
    Blynk.notify("Weather Station: Connected"); 
    //Serial.println("Weather Station: Connected"); 
 
    CheckDataLoggerPB();                                                        //Update the state of the data logger PB on the mobile app     
    //Ensures that the battery status notifications occur upon reconnection to the app so that they are not missed 
    BatteryChargedNotification=1; 
    BatteryChargingNotification=1; 
    BatteryLowNotification=1; 
     
} 
 
//Checks the last known state of the data logger PB and updates it on the app 
void CheckDataLoggerPB(){ 
     
    if(DataLogger==0){ 
             
        Blynk.virtualWrite(V14,0);    
        
    } 
    else if(DataLogger==1){ 
         
        Blynk.virtualWrite(V14,1); 
             
    }     
     
} 
 
//Displays all the sensor and connection status data to the serial monitor for debugging purposes 
void DisplayOnSerialMonitor(){ 
     
    Serial.printlnf("Temperature %f C", Temperature);                            
    Serial.printf("Humidity %f", Humidity); 
    Serial.print(" % "); 
    Serial.printlnf("\nPressure %f hPa", Pressure); 
    Serial.printlnf("Rev Count: %d", RevCount); 
    Serial.printlnf("Windspeed %f km/hr", Speed); 
    Serial.printlnf("Analog voltage: %fV", AnalogValue); 
    Serial.print("Wind direction: ");  
    Serial.print(Direction); 
    Serial.print("\nTemperature, pressure and humidity sensor are "); 
    Serial.print(BME280SensorState); 
    Serial.println(""); 
    Serial.print("Wind speed and direction sensors are "); 
    Serial.print(WindsensorState); 
    Serial.println(""); 
    Serial.print("Rain fall sensor is "); 
    Serial.print(RainSensorState); 
    Serial.println(""); 
     
} 
