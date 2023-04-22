/*  Timer-based plant watering project
 *  Adam Gannon - July 2020
 */
 
#include <LiquidCrystal.h>
#include <Encoder.h>
#include "RTClib.h"
#include <avr/sleep.h>

/* Pin definitions */ 
const int rs = 13, en = 12, d4 = 11, d5 = 10, d6 = 9, d7 = 8; //Liquid crystal
const int inA = 7, inB = 6;
const int buttonPin = 2;
const int relayPin = 5;
const int sqwPin = 3;

// Initialize liquid crystal library with pin definitions
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
Encoder enc(inA, inB);

RTC_PCF8523 rtc;

long oldPosition  = -999;

// Button global variables
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

int maxWaitHours = 7*24;
int minWaitHours = 6;
int waitIncriment = 6;
int waitHoursStore = minWaitHours; //Storage

//Conveniently, 1 second ~= 1 ounce
int maxRelaySeconds = 60;
int minRelaySeconds = 1;
int relayIncriment = 1;
int relaySecondsStore = 5;


bool buttonFired, timerFired;

DateTime nextWatering;
DateTime lastWatering;

void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("testing 123");
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(sqwPin,INPUT_PULLUP);
  
  Serial.begin(9600);
  Serial.println("start");

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin,LOW);

  // Init real-time clock 
  rtc.begin();

  // Initialize last and next watering
  lastWatering = rtc.now(); 
  setNextWatering();

  rtc.deconfigureAllTimers();
  rtc.enableCountdownTimer(PCF8523_FrequencySecond, 10);  // 10 seconds
  
  
}

void loop() 
{

  screen0();
  enableSleep();
  if (buttonFired)
  {
    screen1();
    screen2();
    lcd.clear();
    lcd.print("Settings Saved.");
    delay(500); //Give user time to release button before interrupt resets
  }

  if (timerFired)
  {
    Serial.println("Timer check routine.");
    checkTimer();
  }

}

void setNextWatering()
{
 nextWatering = rtc.now() + TimeSpan(0,waitHoursStore,0,0);
 //nextWatering = rtc.now() + TimeSpan(0,0,waitHoursStore,0); //Minutes, testing
}

void enableSleep()
{
  // Attach interrupts and make sure they have time to register before sleep
  attachInterrupt(digitalPinToInterrupt(sqwPin), timerIsr, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonIsr, FALLING);
  delay(100);

  timerFired = false;
  buttonFired = false;

  // sleep_mode() is a blocking call
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();

  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(sqwPin));
  detachInterrupt(digitalPinToInterrupt(buttonPin));
  Serial.println("morning.");
}

void timerIsr()
{
  timerFired = true;
}

void buttonIsr()
{
  buttonFired = true;
}


void screen0()
{
  //FIXME: Something weird here with LCD writes
  char buffLine1[16],buffLine2[16];
  char buffHour1[4],buffHour2[4];

  Serial.println("0");
  
  // Get current time 
  DateTime now = rtc.now();

  // Format current time 
  formatHour(buffHour1,now.hour());
  sprintf(buffLine1,"Now : %2d/%2d %s",now.month(),now.day(),buffHour1);
  
  lcd.clear();//FIXME
  lcd.print(buffLine1);
  delay(10); //Small delay needed for LCD writes
  
  // Format next watering time
  formatHour(buffHour2,nextWatering.hour());
  sprintf(buffLine2,"Next: %2d/%2d %s",nextWatering.month(),nextWatering.day(),buffHour2);

  lcd.setCursor(0,1);
  lcd.print(buffLine2);
}


void triggerRelay()
{
  digitalWrite(relayPin,HIGH);
  delay(relaySecondsStore*1000);
  digitalWrite(relayPin,LOW);
}

void checkTimer()
{
  DateTime now = rtc.now();
  if (now > nextWatering)
  {
    Serial.println("WATERING");
    // Water now
    triggerRelay();
    delay(1000);
    lcd.begin(16, 2);
    lcd.clear();
    lcd.print("Done.");
    delay(1000);
    // Update timer for next watering interval 
    setNextWatering();
  }
}

void formatHour(char *buff, int hourVal)
{

  if (hourVal > 12)
  {
    sprintf(buff,"%2dPM",hourVal-12);
  }
  else if (hourVal == 0)
  {
    sprintf(buff,"%2dAM",12);
  }
  else
  {
    sprintf(buff,"%2dAM",hourVal);
  }
}


long readEncoder(int inc)
{
  long newPosition = enc.read();
  if (newPosition != oldPosition) 
  {  
    if (abs(newPosition-oldPosition) >= 4)
    {
      oldPosition = newPosition;
    }
  }
  return oldPosition;
  
}

int incrimentEncoder(int inc)
{
  long newPosition = enc.read();
  if (newPosition != oldPosition) 
  {  
    if ((newPosition-oldPosition)>=4)
    {
      oldPosition = newPosition;
      return inc;
    }
    else if ((oldPosition-newPosition)>=4)
    {
      oldPosition = newPosition;
      return -inc;   
    }
  }
  return 0;
}

void screen1()
{
  int screenSet = false;
  lcd.clear();
  lcd.print("Water Every:");

  // Load last set value of wait hours from storage
  int waitHours = waitHoursStore;
  
  // Wait until button has been pressed to confirm
  while (!screenSet) 
  {

    // Incriment wait hours, keeping within bounds 
    waitHours += incrimentEncoder(waitIncriment);
    if (waitHours < minWaitHours)
    {
      waitHours = minWaitHours;
    }
    else if (waitHours > maxWaitHours)
    {
      waitHours = maxWaitHours;
    }

    char data[16];
    if (waitHours > 36)
    {
      float waitDays = float(waitHours)/24;
      char floatBuffer[4];

      dtostrf(waitDays, 4, 2, floatBuffer);
      sprintf(data, "%s days   ",floatBuffer);
    }
    else
    {
      sprintf(data, "%2d hours   ",waitHours);
    }
    
    lcd.setCursor(0,1);
    lcd.print(data);

    
    
    screenSet = buttonPressed();
  }

  // Hours have been set, let's save this 
  waitHoursStore = waitHours;

  // Set the DateTime of next watering
  setNextWatering();
}


void screen2()
{
  char data[16];
  
  int screenSet = false;
  lcd.clear();
  lcd.print("Water With: ");

  int relaySeconds = relaySecondsStore;

  while(!screenSet)
  {
    // Incriment within bounds
    relaySeconds += incrimentEncoder(relayIncriment);
    if (relaySeconds < minRelaySeconds)
    {
      relaySeconds = minRelaySeconds;
    }
    else if (relaySeconds > maxRelaySeconds)
    {
      relaySeconds = maxRelaySeconds;
    }

    sprintf(data,"%2d ounces  ",relaySeconds);
    lcd.setCursor(0,1);
    lcd.print(data);

    screenSet = buttonPressed();
  }

  // Store variable 
  relaySecondsStore = relaySeconds;
}



/*  Check state of rotary button
 *  Return TRUE if button has been pressed
 */
bool buttonPressed()
{

  // No press detected, by default 
  bool returnState = false; 
  
  // Get current reading 
  int reading = digitalRead(buttonPin);

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) 
  {  
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  // Debounce 
  if ((millis() - lastDebounceTime) > debounceDelay) 
  {
    if (reading != buttonState) 
    {
      buttonState = reading;
      
      // Push is low, so register if pushed 
      if (buttonState == LOW)  
      {
        returnState = true;
      }
    }
  }
  
  //Store variable 
  lastButtonState = reading;
  return returnState;
}
