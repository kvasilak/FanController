
//Copyright 2024 by Jeith Vasilakes, Corvette Engineering
//
//This is a fan controller for an electronitally controlled fan clutch (mercedes)
//For use in the GMC motorhome
//
//It has two input triggers:
//1. a 5v active high trigger input that senses the alarm pulses from the trmp sensor alarm
//2. level detect for the ebl fan output
//
//it has 1 output:
// a 20hz 15 amp PWM output
//
//the output has 2 adjustable pwm duty cycles
//When the the trigger input is low, the duty cycle is low, 0 to 50%
//When the trigger is high the duty cycle is 50 to 100%
//The PWM values are set via 2 pots on the controller
//
//There is no hysteresis in the alarn from the gauge so I have a delay off timer to prevent 
//frequent cycling. there is a pot to control that time from 0 to 600 seconds
//
//PWM runs at 20 hz or a 50 millisecond period
//so a 50% duty cycle would be 25ms on and 25ms off
//


#include <PWM.h>

static const byte FanPWMOutPin          = 10;    // PWM out to drive the fan
static const byte AlarmInPin            = 2;    //3 khz alarm signal from the gauge
static const byte ECMInPin              = 4;    //level change from the ECM ( Fan On )
static const byte LowPWNPin             = 0;
static const byte HiPWMPin              = 1;
static const byte HysteresisPin         = 2;

static int32_t PWMFreq                  = 20; //frequency (in Hz)
static bool FanOn                       = false;
static bool AlarmOn                     = false;
static const uint32_t TimeToTurnOffFan  = 80; //(4 seconds / .05 seconds per interrupt)
static uint32_t OVFcount                = 0;
static const uint16_t AlarmValidCount   = 3; //how many milliseconds to debounce the alarm input
static uint32_t LastOnTime              = 0;
static uint32_t HysteresisTime          = 0; //seconds of fake hysterysis from adc2
static bool DoHystersis                 = false;
static uint32_t StartupTime             = (10 * 1000); //10 second startup
static int percent                      = 0;

//Alarm interrupt
//Gauge outputs a square wave with a 1/5 second period, .35 seconds on .15 seconds off
//
void Alarm()
{
   //rising edge so alarm is high
   if(HIGH == digitalRead(AlarmInPin))
   {
      //Serial.println("1");
      AlarmOn = true;
      OVFcount = 0;
   }
}

//called every 50ms due to the PWM frequency
ISR(TIMER1_OVF_vect)
{
  if((AlarmOn))
  {
    OVFcount++;

    if(OVFcount > 3) //150ms
    {
      FanOn = true;
    }
    else
    {
      //Is the alarm still on?
      if(LOW == digitalRead(AlarmInPin))
      {
        //nope, false trigger
        AlarmOn = false;
        OVFcount = 0;
      }
    }

    //It's beena than the alarm off time since the last time it went high
    //Back to low duty cycle
    if(OVFcount > TimeToTurnOffFan)
    {
      AlarmOn = false;
      FanOn = false;
      OVFcount = 0;
      //Serial.println("*");
    }
  }
}

void setup()
{
  Serial.begin(115200);

  Serial.println("Fan Controller\n(C) 2024 Corvette Engineering\n");

  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe(); 

  //sets the frequency for the specified pin
  SetPinFrequencySafe(FanPWMOutPin, PWMFreq);

  //interrupt for Alarm from gauge
  pinMode(AlarmInPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(AlarmInPin), Alarm, CHANGE); //RISING );

  pinMode(ECMInPin, INPUT_PULLUP);

  TIMSK1 |= B00000001;  // Enable Timer Overflow Interrupt
}

void loop()
{
  //wait 10 seconds to avoid false alarm signal during gauge startup
  //This allows the ECM to turn on right away
  if(millis() < StartupTime)
  {
    FanOn = false;
    DoHystersis = false;
  }

  //if the ECM 'Fan On' input is high or the alarm input is active do the high PWM
  bool ECMFanOn = (LOW == digitalRead(ECMInPin));

  if(FanOn || ECMFanOn) //only allow high pwm to vary from 50 to 100% (127 to 255)
  {
    //Serial.println("Fan High\n");
    percent  = analogRead(HiPWMPin) ; 	
    percent = map(percent, 0, 1023, 127, 255);  

    LastOnTime = millis();

    //no need to fake hystersis if the ECM is controling the fan
    if(ECMFanOn)
    {
      DoHystersis = false;
    }
    else
    {
      DoHystersis = true;
    }
  }
  else //only allow low pwm to go from 0 to 50% (0 to 127)
  {
    if(DoHystersis)
    {
      //Serial.println("h");

      int hval = analogRead(HysteresisPin);
      //map the adc value (0 tp 1023) to seconds(0 to 600 (10 minutes))
      HysteresisTime = map(hval, 0, 1023, 0, 600);

      HysteresisTime = HysteresisTime * 1000;

      //delay to fake Hysteresis since we only get an alarm signal
      //rollover handled by being unsigned math
      if(millis() - LastOnTime > HysteresisTime)
      {
        //Serial.println("Hysteresis complete\n");
        DoHystersis = false;
      }
    }
    else
    {
      percent  = analogRead(LowPWNPin) ; 	
      percent = map(percent, 0, 1023, 0, 127); 
    }
  }

  pwmWrite(FanPWMOutPin, percent);

  delay(100);     
}

