// *************************************************************************************************************
//                                          MightyBoost control sample sketch
// *************************************************************************************************************
// Copyright Felix Rusu (2014), felix@lowpowerlab.com
// http://lowpowerlab.com/
// *************************************************************************************************************
// License
// *************************************************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 2 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE.  See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program; if not, write 
// to the Free Software Foundation, Inc.,                
// 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
//                                                        
// Licence can be viewed at                               
// http://www.fsf.org/licenses/gpl.txt                    
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// *************************************************************************************************************
// MightyBoost is a smart backup PSU controllable by Moteino, and this sketch is a sample control sketch to run
// MightyBoost in this mode.
// http://moteino.com
// http://github.com/lowpowerlab
// Be sure to check back for code updates and patches
// *************************************************************************************************************
// This sketch will provide control over the essential features of MightyBoost:
//   - provide switched 5V power to a sensitive load like RaspberryPi which should not lose power instantly
//   - Control the "5V*" output via Moteino+PowerButton (momentary tactile)
//   - Monitor input supply and switch to battery backup when external power is lost
//   - Monitor battery voltage and issue a shutdown signal when battery runs low
// This sketch may be extended to include integration with other LowPowerLab automation products
// *************************************************************************************************************
#define LED                 5     // LED pin, should be analog for fading effect (PWM)
#define BUTTON              3     // Power button pin
#define SIG_REQUESTHALT     6     // Signal to Pi to ask for a shutdown
#define SIG_OKTOCUTOFF     A0     // Signal from Pi that it's OK to cutoff power
                                  // !!NOTE!! Originally this was D7 but it was moved to A0 at least temporarily.
                                  // On MightyBoost R1 you need to connect D7 and A0 with a jumper wire.
                                  // The explanation for this is given here: http://lowpowerlab.com/mightyboost/#source
#define OUTPUT_5V           4     // HIGH on this pin will switch the "5V*" output ON
#define BATTERYSENSE       A7     // Sense VBAT_COND signal (when powered externally should read ~3.25v/3.3v (1000-1023), when external power is cutoff it should start reading around 2.85v/3.3v * 1023 ~= 880 (ratio given by 10k+4.7K divider from VBAT_COND = 1.47 multiplier)
                                  // hence the actual input voltage = analogRead(A7) * 0.00322 (3.3v/1024) * 1.47 (10k+4.7k voltage divider ratio)
                                  // when plugged in this should be 4.80v, nothing to worry about
                                  // when on battery power this should decrease from 4.15v (fully charged Lipoly) to 3.3v (discharged Lipoly)
                                  // trigger a shutdown to the target device once voltage is around 3.4v to allow 30sec safe shutdown
#define LOWBATTERYTHRESHOLD  3.7  // a shutdown will be triggered to the target device when battery voltage drops below this (Volts)

#define ButtonHoldTime       1800 // Button must be hold this many mseconds before a shutdown sequence is started (should be much less than PIForceShutdownDelay)
#define PIShutdownDelay_Min  6000 // will start checking the SIG_OKTOCUTOFF line after this long
#define PIShutdownDelay_Max 38000 // window of time in which SIG_OKTOCUTOFF is expected to go HIGH
                                  // should be at least 3000 more than Min
                                  // if nothing happens after this window, if button is 
                                  // still pressed, force cutoff power, otherwise switch back to normal ON state
#define PIForceShutdownDelay 6500 // when SIG_OKTOCUTOFF==0 (PI in unknown state): if button is held
                                  // for this long, force shutdown (this should be less than PIShutdownDelay_Max)
#define ShutdownFINALDELAY   4000 // after shutdown signal is received, delay for this long
                                  // to allow all PI LEDs to stop activity (pulse LED faster)

#define PRINTPERIOD              1000

int lastValidReading = 1;
unsigned long lastValidReadingTime = 0;
unsigned long now=0;
int PowerState = 0;
long lastPeriod = -1;
float systemVoltage = 5;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(SIG_OKTOCUTOFF, INPUT);
  pinMode(SIG_REQUESTHALT, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(OUTPUT_5V, OUTPUT);
  pinMode(A7, INPUT);
  digitalWrite(SIG_REQUESTHALT, LOW);//added after sudden shutdown quirks, DO NOT REMOVE!
  digitalWrite(OUTPUT_5V, LOW);//added after sudden shutdown quirks, DO NOT REMOVE!
}

void loop() {
  int reading = digitalRead(BUTTON);
  now = millis();
  digitalWrite(SIG_REQUESTHALT, LOW);//added after sudden shutdown quirks, DO NOT REMOVE!
  
  boolean batteryLow = systemVoltage < LOWBATTERYTHRESHOLD;
  
  if (batteryLow || reading != lastValidReading && now - lastValidReadingTime > 200) {
    lastValidReading = reading;
    lastValidReadingTime = now;
    //((PowerState==0 && ()) || (PowerState==1 && (now - lastValidReadingTime > ButtonHoldTime)))
    if (batteryLow || reading == 0)
    {
      //make sure the button is held down for at least 'ButtonHoldTime' before taking action (this is to avoid accidental button presses and consequently Pi shutdowns)
      now = millis();
      while (!batteryLow && (PowerState == 1 && millis()-now < ButtonHoldTime)) { delay(10); if (digitalRead(BUTTON) != 0) return; }
          
      //SIG_OKTOCUTOFF must be HIGH when Pi is ON. During boot, this will take a while to happen (till it executes the "shutdowncheck" script
      //so I dont want to cutoff power before it had a chance to fully boot up
      //if (batteryLow || (PowerState == 1 && digitalRead(SIG_OKTOCUTOFF)==1))
      if (batteryLow || (PowerState == 1 && analogRead(SIG_OKTOCUTOFF)>800))
      {
        // signal Pi to shutdown
        digitalWrite(SIG_REQUESTHALT, HIGH);

        //now wait for the Pi to signal back
        now = millis();
        float in, out;
        boolean forceShutdown = true;
        
        while (millis()-now < PIShutdownDelay_Max)
        {
          if (in > 6.283) in = 0;
          in += .00628;
          
          out = sin(in) * 127.5 + 127.5;
          analogWrite(LED,out);
          delayMicroseconds(1500);
          
          //account for force-shutdown action (if button held for PIForceShutdownDelay, then force shutdown regardless)
          if (millis()-now <= (PIForceShutdownDelay-ButtonHoldTime) && digitalRead(BUTTON) != 0)
            forceShutdown = false;
          if (millis()-now >= (PIForceShutdownDelay-ButtonHoldTime) && forceShutdown)
          {
            PowerState = 0;
            digitalWrite(LED, PowerState); //turn off LED to indicate power is being cutoff
            digitalWrite(OUTPUT_5V, PowerState); //digitalWrite(LED, PowerState); 
            break;
          }
          
          if (millis() - now > PIShutdownDelay_Min)
          {
            // Pi signaling OK to turn off
            //if (digitalRead(SIG_OKTOCUTOFF) == 0)
            if (analogRead(SIG_OKTOCUTOFF) < 800)
            {
              PowerState = 0;
              digitalWrite(LED, PowerState); //turn off LED to indicate power is being cutoff
              
              //delay(3500);   //takes about 3sec between SIG_OKTOCUTOFF going LOW and Pi LEDs activity to stop
              now = millis();
              while (millis()-now < ShutdownFINALDELAY)
              {
                if (in > 6.283) in = 0;
                in += .00628;
                
                out = sin(in) * 127.5 + 127.5;
                analogWrite(LED,out);
                delayMicroseconds(300);
              }
              
              digitalWrite(OUTPUT_5V, PowerState); //digitalWrite(LED, PowerState); 
              break;
            }
          }
        }
        
        // last chance: if power still on but button still pressed, force cutoff power
        if (PowerState == 1 && digitalRead(BUTTON) == 0)
        {
          PowerState = 0;
          digitalWrite(OUTPUT_5V, PowerState);
        }
        
        digitalWrite(SIG_REQUESTHALT, LOW);
      }
      //else if (PowerState == 1 && digitalRead(SIG_OKTOCUTOFF)==0)
      else if (PowerState == 1 && analogRead(SIG_OKTOCUTOFF)<800)
      {
        now = millis();
        unsigned long now2 = millis();
        int analogstep = 255 / ((PIForceShutdownDelay-ButtonHoldTime)/100); //every 500ms decrease LED intensity
        while (digitalRead(BUTTON) == 0)
        {
          if (millis()-now2 > 100)
          {
            analogWrite(LED, 255 - ((millis()-now)/100)*analogstep);
            now2 = millis();
          }
          if (millis()-now > PIForceShutdownDelay-ButtonHoldTime)
          {
            //TODO: add blinking here to signal final shutdown delay
            PowerState = 0;
            digitalWrite(OUTPUT_5V, PowerState);
            break;
          }
        }
      }
      else if (PowerState == 0)
      {
        PowerState = 1;
        digitalWrite(OUTPUT_5V, PowerState); //digitalWrite(LED, PowerState);
      }
    }
    
    digitalWrite(LED, PowerState);
  }
  
  int currPeriod = millis()/PRINTPERIOD;
  if (currPeriod != lastPeriod)
  {
    lastPeriod=currPeriod;
    Serial.print("VIN: ");
    systemVoltage = analogRead(BATTERYSENSE) * 0.00322 * 1.47;
    Serial.print(systemVoltage);
    if (systemVoltage > 4.3)
      Serial.println("  (plugged in)");
    else Serial.println("  (running from battery!)");
  }
}