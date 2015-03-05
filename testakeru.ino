// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "Akeru.h"


struct sigfoxData {
  float lat;
  float lon;
  float alt;
} data;


SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);


#define DEEPSLEEPTIME 20 // sec
#define LOOKUPTIME 120 // sec
#define AKERUWAIT 600 // sec
#define GPSPWR 6
#define GPSECHO  false
#define OFF false
#define ON true

boolean backFromSleep = false;

uint32_t lookupTimer = millis();





void setup()  
{
  
  // Watchdog stuff
  disableReset();
  setupWatchdogForSleep(WDTO_1S);
  
  Serial.begin(9600);
  Serial.println("Setup");
  
  pinMode(GPSPWR, OUTPUT);
  digitalWrite(GPSPWR, HIGH);
  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_NOANTENNA);

  delay(1000); // Let the GPS warm up before sending actual commands
  mySerial.println(PMTK_Q_RELEASE);
  Serial.println("Setup done");
}

void loop()                    
{
  
  // Read characters from the GPS' serial and parse them.
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
    
  if (GPS.newNMEAreceived()) {
    char *stringptr = GPS.lastNMEA();
    if (!GPS.parse(stringptr))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  
  
  
  if (GPS.fix && GPS.altitude) {
    
    Serial.println("Got a fix, sending some data :)");
    
    data.lat = GPS.latitudeDegrees;
    data.lon = GPS.longitudeDegrees;
    data.alt = GPS.altitude;
    
    sendSigfoxData();
    deepSleep();
    
  } else {
    
    // There has been no fix for longer than the maximum lookuptime.
    // Send default data (keepalive) and go to sleep.
    if ((millis() - lookupTimer) / 1000 >= LOOKUPTIME) {
      
      Serial.println("No fix for long enough, sending defualt data and sleeping.");
      
      data.lat = 0;
      data.lon = 0;
      data.alt = 0;
      
      sendSigfoxData();
      deepSleep();
    }
  }
}

void powerGps (boolean on) {
  
  if (on) {
    
    digitalWrite(GPSPWR, HIGH);
    GPS.fix = false;
    
    
    GPS.begin(9600);
    
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    GPS.sendCommand(PGCMD_NOANTENNA);
    
    delay(1000); // Let it warm up

  } else {
    
    digitalWrite(GPSPWR, LOW);
    
  }
  
}

void sendSigfoxData () {
  
  powerGps(OFF);
  
  Akeru.begin();
  uint32_t timer = millis();
  
  while(!Akeru.isReady()) {
    if (timer / 1000 >= AKERUWAIT) {
      return;
    }
  }
  
  Akeru.send(&data, sizeof(data));
  delay(500); // Should not be necessary, but let's be safe here ...
  Akeru.end(); // This is a custom method that has been added manually. It simply calls _serial.end() and nothing more.
  
}



/****************************
 * Sleep & Power Management *
 ****************************/

void deepSleep() {
  
  Serial.println("Going to sleep");
  Serial.flush();
  
  powerDown();
  sleep(DEEPSLEEPTIME);
  powerUp();
  
  lookupTimer = millis();
  
  Serial.println("Back from sleep");
  
}

void powerDown() {
  powerGps(OFF);
}

void powerUp() {
  powerGps(ON);  
}

void sleep(int seconds) {

  byte oldADCSRA = ADCSRA;
  ADCSRA = 0;
  wdt_reset();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);     // sleep mode is set here

  sleep_enable();                          // enables the sleep bit in the mcucr register
  for (int i = 0 ; i < seconds ; i++) {
    sleep_mode();
  }
  sleep_disable();
  ADCSRA = oldADCSRA;
  
}




/***********************
 * watchdog management *
 ***********************/

void enableReset(uint8_t timeoutBits) {
  wdt_reset();             // restart the watchdog counter
  wdt_enable(timeoutBits); // enable software reset in "timeout"
}

void delayReset() {
  wdt_reset(); // restart the watchdog counter
}

void disableReset() {
  MCUSR = ~(1 << WDRF); // clear the reset flag
  wdt_disable();        // disable watchdog reset
}

void forceReset() {
  Serial.flush();
  enableReset(WDTO_1S);
  while (1);
}


int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void setupWatchdogForSleep(uint8_t durationBits) {
  uint8_t WDTCSR_BIT_3 = (durationBits >> 3) & 1; // WDP3 is bit 5 in WDTCSR so we handle it separately
  uint8_t WDTCSR_BITS_210 = durationBits & 7;

  cli();
  /*
  WDTCSR configuration:
  WDIE = 1: Interrupt Enable WDE = 1 :Reset Enable
  */

  // Enter Watchdog Configuration mode: WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  // Set Watchdog settings:
  WDTCSR = (1 << WDIE) | (0 << WDE) | (WDTCSR_BIT_3 << WDP3) | WDTCSR_BITS_210;

  sei();
}

ISR(WDT_vect) {
    // nothing to do in the interrupt
    // but declaration required
}
