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
#include <stdlib.h>
#include <SdFat.h>

SdFat SD;

struct sigfoxData {
  float lat;
  float lon;
  float alt;
} data;

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false
#define DEEPSLEEPTIME 600 // sec
#define LOOKUPTIME 120 // sec

int gpsPowerPin = 8;
int sdPin = 10;
int fixPin = 7;
int fixSignal = 0;
boolean hardwareFix = false;
boolean usingInterrupt = false;
boolean newSentence = false;
boolean toggleSerial = false;
boolean toggleHardwareFix = false;
uint32_t timer = millis();
uint32_t fixTimer = millis();
uint32_t lookupTimer = millis();

SdFile logfile;
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
void setup()  
{
    
  disableReset();
  
  // watchdog, to wake up
  setupWatchdogForSleep(WDTO_1S);
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(9600);
 // Serial.println("BAdafruit GPS library basic test wat !");
  Serial.print("Setup");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(sdPin, OUTPUT);
  
  // see if the card is present and can be initialized:
  Serial.flush();
  if (!SD.begin(sdPin)) {
    Serial.println("Card failed, or not present");
    Serial.flush();
    // don't do anything more:
  } else {
    Serial.println("Card found !");
  }
  
  //logfile.open("bulb.log", O_RDWR);
  Serial.println("Starting up");

  pinMode(fixPin, INPUT);
  pinMode(gpsPowerPin, OUTPUT);
  
  digitalWrite(gpsPowerPin, HIGH);
  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_NOANTENNA);
  
  useInterrupt(true);

  // Wait 1 second for the modem to warm
  delay(2000);
  
  // Init modem
  Akeru.begin();
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  fixTimer = millis();
  Serial.println("Setup done");
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void getHardwareFix() { // Deduce fix status from fix pin.
  
  fixSignal = digitalRead(fixPin);
  uint32_t now = millis();
  
  if (hardwareFix) {
    if (!fixSignal) { fixTimer = now; }
    if (now - fixTimer > 400) {
      hardwareFix = false;
    }
  } else {    
    if (fixSignal) { fixTimer = now; }
    if (now - fixTimer > 1200) {
      hardwareFix = true;
    }
  }
}

void loop()                     // run over and over again
{
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  getHardwareFix();
  
  if (hardwareFix && !toggleHardwareFix) {
    
//    Serial.println("Got Fix");
    Serial.println("Got hardware Fix, starting up GPS communication");
    GPS.begin(9600);
  
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    GPS.sendCommand(PGCMD_NOANTENNA);
    
    useInterrupt(true);
    
    Akeru.begin();
    
    toggleHardwareFix = true;
  }
  if (!hardwareFix && toggleHardwareFix) { 
    toggleHardwareFix = false;
  }
  
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    char *stringptr = GPS.lastNMEA();
    
    if (!GPS.parse(stringptr))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    
    newSentence = true;
    Serial.println("Parsed GPS Sentence");
  }
  
  
  if ((millis() - lookupTimer) / 1000 >= LOOKUPTIME) {
    Serial.print("No fix for ");
    Serial.print(LOOKUPTIME);
    Serial.print(" sec, sending default data and sleeping.");
    Serial.println(" ");
    
    data.lat = 0;
    data.lon = 0;
    data.alt = 0;
    
    if (Akeru.isReady()) {
      Akeru.send(&data, sizeof(data));
      delay(1000);
    }
    
    deepSleep();
  }
  
  if (!GPS.fix) {
    
    int since = (millis() - lookupTimer) / 1000;
    
    if (!(since % 5) && !toggleSerial) {
      Serial.print("No Fix Since ");
      Serial.print((millis() - lookupTimer) / 1000);
      Serial.println(" sec");
      toggleSerial = true;
    }
    
    if (since % 5 > 0) {
      toggleSerial = false;
    }
    
    return;
  }
  
  Serial.println("Software Fix !");
  
  data.lat = GPS.latitudeDegrees;
  data.lon = GPS.longitudeDegrees;
  data.alt = GPS.altitude;
  
  if (Akeru.isReady() && data.lat != 0 && data.lon != 0 && newSentence) {
    
    Serial.println("Sending data ...");
    Akeru.send(&data, sizeof(data));
    newSentence = false;
    
    Serial.println("Going to sleep ...");
    
    delay(1000);
    deepSleep();
    
  } else {
    Serial.println("Sigfox isn't ready or there is no relevant data to send");
  }
  
}

void deepSleep() {
  
  Serial.println("Going to sleep");
  Serial.flush();
  powerDown();
  sleep(DEEPSLEEPTIME);
  powerUp();
  
  Serial.println("Back from sleep");
  
}

void powerDown() {
  useInterrupt(false);
  digitalWrite(gpsPowerPin, LOW);
}

void powerUp() {
  useInterrupt(true);
  digitalWrite(gpsPowerPin, HIGH);
  lookupTimer = millis();
  timer = millis();
  fixTimer = millis();
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
