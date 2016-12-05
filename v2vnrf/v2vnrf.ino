#include "v2vnrf.h"
#include <SoftwareSerial.h>
#include <SPI.h>
#include "RF24.h"
#include <Adafruit_GPS.h>
#include <time.h>
#include <stdlib.h>

int vehId = 1;
#define GPSECHO false
#define RELEASE true

#if(RELEASE)
	SoftwareSerial mySerial(3, 2); /*to read gps*/
	Adafruit_GPS GPS(&mySerial);
#else
	S_GPS GPS;	
	bool bsminit = false;
#endif

long lastBroadcast = 0;
bsmf bsm;
byte addresses[7] = { "AllVeh" };
cautionPoly mypoly;
notification notice = {WATCH, 0};
long lastRecieved = 0;

RF24 radio(7,8); /* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */


boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
void printGpsData();
void printBSM(bsmf vbsm);


void setupOther(){
	pinMode(RECVLED, OUTPUT);
	pinMode(STOP, OUTPUT);
	pinMode(WATCH, OUTPUT);
	pinMode(CAUTION, OUTPUT);
}


void setupRadio() {

	radio.begin();
	//radio.setPALevel(RF24_PA_LOW);          // Set the PA Level low to prevent power supply related issue. RF24_PA_MAX is default.
	radio.openWritingPipe(addresses);       // Open a writing and reading pipe on each radio, with opposite addresses
	radio.openReadingPipe(1, addresses);     // Start the radio listening for data
	radio.startListening();


}


void setupGPS() {

#if(RELEASE)
	// to setup the ada fruit GPS environment
	GPS.begin(9600);
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 10 Hz update rate
	GPS.sendCommand(PGCMD_ANTENNA);
	useInterrupt(true);
	delay(1000);
	mySerial.println(PMTK_Q_RELEASE);             // Ask for firmware version

#else
	// test setup for off road testing
	if (vehId == 1) {

		GPS.speed = 16;
		GPS.latitudeDegrees = 42.468159;
		GPS.longitudeDegrees = -83.396793;
		GPS.angle = 270;
		GPS.fix = true;
		GPS.satellites = 10;
	
	}else if (vehId == 2) {
	
		GPS.speed = 8;
		GPS.latitudeDegrees = 42.467890;
		GPS.longitudeDegrees = -83.397131;
		GPS.angle = 0;
		GPS.fix = true;
		GPS.satellites = 10;
	
	}
#endif
}


void setup() {
	srand(time(NULL));

	Serial.begin(115200);
	bsm.vehId = vehId;
	setupRadio();
	setupGPS();
	setupOther();
}


void notify(int severity, char* msg) {
	Serial.println(msg);
	if (millis() - notice.lastUpdate >= NOTICETO) {
		notice.severity = severity;
		notice.lastUpdate = millis();
		digitalWrite(severity, HIGH);
	}
	else if (notice.severity < severity) {
		notice.severity = severity;
		notice.lastUpdate = millis();
		digitalWrite(severity, HIGH);
	}
}


void updateBSM() {
	
	bsm.cpos.latitude = GPS.latitudeDegrees; //new
	bsm.cpos.longitude = GPS.longitudeDegrees;
	bsm.speed = GPS.speed * CNV_KNMPS > 1 ? GPS.speed * CNV_KNMPS : 0;
	bsm.heading = GPS.angle;
}

#if(RELEASE)
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

uint32_t timer = millis();
void checkGps(){
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  if (GPS.newNMEAreceived()) {
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return; 
	updateBSM();
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
	printGpsData();
  }
}
#endif

void broadcast(){

	radio.stopListening();                           // First, stop listening so we can talk.    
    //Serial.print(F("Now sending : "));
    unsigned long start_time = micros();             // Take the time, and send it.  This will block until complete
 
     if (!radio.write( &bsm, sizeof(bsmf) )){
       Serial.println(F("failed"));
	   delay(rand() % 20);
	 }

     //Serial.println(start_time);          
    radio.startListening();   // Now, continue listening
	bsm.bsmId++;
	/*
	Serial.println("Sending BSM");
	printMSG(bsm);
	*/
}


void checkRadio(){
	unsigned long got_time;
    if (millis() - lastBroadcast > BROADCAST_INTERVAL){
      broadcast();
      lastBroadcast = millis();
    }
    
    if( radio.available()){
      bsmf bsmr;
                                                                    // Variable for the received timestamp
      while (radio.available()) {                                   // While there is data ready
        radio.read( &bsmr, sizeof(bsmf) );				            // Get the payload
      }

	  digitalWrite(RECVLED, HIGH);
	  lastRecieved = millis();
	  Serial.println("receiving BSM");
	  //printBSM(bsmr);
	  processBSMR(bsm, bsmr);

	}
	/*
	else {
		Serial.println("radio not available to recieve");
	}
	*/
}


void loop() {
#if(RELEASE)
	checkGps();
#else
	if (!bsminit) {
		updateBSM();
		bsminit = true;
	}
#endif

	if (millis() - notice.lastUpdate >= NOTICETO) digitalWrite(notice.severity, LOW);
	if (millis() - lastRecieved > RECVTO) {
		//Serial.println("receiving low");
		digitalWrite(RECVLED, LOW);
	}

	if (GPS.fix) {
		checkRadio();
	}
 }


void printGpsData(){
	Serial.println(" ---------------start------------------------ ");
    /*Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    */
	if (GPS.fix) {
#if(RELEASE)
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
#endif      
	  Serial.print("Location (in degrees): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      //Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
	Serial.println(" --------------- end ------------------------ ");
}

void printBSM(bsmf vbsm) {
	/*if (vbsm.speed <= 1) {
	Serial.print(vbsm.vehId); Serial.print(" speed less than 1 : "); Serial.println(vbsm.speed);
	return;
	}*/
	Serial.print("bsmId   : "); Serial.println(vbsm.bsmId);
	Serial.print("Speed   : "); Serial.println(vbsm.speed);
	Serial.print("heading : "); Serial.println(vbsm.heading);
	Serial.print("vehId   : "); Serial.println(vbsm.vehId);
	Serial.print("lat     : "); Serial.println(vbsm.cpos.latitude, 10);
	Serial.print("long    : "); Serial.println(vbsm.cpos.longitude, 10);

	//for (int i = 0; i < NUM_SAFE_POLY_SIDES; i++) {
	//	Serial.print(i); Serial.print("lat    : "); Serial.println(vbsm.cpoly.ver[i].latitude, 10);
	//	Serial.print(i); Serial.print("long   : "); Serial.println(vbsm.cpoly.ver[i].longitude, 10);
	//}
}

void sprintint(char* msg, int i) {
	Serial.print(msg); Serial.print(" : "); Serial.println(i);
}

void sprintgps(char* msg, geodot g) {
	double lat = g.latitude;
	double lon = g.longitude;

	Serial.print(msg); Serial.print(" : "); 
	Serial.print(lat, 6);  Serial.print(" , "); Serial.println(lon, 6);

	/*
	Serial.print((int)lat/1, 4); Serial.print("  "); Serial.print((lat - floor(lat)) * 100, 4);
	Serial.print(" , ");
	Serial.print((int)lon/1, 4); Serial.print("  "); Serial.println((lon - floor(lon)) * 100, 4);
	*/
}

void sprintdouble(char* msg, double i) {
	Serial.print(msg); Serial.print(" : "); Serial.println(i,10);
}

void sprintstring(char* msg, char* i) {
	Serial.print(msg); Serial.print(" : "); Serial.println(i);
}