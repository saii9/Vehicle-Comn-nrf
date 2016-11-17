
/*
* Getting Started example sketch for nRF24L01+ radios
* This is a very basic example of how to send data from one node to another
* Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "v2vnrf.h"
#define GPSECHO  true

byte addresses[7] = { "AllVeh" };
bsmf bsm;
cautionPoly vpoly;
notification notice;
long lastRecieved = 0;

//S_GPS GPS = {38.8769, 42.28091, -83.23508, 90, true }; //vehicle 2  home left // com6
//char vehId[10] = "Vehicle2";

S_GPS GPS = {38.8769, 42.28077, -83.23482, 0, true}; // vehicle 1 infront home
char vehId = 1;

RF24 radio(7, 8); /* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
				  //SoftwareSerial mySerial(3, 2); /*to read gps*/
				  //Adafruit_GPS GPS(&mySerial);

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
void printGpsData();

long lastBroadcast = 0;

void setup() {
	bsm.vehId = vehId;
	Serial.begin(115200);
	pinMode(RECVLED, OUTPUT);
	pinMode(STOP, OUTPUT);
	pinMode(WATCH, OUTPUT);
	pinMode(CAUTION, OUTPUT);

	/*Radio setup Start*/
	radio.begin();

	//radio.setPALevel(RF24_PA_LOW);          // Set the PA Level low to prevent power supply related issue. RF24_PA_MAX is default.
	radio.openWritingPipe(addresses);       // Open a writing and reading pipe on each radio, with opposite addresses
	radio.openReadingPipe(1, addresses);     // Start the radio listening for data
	radio.startListening();
	/*Radio setup End*/
}

double dround(double value, int numdecimal) {
	return (value * pow(10, numdecimal)) / pow(10, numdecimal);
}

geodot calculatePolyOffset(double lat1, double lon1, double d, double brng) {
	geodot gdot;

//	Serial.print("lat1    : "); Serial.println(lat1, 10);
//	Serial.print("long1   : "); Serial.println(lon1, 10);
	lat1 = lat1 * CNV_DEGTR;
	lon1 = lon1 * CNV_DEGTR;


	double lat2 = asin(sin(lat1) * cos(d / RADIUS) + cos(lat1) * sin(d / RADIUS) * cos(brng)); 
	double lon2= lon1 + atan2(sin	(brng) * sin(d / RADIUS) * cos(lat1), cos(d / RADIUS) - sin(lat1) * sin(lat2));
	
	gdot.longitude = dround(lon2 * CNV_RTDEG, 5);
	gdot.latitude = dround(lat2 * CNV_RTDEG, 5);

//	Serial.print("lat2    : "); Serial.println(gdot.latitude, 10);
//	Serial.print("long2   : "); Serial.println(gdot.longitude, 10);

	return gdot;
}

void updateTimePolygon() {

	double s = GPS.speed * CNV_KNMPS;
	double h = GPS.angle;

	//0->nearleft, 1->farleft, 2->farright, 3->nearright
	vpoly.ver[3] = calculatePolyOffset(GPS.latitude, GPS.longitude, HLW, fmod((h + 90), 360));			//near right
	vpoly.ver[0] = calculatePolyOffset(GPS.latitude, GPS.longitude, HLW, fmod(((h - 90) + 360), 360));	//near left
	geodot fp = calculatePolyOffset(GPS.latitude, GPS.longitude, TTR * s, h);							//far
	vpoly.ver[2] = calculatePolyOffset(fp.latitude, fp.longitude, HLW, fmod(h + 90, 360));				//far right
	vpoly.ver[1] = calculatePolyOffset(fp.latitude, fp.longitude, HLW, fmod(((h - 90) + 360), 360));	//far left
}

void printMSG(bsmf vbsm) {

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

	for (int i = 0; i < 4; i++) {
		Serial.print(i); Serial.print("lat    : "); Serial.println(vbsm.cpoly.ver[i].latitude, 10);
		Serial.print(i); Serial.print("long   : "); Serial.println(vbsm.cpoly.ver[i].longitude, 10);
	}

}


void updateBSM() {
	bsm.cpos.latitude = dround(GPS.latitude, 5); //new
	bsm.cpos.longitude = dround(GPS.longitude, 5);
	bsm.speed = dround(GPS.speed * CNV_KNMPS, 5);
	bsm.heading = dround(GPS.angle, 5);
	updateTimePolygon();
	bsm.cpoly = vpoly;

}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it

uint32_t timer = millis();
void checkGps() {
	updateBSM();
}

void broadcast() {

	if (millis() - lastRecieved > RECVTO) {
		Serial.println("receiving low");
		digitalWrite(9, LOW);
	}
	radio.stopListening();                           // First, stop listening so we can talk.    

	//Serial.print(F("Now sending : "));
	unsigned long start_time = micros();             // Take the time, and send it.  This will block until complete
	bsm.bsmId++;
	if (!radio.write(&bsm, sizeof(bsmf))) {
		Serial.println(F("failed"));
	}
	//Serial.println(start_time);
	radio.startListening();                          // Now, continue listening

	Serial.println("Sending BSM");
	printMSG(bsm);
	/*
	unsigned long started_waiting_at = micros();               // Set up a timeout period, get the current microseconds
	boolean timeout = false;                                   // Set up a variable to indicate if a response was received or not

	while ( ! radio.available() ){                             // While nothing is received
	if (micros() - started_waiting_at > 200000 ){            // If waited longer than 200ms, indicate timeout and exit while loop
	timeout = true;
	break;
	}
	}

	if ( timeout ){                                             // Describe the results
	Serial.println(F("Failed, response timed out."));
	}else{
	unsigned long got_time;                                 // Grab the response, compare, and send to debugging spew
	radio.read( &got_time, sizeof(unsigned long) );
	unsigned long end_time = micros();

	// Spew it
	Serial.print(F("Sent "));
	Serial.print(start_time);
	Serial.print(F(", Got response "));
	Serial.print(got_time);
	Serial.print(F(", Round-trip delay "));
	Serial.print(end_time-start_time);
	Serial.println(F(" microseconds"));
	}
	*/
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

int v2pAppICW(bsmf bsmr) {
	cautionPoly rpoly = bsmr.cpoly;
	// check intersection of vploy with rpoly
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (willIntersect(vpoly.ver[i], vpoly.ver[(i + 1) % 4], rpoly.ver[j], rpoly.ver[(j + 1) % 4])) {
				notify(STOP, "car some where");
				return false;
			}
		}
	}
} 

void processBSMR(bsmf bsmr) {
	if (bsmr.speed > 1)
		v2pAppICW(bsmr);			// UNO doesnt support multithreading, mutithread if possible for all the apps
}

void checkRadio() {

	unsigned long got_time;
	if (millis() - lastBroadcast > BROADCAST_INTERVAL) {
		broadcast();
		lastBroadcast = millis();
	}

	if (radio.available()) {
		//Serial.print(F("Now receiving : "));
		bsmf bsmr;
		// Variable for the received timestamp
		while (radio.available()) {                                   // While there is data ready
			radio.read(&bsmr, sizeof(bsmf));				            // Get the payload
		}
		digitalWrite(RECVLED, HIGH);
		lastRecieved = millis();
		Serial.println("receiving BSM");
		printMSG(bsmr);
		processBSMR(bsmr);
		/* 
		long got_time = micros();
		radio.stopListening();                                        // First, stop listening so we can talk
		radio.write( &got_time, sizeof(unsigned long) );              // Send the final one back.
		radio.startListening();                                       // Now, resume listening so we catch the next packets.
		Serial.print(F("Sent response "));
		Serial.println(got_time);
		*/
	}
}


void loop() {
	if (millis() - notice.lastUpdate >= NOTICETO) digitalWrite(notice.severity, LOW);
	if (millis() - lastRecieved > RECVTO) {
		//Serial.println("receiving low");
		digitalWrite(RECVLED, LOW);

	}
	checkGps();
	if (GPS.fix) {
		checkRadio();
	}
}

/*
void printGpsData(){
Serial.print("\nTime: ");
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
if (GPS.fix) {
Serial.print("Location: ");
Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
Serial.print(", ");
Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
Serial.print("Location (in degrees, works with Google Maps): ");
Serial.print(GPS.latitudeDegrees, 4);
Serial.print(", ");
Serial.println(GPS.longitudeDegrees, 4);

Serial.print("Speed (knots): "); Serial.println(GPS.speed);
Serial.print("Angle: "); Serial.println(GPS.angle);
Serial.print("Altitude: "); Serial.println(GPS.altitude);
Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
}
}

*/