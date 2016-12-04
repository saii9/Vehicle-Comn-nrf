#include <string.h>
#include "v2vnrf.h"

double calcDistance(geodot a, geodot b) {

	double t1 = a.latitude * CNV_DEGTR;
	double t2 = b.longitude * CNV_DEGTR;
	double dt = (a.latitude - b.latitude) * CNV_DEGTR;
	double dl = (a.longitude - b.longitude) * CNV_DEGTR;

	double ac = sin(dt / 2) * sin(dt / 2) + cos(t1) * cos(t2) * sin(dl / 2) * sin(dl / 2);
	double c = 2 * atan2(sqrt(ac), sqrt(1 - ac));

	return RADIUS * c;
}

geodot calculatePolyOffset(double lat1, double lon1, double d, double brng) {
	geodot gdot;

	//Serial.print("lat1----: "); Serial.println(lat1, 10);
	//Serial.print("long1---: "); Serial.println(lon1, 10);
	lat1 = lat1 * CNV_DEGTR;
	lon1 = lon1 * CNV_DEGTR;

	//Serial.print("lat1--aft: "); Serial.println(lat1, 10);
	//Serial.print("long1-aft: "); Serial.println(lon1, 10);

	double lat2 = asin(sin(lat1) * cos(d / RADIUS) + cos(lat1) * sin(d / RADIUS) * cos(brng));
	double lon2 = lon1 + atan2(sin(brng) * sin(d / RADIUS) * cos(lat1), cos(d / RADIUS) - sin(lat1) * sin(lat2));

	gdot.longitude = dround(lon2 * CNV_RTDEG, 5);
	gdot.latitude = dround(lat2 * CNV_RTDEG, 5);

	//Serial.print("lat2----: "); Serial.println(gdot.latitude, 10);
	//Serial.print("long2---: "); Serial.println(gdot.longitude, 10);

	return gdot;
}

cautionPoly getTimePolygon(geodot gd, double speed, double h /*Heading*/) {
	double latitude = gd.latitude;
	double longitude = gd.longitude;
	cautionPoly cpoly;
	double s = speed * CNV_KNMPS;


#if(NUM_SAFE_POLY_SIDES == 4)
	//0->nearleft, 1->farleft, 2->farright, 3->nearright
	cpoly.ver[3] = calculatePolyOffset(latitude, longitude, HLW, fmod((h + 90), 360));					//near right
	cpoly.ver[0] = calculatePolyOffset(latitude, longitude, HLW, fmod(((h - 90) + 360), 360));			//near left
	geodot fp = calculatePolyOffset(latitude, longitude, TTS * s, h);									//far
	cpoly.ver[2] = calculatePolyOffset(fp.latitude, fp.longitude, HLW, fmod(h + 90, 360));				//far right
	cpoly.ver[1] = calculatePolyOffset(fp.latitude, fp.longitude, HLW, fmod(((h - 90) + 360), 360));	//far left
#elif(NUM_SAFE_POLY_SIDES == 1)
	cpoly.ver[0] = gd;																					// current position
	cpoly.ver[1] = calculatePolyOffset(latitude, longitude, TTS * s, h);								// predicted position
#endif // 
	return cpoly;
}


int v2pAppICW(bsmf bsm, bsmf bsmr) {

	cautionPoly vpoly = getTimePolygon(bsm.cpos, bsm.speed, bsm.heading);
	cautionPoly rpoly = getTimePolygon(bsmr.cpos, bsmr.speed, bsmr.heading);


#if(NUM_SAFE_POLY_SIDES == 4)
	// check intersection of vploy with rpoly
	for (int i = 0; i < NUM_SAFE_POLY_SIDES; i++) {
		for (int j = 0; j < NUM_SAFE_POLY_SIDES; j++) {
			if (willIntersect(vpoly.ver[i], vpoly.ver[(i + 1) % NUM_SAFE_POLY_SIDES], rpoly.ver[j], rpoly.ver[(j + 1) % NUM_SAFE_POLY_SIDES])) {
				notify(STOP, "car some where");
				return false;
			}
		}
	}
#elif(NUM_SAFE_POLY_SIDES == 1)
	geodot *p = malloc(sizeof(geodot));
	if (get_line_intersection(vpoly.ver[0], vpoly.ver[1], rpoly.ver[0], rpoly.ver[1], p)) {
		
		double t = calcDistance(vpoly.ver[0], *p) / bsm.speed;
		
		sprintdouble("distance", calcDistance(vpoly.ver[0], *p));
		sprintdouble("speed", bsm.speed);
		sprintdouble("time",t);

		sprintdouble("intersection pt lat", p->latitude);
		sprintdouble("time", t);

		free(p);
		if (t < TTS) notify(STOP, "bake, car in proximity");
		else if (t < TTW) notify(CAUTION, "car approching");
		else notify(WATCH, "car somewhere");
		return false;
	}
	free(p);
#endif // 
}

void processBSMR(bsmf bsm, bsmf bsmr) {
	// UNO doesnt support multithreading, mutithread if possible for all the apps
	if (bsmr.speed > 1)
		v2pAppICW(bsm, bsmr);
}
