#include "v2vnrf.h"

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

	//if (NUM_SAFE_POLY_SIDES == 1) {
	//	vpoly.ver[0] = calculatePolyOffset(latitude, longitude, TTR * s, h);								//far
	//}else

	if (NUM_SAFE_POLY_SIDES == 4) {
		//0->nearleft, 1->farleft, 2->farright, 3->nearright
		cpoly.ver[3] = calculatePolyOffset(latitude, longitude, HLW, fmod((h + 90), 360));					//near right
		cpoly.ver[0] = calculatePolyOffset(latitude, longitude, HLW, fmod(((h - 90) + 360), 360));			//near left
		geodot fp = calculatePolyOffset(latitude, longitude, TTR * s, h);								//far
		cpoly.ver[2] = calculatePolyOffset(fp.latitude, fp.longitude, HLW, fmod(h + 90, 360));				//far right
		cpoly.ver[1] = calculatePolyOffset(fp.latitude, fp.longitude, HLW, fmod(((h - 90) + 360), 360));	//far left
	}
	return cpoly;

}


int v2pAppICW(bsmf bsm ,bsmf bsmr) {
	cautionPoly rpoly = getTimePolygon(bsmr.cpos, bsmr.speed, bsmr.heading);
	cautionPoly vpoly = getTimePolygon(bsm.cpos, bsm.speed, bsm.heading);
	// check intersection of vploy with rpoly
	for (int i = 0; i < NUM_SAFE_POLY_SIDES; i++) {
		for (int j = 0; j < NUM_SAFE_POLY_SIDES; j++) {
			if (willIntersect(vpoly.ver[i], vpoly.ver[(i + 1) % NUM_SAFE_POLY_SIDES], rpoly.ver[j], rpoly.ver[(j + 1) % NUM_SAFE_POLY_SIDES])) {
				notify(STOP, "car some where");
				return false;
			}
		}
	}
}

void processBSMR(bsmf bsm, bsmf bsmr) {
	// UNO doesnt support multithreading, mutithread if possible for all the apps
	if (bsmr.speed > 1)
		v2pAppICW(bsm, bsmr);
}
