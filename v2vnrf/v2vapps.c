#include <string.h>
#include "v2vnrf.h"

double calcDistance(geodot dot1, geodot dot2) {
	/*double lat1 = dot1.latitude;
	double lon1 = dot1.longitude;
	double  lat2 = dot2.latitude;
	double  lon2 = dot2.longitude;
	double theta, dist;
	theta = lon1 - lon2;
	dist = sin(lat1) * sin(lat2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
	dist = acos(dist);
	dist = rad2deg(dist);
	dist = dist * 60 * 1.1515;
	*/

	double phi1 = dot1.latitude * CNV_DEGTR;
	double phi2 = dot2.latitude * CNV_DEGTR;
	double dPhi = (dot2.latitude - dot1.latitude) * CNV_DEGTR;
	double dLambda = (dot2.longitude - dot1.longitude) * CNV_DEGTR;
	double a = sin(dPhi / 2) * sin(dPhi / 2) + cos(phi1) * cos(phi2) * sin(dLambda / 2) * sin(dLambda / 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	return RADIUS * c;


}

geodot calculatePolyOffset(double lat1, double lon1, double d, double brng) {

	sprintdouble("lat1", lat1);
	sprintdouble("lon1", lon1);
	sprintdouble("brng", brng);
	sprintdouble("dis", d);

	geodot gdot;

	lat1 = lat1 * CNV_DEGTR;
	lon1 = lon1 * CNV_DEGTR;
	brng = brng * CNV_DEGTR;

	double lat2 = asin(sin(lat1) * cos(d / RADIUS) + cos(lat1) * sin(d / RADIUS) * cos(brng));
	double lon2 = lon1 + atan2(sin(brng) * sin(d / RADIUS) * cos(lat1), cos(d / RADIUS) - sin(lat1) * sin(lat2));
	
	lat2 = lat2 * CNV_RTDEG;
	lon2 = lon2 * CNV_RTDEG;


	sprintdouble("lat2", lat2);
	sprintdouble("lon2", lon2);


	gdot.longitude = lon2;
	gdot.latitude = lat2;
	return gdot;
}

cautionPoly getTimePolygon(geodot gd, double speed, double h) {
	double latitude = gd.latitude;
	double longitude = gd.longitude;
	cautionPoly cpoly; 

#if(NUM_SAFE_POLY_SIDES == 4)
	//0->nearleft, 1->farleft, 2->farright, 3->nearright
	cpoly.ver[3] = calculatePolyOffset(latitude, longitude, HLW, fmod((h + 90), 360));					//near right
	cpoly.ver[0] = calculatePolyOffset(latitude, longitude, HLW, fmod(((h - 90) + 360), 360));			//near left
	geodot fp = calculatePolyOffset(latitude, longitude, TTS * s, h);									//far
	cpoly.ver[2] = calculatePolyOffset(fp.latitude, fp.longitude, HLW, fmod(h + 90, 360));				//far right
	cpoly.ver[1] = calculatePolyOffset(fp.latitude, fp.longitude, HLW, fmod(((h - 90) + 360), 360));	//far left
#elif(NUM_SAFE_POLY_SIDES == 1)
	cpoly.ver[0] = gd;		
	cpoly.ver[1] = calculatePolyOffset(latitude, longitude, TOF * speed, h);								// predicted position
#endif // 
	return cpoly;
}


int v2pAppICW(bsmf bsm, bsmf bsmr) {
	
	sprintdouble("this veh", bsm.heading);
	cautionPoly vpoly = getTimePolygon(bsm.cpos, bsm.speed, bsm.heading);

	sprintdouble("othr veh", bsmr.heading);
	cautionPoly rpoly = getTimePolygon(bsmr.cpos, bsmr.speed, bsmr.heading);

	sprintdouble(" this len   : ", calcDistance(vpoly.ver[0], vpoly.ver[1]));
	sprintdouble(" recv len   : ", calcDistance(rpoly.ver[0], rpoly.ver[1]));

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
