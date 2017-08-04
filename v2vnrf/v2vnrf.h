#include <Math.h>
#include <stdbool.h>

/* ------------Constants-------------------- START */
#define RADIUS  6371000
#define HLW 1.5														// Half line width 1.5m default

#define TOF 12														// Time of scope in sec
#define TTW 10														// Time to warn in sec
#define TTS 6														// Time to stop in sec

#define NUM_SAFE_POLY_SIDES 1
#define CNV_KNMPS 0.514444445										// conversion from knots to meters per sec
#define CNV_RTDEG 180/3.14159265359									// conversion from degrees to radians
#define CNV_DEGTR 3.14159265359/180									// conversion from radians to degrees
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))	
#define BROADCAST_INTERVAL 200	 									//broadcast interval in millis


#define STOP 4
#define CAUTION 5
#define WATCH 6
#define RECVLED 9
#define NOTICETO 2000
#define RECVTO 10000
//#define false 0
//#define true 1

/* ------------Constants-------------------- END */

/* ------------ Structures-------------------- START */

typedef struct {
	double x;
	double y;
}Point;

typedef struct {
	double longitude;
	double latitude;
}geodot;

typedef struct {
	geodot ver[NUM_SAFE_POLY_SIDES == 1 ? 2: NUM_SAFE_POLY_SIDES];								// 0 -> nearleft, 1-> farleft, 2->farright, 3->nearright																	//time span
}cautionPoly;

typedef struct {
	long bsmId;
	geodot cpos;													//current position
	float heading;													//vehicle heading
	float speed;													//vehicle speed
	int vehId;														//vehicle id
	int events;														//vehicle events [bsm part 2]
}bsmf;

typedef struct {
	int severity;// = WATCH;
	long lastUpdate;// = 0;
}notification;

typedef struct {
	int speed;
	double latitudeDegrees;
	double longitudeDegrees;
	double angle;
	int fix;
	int satellites;
}S_GPS;
/* ------------ Structures-------------------- END */


/*------ Function Declarations ------------- START*/

#ifdef __cplusplus
extern "C" {
#endif
	extern void processBSMR(bsmf bsm, bsmf bsmr);
	//extern int v2pAppICW(bsmf bsm, bsmf bsmr);
	extern double dround(double value, int numdecimal);
	extern void notify(int severity, char* msg);

	extern bool willIntersect(geodot v1a, geodot v1b, geodot v2a, geodot v2b);
	extern bool doIntersect(Point p1, Point q1, Point p2, Point q2);
	extern bool get_line_intersection(geodot v1a, geodot v1b, geodot v2a, geodot v2b, geodot *p);
	extern void sprintint(char* msg, int i);
	extern void sprintdouble(char* msg, double i);
	extern void sprintstring(char* msg, char* i);
	extern void sprintgps(char* msg, geodot g);
#ifdef __cplusplus
}
#endif

/*------ Function Declarations ------------- END*/

