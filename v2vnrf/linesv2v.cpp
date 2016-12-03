// A C++ program to check if two given line segments intersect
#include "v2vnrf.h"
using namespace std;

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(Point p, Point q, Point r)
{
	if (q.x <= MAX(p.x, r.x) && q.x >= MIN(p.x, r.x) &&
		q.y <= MAX(p.y, r.y) && q.y >= MIN(p.y, r.y))
		return true;

	return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
bool orientation(Point p, Point q, Point r)
{
	// for details of below formula.
	int val = (q.y - p.y) * (r.x - q.x) -
		(q.x - p.x) * (r.y - q.y);

	if (val == 0) return 0;  // colinear

	return (val > 0) ? 1 : 2; // clock or counterclock wise
}

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
//bool doIntersect(Point p1, Point q1, Point p2, Point q2)

bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases
	// p1, q1 and p2 are colinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 and p2 are colinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are colinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are colinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases
}


bool willIntersect(geodot v1a, geodot v1b, geodot v2a, geodot v2b) {
	Point p1, p2, q1, q2;
	p1.x = v1a.latitude;
	p1.y = v1a.longitude;
	q1.x = v1b.latitude;
	q1.y = v1b.longitude;
	p2.x = v2a.latitude;
	p2.y = v2a.longitude;
	q1.x = v1a.latitude;
	q1.y = v1a.longitude;

	return doIntersect(p1, q1, p2, q2);
}


// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
// intersect the intersection point may be stored in the floats i_x and i_y.
int get_line_intersection(geodot v1a, geodot v1b, geodot v2a, geodot v2b, geodot *p)
{
	double p0_x = v1a.latitude;
	double  p0_y = v1a.longitude;
	double  p1_x = v1b.latitude;
	double  p1_y = v1a.longitude;
	double  p2_x = v2a.latitude;
	double  p2_y = v1a.longitude;
	double  p3_x = v2b.latitude;
	double  p3_y = v1a.longitude;
	double  *i_x = &p->latitude;
	double  *i_y = &p->longitude;


	double s1_x, s1_y, s2_x, s2_y;
	s1_x = p1_x - p0_x;     
	s1_y = p1_y - p0_y;
	s2_x = p3_x - p2_x;     
	s2_y = p3_y - p2_y;

	double s, t;
	s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
	t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

	if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
	{
		// Collision detected
		if (i_x != NULL)
			*i_x = p0_x + (t * s1_x);
		if (i_y != NULL)
			*i_y = p0_y + (t * s1_y);
		return 0;
	}

	return 1; // No collision
}