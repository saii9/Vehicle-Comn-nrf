import math
CNV_DEGTR = 3.14159265359/180
CNV_RTDEG = 180/3.14159265359
CNV_DEGTR = 3.14159265359/180
RADIUS  = 6371000



def calcDistance(alat,alon, blat, blon):

    phi1 = alat;
    phi2 = blat;
    dPhi = (blat - alat);
    dLambda = (blon - alon);
    a = math.sin(dPhi/2) * math.sin(dPhi/2) + math.cos(phi1) * math.cos(phi2) * math.sin(dLambda/2) * math.sin(dLambda/2);
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a));
    return RADIUS * c;    


def calculatePolyOffset(lat1, lon1, d, brng) :

	lat1 = lat1 * CNV_DEGTR
	lon1 = lon1 * CNV_DEGTR

	lat2 = math.asin(math.sin(lat1) * math.cos(d / RADIUS) + math.cos(lat1) * math.sin(d / RADIUS) * math.cos(brng))
	lon2 = lon1 + math.atan2(math.sin(brng) * math.sin(d / RADIUS) * math.cos(lat1), math.cos(d / RADIUS) - math.sin(lat1) * math.sin(lat2))

	print lat2 * CNV_RTDEG
	print lon2 * CNV_RTDEG

speed = 38.8769
latitude = 42.468158
lat = 'N'
longitude =  -83.396789
lon = 'W'
angle = 260
fix = True
satellites = 10

print ("othr now : " +str(latitude * 100)+" , "+str(longitude*100))


calculatePolyOffset(latitude, longitude, 234.5866699218, 0)
print calcDistance(42.468158 , -83.396789, 42.4677963256, -83.3996047973)
#42.468158 , -83.396789, 42.467899 , -83.396461


