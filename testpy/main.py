import math
CNV_DEGTR = 3.14159265359/180
CNV_RTDEG = 180/3.14159265359
CNV_DEGTR = 3.14159265359/180
RADIUS  = 6371000



def calcDistance(alat,alon, blat, blon):

    t1 = alat * CNV_DEGTR
    t2 = blon * CNV_DEGTR
    dt = (alat - blat) * CNV_DEGTR
    dl = (alon - blon) * CNV_DEGTR

    ac = math.sin(dt / 2) * math.sin(dt / 2) + math.cos(t1) * math.cos(t2) * math.sin(dl / 2) * math.sin(dl / 2)
    c = 2 * math.atan2(math.sqrt(ac), math.sqrt(1 - ac))

    return RADIUS * c


def calculatePolyOffset(lat1, lon1, d, brng) :

	lat1 = lat1 * CNV_DEGTR
	lon1 = lon1 * CNV_DEGTR

	lat2 = math.asin(math.sin(lat1) * math.cos(d / RADIUS) + math.cos(lat1) * math.sin(d / RADIUS) * math.cos(brng))
	lon2 = lon1 + math.atan2(math.sin(brng) * math.sin(d / RADIUS) * math.cos(lat1), math.cos(d / RADIUS) - math.sin(lat1) * math.sin(lat2))

	print lat2 * CNV_RTDEG
	print lon2 * CNV_RTDEG

speed = 38.8769
latitude = 42.280859
lat = 'N'
longitude = 83.238056
lon = 'W'
angle = 260
fix = True
satellites = 10

print ("othr now : " +str(latitude * 100)+" , "+str(longitude*100))


# calculatePolyOffset(latitude, longitude,speed, angle)
print calcDistance(42.468164,-83.396571, 42.468165, -83.397091)


# othr now : 4228.0859 , -8323.8056
# othr ltr : 4228.0590 , -8323.7734


# othr now : 4228.0859 , 8323.8056
#            4228.0603, 8323.83788724