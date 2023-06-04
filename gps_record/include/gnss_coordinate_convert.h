#pragma once
#ifndef _IV_PERCEPTION_GNSS_CONVERT_
#define _IV_PERCEPTION_GNSS_CONVERT_

#include <math.h>
//double nmeaConvert2Lat(string lat)
//{
//	Console.WriteLine(lat);
//	double nmea_d = double.Parse(lat.Substring(0, 2));
//	double nmea_m = double.Parse(lat.Substring(2, 6));
//	return nmea_d + nmea_m / 60;
//}
//
//double nmeaConvert2Lon(string lon)
//{
//	Console.WriteLine(lon);
//	double nmea_d = double.Parse(lon.Substring(0, 3));
//	double nmea_m = double.Parse(lon.Substring(3, 7));
//	return nmea_d + nmea_m / 60;
//}

void gps2xy(double , double , double *, double *);
void GaussProjCal(double longitude, double latitude, double *X, double *Y);
void GaussProjInvCal(double X, double Y, double *longitude, double *latitude);

#endif // !_IV_PERCEPTION_GNSS_CONVERT_