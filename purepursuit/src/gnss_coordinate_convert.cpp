#include "../include/gnss_coordinate_convert.h"

void gps2xy(double J4, double K4, double *x, double *y)
{
    int L4 = (int)((K4 - 1.5) / 3) + 1;
    double M4 = K4 - (L4 * 3);
    double N4 = sin(J4 * 3.1415926536 / 180);
    double O4 = cos(J4 * 3.1415926536 / 180);
    double P4 = tan(J4 * 3.1415926536 / 180);
    double Q4 = 111134.8611 * J4 - N4 * O4 * (32005.7799 + 133.9238 * N4 * N4 + 0.6973 * N4 * N4 * N4 * N4 + 0.0039 * N4 * N4 * N4 * N4 * N4 * N4);
    double R4 = sqrt(0.006738525414683) * O4;
    double S4 = sqrt(1 + R4 * R4);
    double T4 = 6399698.901783 / S4;
    double U4 = (T4 / S4) / S4;
    double V4 = O4 * M4 * 3.1415926536 / 180;
    double W4 = 0.5 + (5 - P4 * P4 + 9 * R4 * R4 + 4 * R4 * R4 * R4 * R4) * V4 * V4 / 24;
    double X4 = V4 * V4 * V4 * V4 / 720 * (61 + (P4 * P4 - 58) * P4 * P4);
    double Y4 = 1 + V4 * V4 * (1 - P4 * P4 + R4 * R4) / 6;
    double Z4 = V4 * V4 * V4 * V4 * (5 - 18 * P4 * P4 * P4 * P4 * P4 * P4 + 14 * R4 * R4 - 58 * R4 * R4 * P4 * P4) / 120;

    *y = Q4 + T4 * P4 * V4 * V4 * (W4 + X4);
    *x = 500000 + T4 * V4 * (Y4 + Z4);
}
/*
//高斯投影由经纬度(Unit:DD)反算大地坐标(含带号，Unit:Metres)
void GaussProjCal(double longitude, double latitude, double *X, double *Y)
{
    int ProjNo = 0; int ZoneWide; ////带宽
    double longitude1, latitude1, longitude0, latitude0, X0, Y0, xval, yval;
    double a, f, e2, ee, NN, T, C, A, M, iPI;
    iPI = 0.0174532925199433; ////3.1415926535898/180.0;
    ZoneWide = 6; ////6度带宽
    a = 6378245.0; f = 1.0 / 298.3; //54年北京坐标系参数
                                    ////a=6378140.0; f=1/298.257; //80年西安坐标系参数
    ProjNo = (int)(longitude / ZoneWide);
    longitude0 = ProjNo * ZoneWide + ZoneWide / 2;
    longitude0 = longitude0 * iPI;
    latitude0 = 0;
    longitude1 = longitude * iPI; //经度转换为弧度
    latitude1 = latitude * iPI; //纬度转换为弧度
    e2 = 2 * f - f * f;
    ee = e2 * (1.0 - e2);
    NN = a / sqrt(1.0 - e2 * sin(latitude1)*sin(latitude1));
    T = tan(latitude1)*tan(latitude1);
    C = ee * cos(latitude1)*cos(latitude1);
    A = (longitude1 - longitude0)*cos(latitude1);
    M = a * ((1 - e2 / 4 - 3 * e2*e2 / 64 - 5 * e2*e2*e2 / 256)*latitude1 - (3 * e2 / 8 + 3 * e2*e2 / 32 + 45 * e2*e2
        *e2 / 1024)*sin(2 * latitude1)
        + (15 * e2*e2 / 256 + 45 * e2*e2*e2 / 1024)*sin(4 * latitude1) - (35 * e2*e2*e2 / 3072)*sin(6 * latitude1));
    xval = NN * (A + (1 - T + C)*A*A*A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ee)*A*A*A*A*A / 120);
    yval = M + NN * tan(latitude1)*(A*A / 2 + (5 - T + 9 * C + 4 * C*C)*A*A*A*A / 24
        + (61 - 58 * T + T * T + 600 * C - 330 * ee)*A*A*A*A*A*A / 720);
    X0 = 1000000L * (ProjNo + 1) + 500000L;
    Y0 = 0;
    xval = xval + X0; yval = yval + Y0;
    *X = xval;
    *Y = yval;
}
*/


//=======================zhaobo0904
#define PI  3.14159265358979
void GaussProjCal(double lon, double lat, double *X, double *Y)
{
    // 1975 年国际椭球体长半轴 a, 第一离心率 e2, 第二离心率 ep2
    double a = 6378140.0;
    double e2 = 0.006694384999588;
    double ep2 = e2/(1-e2);

    // 原点所在经度
    double lon_origin = 6.0*int(lon/6) + 3.0;
    lon_origin *= PI / 180.0;

    double k0 = 0.9996;

    // 角度转弧度
    double lat1 = lat * PI / 180.0;
    double lon1 = lon * PI / 180.0;


    // 经线在该点处的曲率半径,
    double N = a / sqrt(1 - e2*sin(lat1)*sin(lat1));


    // 赤道到该点的经线长度近似值 M, 使用泰勒展开逐项积分然后取前四项.
    // 这个近似值是将 N 作为纬度 \phi 的函数展开为泰勒计数, 然后在区间 [0, lat1] 上积分得到的.
    // 首先计算前四项的系数 a1~a4.
    double a1 = 1 - e2/4 - (3*e2*e2)/64 - (5*e2*e2*e2)/256;
    double a2 = (3*e2)/8 + (3*e2*e2)/32 + (45*e2*e2*e2)/1024;
    double a3 = (15*e2*e2)/256 + (45*e2*e2*e2)/1024;
    double a4 = (35*e2*e2*e2)/3072;
    double M = a * (a1*lat1 - a2*sin(2*lat1) + a3*sin(4*lat1) - a4*sin(6*lat1));

    // 辅助量
    double T = tan(lat1)*tan(lat1);
    double C = ep2*cos(lat1)*cos(lat1);
    double A = (lon1 - lon_origin)*cos(lat1);

    *X = 500000.0 + k0 * N * (A + (1 - T + C)*(A*A*A)/6 + (5 - 18*T + T*T + 72*C - 58*ep2)*(A*A*A*A*A)/120);
    *Y = M + N * tan(lat1) * ((A*A)/2 +
                              (5 - T + 9*C + 4*C*C)*(A*A*A*A)/24 +
                              (61 - 58*T + T*T + 600*C - 330*ep2)*(A*A*A*A*A*A)/720);


    *Y *= k0;
    return;
}
//==========================================================





//高斯投影由大地坐标(Unit:Metres)反算经纬度(Unit:DD)
void GaussProjInvCal(double X, double Y, double *longitude, double *latitude)
{
    int ProjNo; int ZoneWide; ////带宽
    double longitude1, latitude1, longitude0, latitude0, X0, Y0, xval, yval;
    double e1, e2, f, a, ee, NN, T, C, M, D, R, u, fai, iPI;
    iPI = 0.0174532925199433; ////3.1415926535898/180.0;
    a = 6378245.0; f = 1.0 / 298.3; //54年北京坐标系参数
    ////a=6378140.0; f=1/298.257; //80年西安坐标系参数
    ZoneWide = 6; ////6度带宽
    ProjNo = (int)(X / 1000000L); //查找带号
    longitude0 = (ProjNo - 1) * ZoneWide + ZoneWide / 2;
    longitude0 = longitude0 * iPI; //中央经线
    X0 = ProjNo * 1000000L + 500000L;
    Y0 = 0;
    xval = X - X0; yval = Y - Y0; //带内大地坐标
    e2 = 2 * f - f * f;
    e1 = (1.0 - sqrt(1 - e2)) / (1.0 + sqrt(1 - e2));
    ee = e2 / (1 - e2);
    M = yval;
    u = M / (a*(1 - e2 / 4 - 3 * e2*e2 / 64 - 5 * e2*e2*e2 / 256));
    fai = u + (3 * e1 / 2 - 27 * e1*e1*e1 / 32)*sin(2 * u) + (21 * e1*e1 / 16 - 55 * e1*e1*e1*e1 / 32)*sin(
                4 * u)
            + (151 * e1*e1*e1 / 96)*sin(6 * u) + (1097 * e1*e1*e1*e1 / 512)*sin(8 * u);
    C = ee * cos(fai)*cos(fai);
    T = tan(fai)*tan(fai);
    NN = a / sqrt(1.0 - e2 * sin(fai)*sin(fai));
    R = a * (1 - e2) / sqrt((1 - e2 * sin(fai)*sin(fai))*(1 - e2 * sin(fai)*sin(fai))*(1 - e2 * sin
                                                                                       (fai)*sin(fai)));
    D = xval / NN;
    //计算经度(Longitude) 纬度(Latitude)
    longitude1 = longitude0 + (D - (1 + 2 * T + C)*D*D*D / 6 + (5 - 2 * C + 28 * T - 3 * C*C + 8 * ee + 24 * T*T)*D
                               *D*D*D*D / 120) / cos(fai);
    latitude1 = fai - (NN*tan(fai) / R)*(D*D / 2 - (5 + 3 * T + 10 * C - 4 * C*C - 9 * ee)*D*D*D*D / 24
                                         + (61 + 90 * T + 298 * C + 45 * T*T - 256 * ee - 3 * C*C)*D*D*D*D*D*D / 720);
    //转换为度 DD
    *longitude = longitude1 / iPI;
    *latitude = latitude1 / iPI;
}
