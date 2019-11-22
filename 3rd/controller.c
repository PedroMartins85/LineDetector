
#include "controller.h"

double controller(double Th,double X,double Y,double et,double ex,double lamda,double pho,double ty,double tz,double v)
{
	double L=0.30; // in meters

	double  t2 = cos(pho);
	double  t3 = cos(Th);
	double  t4 = 1.0/ty;
	double  t5 = sin(Th);
	double  t6 = sin(pho);
	double  t15 = Y*t2;
	double  t16 = t6+t15;
	double  t17 = X*X;
	double  t18 = t17+1.0;
	double  t19 = t2*t18;
	double  t20 = Y*t6;
	double  t21 = t4*t16*tz;
	double  t7 = t19-t20+t21;
	double  t8 = X*t3;
	double  t9 = Y*t5;
	double  t10 = t8+t9;
	double  t12 = t2*t5*t10;
	double  t13 = t3*t3;
	double  t14 = t2*t4*t13*tz;
	double  t11 = t6+t12-t14;
	double  t22 = t7*t7;
	double  t23 = t11*t11;
	double  t24 = t22+t23;
	double  t25 = 1.0/t24;
	double  t0 = atan(-(L*(t11*t25*(et*lamda-v*((t2*t2)*t3*t4*t10+t2*t3*t4*t5*t6))+t7*t25*(ex*lamda+X*t2*t4*t16*v)))/v);
	
	return t0;
}
