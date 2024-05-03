#ifndef QUATERNION_H
#define QUATERNION_H

#include <Base.hpp>

struct Quaternion
{
	double i;
	double j;
	double k;
	double w;
	Quaternion(double I = 0, double J = 0, double K = 0, double W = 1);
	Quaternion operator*(const Quaternion &q);
	Quaternion operator+(const Quaternion &q);
	Quaternion operator-(const Quaternion &q);
	Quaternion operator/(const Quaternion &q);
	Quaternion operator*(double scalar);
	Quaternion operator/(double scalar);
	Quaternion conjugate();
	double norm();
	Quaternion normalize();
	void PrintQuaternion();
};

#endif // !QUATERNION_H