#include "Aeropointer.hpp"

Copter::Copter(double X, double Y, double Z, double ROLL, double YAW,
			   double PITCH)
	: pose(X, Y, Z, ROLL, YAW, PITCH) {}

Copter::~Copter() {}

void setPoint(const Pose destPose){
	
}

void Copter::printPose() {
	std::cout << "X: " << pose.x << std::endl;
	std::cout << "Y: " << pose.y << std::endl;
	std::cout << "Z: " << pose.z << std::endl;
	std::cout << "ROLL: " << pose.roll << std::endl;
	std::cout << "YAW: " << pose.yaw << std::endl;
	std::cout << "PITCH: " << pose.pitch << std::endl;
}