#ifndef AEROPOINTER_H
#define AEROPOINTER_H

#include <Trajectory.hpp>

class Copter {
private:
	Pose pose;
	trajectory traj;

public:
	Copter(double X = 0, double Y = 0, double Z = 0, double ROLL = 0, double YAW = 0, double PITCH = 0);

	~Copter();

	void setPoint(const Pose destPose);

	void printPose();
};

#endif