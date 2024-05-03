#ifndef POSE_H
#define POSE_H
#include <Quaternion.hpp>

/* Item pose
	NED: x->North, y->East, z->Down

	For Euler Orientation:
	YAW 0˚ = Copter Nose, then rotate CLOCKWISE from 0 to 360˚
	That means 90˚ is Right, 180˚ is Back, 270˚ is Left

	For Quaternion, each is:
	double yaw0 = 0.0 * M_PI / 180.0;
    double yaw90 = 90.0 * M_PI / 180.0;
    double yaw180 = 180.0 * M_PI / 180.0;
    double yaw270 = 270.0 * M_PI / 180.0;
	Quaternion q0(cos(yaw0 / 2), 0.0, 0.0, sin(yaw0 / 2));
    Quaternion q90(cos(yaw90 / 2), 0.0, 0.0, sin(yaw90 / 2));
    Quaternion q180(cos(yaw180 / 2), 0.0, 0.0, sin(yaw180 / 2));
    Quaternion q270(cos(yaw270 / 2), 0.0, 0.0, sin(yaw270 / 2));
*/
struct Pose {
	double x;
	double y;
	double z;
	double roll;
	double yaw;
	double pitch;
	Quaternion q;

	Pose(double X = 0, double Y = 0, double Z = 0, double ROLL = 0, double YAW = 0, double PITCH = 0);
	Pose(double X, double Y, double Z, double i, double j, double k, double w);
	Pose(const Pose &p);
	void PrintPose();
};

#endif