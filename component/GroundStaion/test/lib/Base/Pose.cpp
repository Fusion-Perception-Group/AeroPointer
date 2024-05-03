#include <Pose.hpp>

Pose::Pose(double X, double Y, double Z, double ROLL, double YAW, double PITCH)
	: x(X), y(Y), z(Z), roll(ROLL), yaw(YAW), pitch(PITCH) {}

Pose::Pose(double X, double Y, double Z, double i, double j, double k, double w) {
	x = X;
	y = Y;
	z = Z;
	q = Quaternion(i, j, k, w);
}

Pose::Pose(const Pose &p) {
	x = p.x;
	y = p.y;
	z = p.z;
	roll = p.roll;
	yaw = p.yaw;
	pitch = p.pitch;
	q = p.q;
}

void Pose::PrintPose() {
	std::cout << "X: " << x << std::endl;
	std::cout << "Y: " << y << std::endl;
	std::cout << "Z: " << z << std::endl;
	std::cout << "ROLL: " << roll << std::endl;
	std::cout << "YAW: " << yaw << std::endl;
	std::cout << "PITCH: " << pitch << std::endl;
}