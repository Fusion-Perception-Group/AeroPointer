#include <Trajectory.hpp>

void trajectory::addData(const Pose &pose) {
	if (poses.size() == maxSize) {
		poses.pop_front();
	}
	poses.push_back(pose);
}

trajectory &trajectory::operator+=(const Pose &pose) {
	addData(pose);
	return *this;
}
