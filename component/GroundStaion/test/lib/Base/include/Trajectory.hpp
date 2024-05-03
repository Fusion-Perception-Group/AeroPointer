#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Pose.hpp>

struct trajectory {
	std::deque<Pose> poses;
	size_t maxSize; // MaxSize of Buffer

	void addPose(const Pose &pose);
	trajectory &operator+=(const Pose &pose);
	void saveToFile(const std::string &filename);
};
#endif // !TRAJECTORY_H