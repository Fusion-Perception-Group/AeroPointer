#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Pose.hpp>

struct trajectory {
	std::deque<Pose> poses;
	size_t maxSize; // MaxSize of Buffer

	// add Pose record to buffer
	void addData(const Pose &pose);

	// add Pose record to buffer with '+='
	trajectory &operator+=(const Pose &pose);
};
#endif // !TRAJECTORY_H