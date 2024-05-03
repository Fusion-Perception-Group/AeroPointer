#include <Trajectory.hpp>

void trajectory::addPose(const Pose &pose) {
	if (poses.size() == maxSize) {
		poses.pop_front();
	}
	poses.push_back(pose);
}

// async trajectory &trajectory::operator+=(const Pose &pose) {
// 	if (poses.size() == maxSize) {
// 		poses.pop_front();
// 	}
// 	poses.push_back(pose);

// 	return *this;
// }

async void trajectory::saveToFile(const std::string &filename){
	std::ofstream file;
	file.open(filename);
	for (auto &pose : poses) {
		file << pose.x << " " << pose.y << " " << pose.z << " " << pose.roll << " " << pose.yaw << " " << pose.pitch << std::endl;
	}
}
