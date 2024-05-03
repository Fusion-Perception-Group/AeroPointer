#include <Quaternion.hpp>

Quaternion::Quaternion(double I, double J, double K, double W)
	: i(I), j(J), k(K), w(W) {}

Quaternion Quaternion::operator*(const Quaternion &q) {
	return Quaternion(w * q.i + i * q.w + j * q.k - k * q.j,
					  w * q.j - i * q.k + j * q.w + k * q.i,
					  w * q.k + i * q.j - j * q.i + k * q.w,
					  w * q.w - i * q.i - j * q.j - k * q.k);
}

Quaternion Quaternion::operator+(const Quaternion &q) {
	return Quaternion(i + q.i, j + q.j, k + q.k, w + q.w);
}

Quaternion Quaternion::operator-(const Quaternion &q) {
	return Quaternion(i - q.i, j - q.j, k - q.k, w - q.w);
}

Quaternion Quaternion::operator*(double scalar) {
	return Quaternion(i * scalar, j * scalar, k * scalar, w * scalar);
}

Quaternion Quaternion::operator/(double scalar) {
	return Quaternion(i / scalar, j / scalar, k / scalar, w / scalar);
}

Quaternion Quaternion::conjugate() { return Quaternion(-i, -j, -k, w); }

double Quaternion::norm() { return sqrt(i * i + j * j + k * k + w * w); }

Quaternion Quaternion::normalize() { return *this / norm(); }

void Quaternion::PrintQuaternion() {
	std::cout << "i: " << i << std::endl;
	std::cout << "j: " << j << std::endl;
	std::cout << "k: " << k << std::endl;
	std::cout << "w: " << w << std::endl;
}

// Path: lib/Aeropointer/Quaternion.cpp