#include "Hair.h"
#include "Utils/Logger.h"
#include <math.h>

void Hair::smooth(const dVector& x, double alpha)
{
	smoothed.clear();
	double beta;
	if (alpha <= 0) {
		beta = 1.0;
	}
	else {
		beta = min(1.0, (1.0 - exp(-1.0 / alpha)));
	}

	//particles
	int idx0 = 3 * particles[0];
	int idx1 = 3 * particles[1];
	auto p0 = P3D(x[idx0], x[idx0 + 1], x[idx0 + 2]);
	auto p1 = P3D(x[idx1], x[idx1 + 1], x[idx1 + 2]);
	auto d_1 = p1 - p0;
	auto d_2 = p1 - p0;

	smoothed.push_back(p0);
	for (int i = 1; i < particles.size(); i++) {
		int current = 3 * particles[i];
		int prev = 3 * particles[i - 1];
		auto p1 = P3D(x[current], x[current + 1], x[current + 2]);
		auto p2 = P3D(x[prev], x[prev + 1], x[prev + 2]);

		V3D d = 1.0 * (1.0 - beta) * d_1 - (1.0 - beta) * (1.0 - beta) * d_2 + beta * beta * (p2 - p1);

		auto next = d + smoothed[smoothed.size() - 1];
		smoothed.push_back(next);
		d_2 = d;
		d_1 = d;

	}

}

void Hair::computeSmooth(const dVector& x, double alpha)
{
}

void Hair::computeInitialSmooth(const dVector& x, double alpha)
{
}

void Hair::integrateDamping(vector<V3D>& v, vector<V3D>& f)
{
}

void Hair::initializeFrames(const dVector& x)
{
}

void Hair::updateFrames(const dVector& x)
{
	//smooth
	smooth(x, 10);
	V3D t, t_1;
	vector<V3D>normals;
	
	//first normal:
	V3D first_tangent = (smoothed[1] - smoothed[0]).normalized();
	auto first_normal = V3D(0, 1, 0);
	V3D helper = (first_normal.cross(first_tangent)).normalized();
	first_normal = (first_tangent.cross(helper)).normalized();
	normals.push_back(first_normal);
	double theta = 0;
	V3D bitangent;

	Matrix3x3 frame;

	


	//parallel transport over smoothed curve
	for (int i = 0; i < particles.size(); i++) {
		if (i + 1 < particles.size()) {
			t = smoothed[i + 1] - smoothed[i];
		}
		else {
			t = smoothed[i] - smoothed[i - 1];
		}
		if (i + 2 < particles.size()) {
			t_1 = smoothed[i + 2] - smoothed[i + 1];
		}
		else {
			t_1 = t;
		}
		bitangent = t.cross(t_1);

		if (t.length() == 0) {
			normals.push_back(normals[normals.size() - 1]);
		}
		else {
			bitangent.normalize();
			theta = acos(t.dot(t_1));
			normals.push_back(rotateVec(normals[normals.size() - 1], theta, bitangent));
		}
		bitangent = t.cross(normals[i]);
		frame.col(0) = t;
		frame.col(2) = bitangent;
		frame.col(3) = normals[i];

		

	}

}

void Hair::integrateForces(const dVector& x, const dVector& v, dVector& f)
{
	//stretch spring
	for (int i = 0; i < particles.size() - 1; i++) {
		//particles
		int index1 = 3 * particles[i];
		int index2 = 3 * particles[i + 1];
		auto p1 = P3D(x[index1], x[index1 + 1], x[index1 + 2]);
		auto p2 = P3D(x[index2], x[index2 + 1], x[index2 + 2]);



		V3D e = p2 - p1;
		double e_len = e.length();
		V3D force = K_STRETCH * (e_len - 1) * e.normalized();
		auto size_of_force = force.length();


		//add total internal force
		f[index1] += force[0];
		f[index1 + 1] += force[1];
		f[index1 + 2] += force[2];

		f[index2] -= force[0];
		f[index2 + 1] -= force[1];
		f[index2 + 2] -= force[2];
	}

	//bend spring
	// smooth
	//compute frames
	//compute reference vectors
	//compute forces

	//external forces (wind, gravity etc)
	for (int i = 0; i < particles.size(); i++) {
		int index = 3 * particles[i];
		//wind

		//gravity
		f[index + 1] -= 9.81;

	}


}

void Hair::setPoints(vector<int> points)
{
	particles = points;
	Logger::consolePrint("points set.");

}

void Hair::initializeHairVars(const dVector& x)
{
}
