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
