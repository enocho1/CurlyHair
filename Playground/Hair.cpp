#include "Hair.h"
#include "Utils/Logger.h"
#include <math.h>

vector<V3D> Hair::smooth(const dVector& x)
{
	return vector<V3D>();
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
	for (int i = 0; i < particles.size() - 1; i++) {
		//particles
		int index1 = 3 * particles[i];
		int index2 = 3 * particles[i + 1];
		auto p1 = P3D(x[index1], x[index1 + 1], x[index1 + 2]);
		auto p2 = P3D(x[index2], x[index2 + 1], x[index2 + 2]);


		//stretch spring
		V3D e = p2 - p1;
		double e_len = e.length();
		V3D force = K_STRETCH * (e_len - 1) * e.normalized();
		auto size_of_force = force.length();


		Logger::consolePrint("forces:  %d N", size_of_force);

		//bend spring

		//core spring

		//external (wind, gravity etc)


		//add total internal force
		f[index1] += force[0];
		f[index1 + 1] += force[1];
		f[index1 + 2] += force[2];

		f[index2] -= force[0];
		f[index2 + 1] -= force[1];
		f[index2 + 2] -= force[2];
	}

	//external forces
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
