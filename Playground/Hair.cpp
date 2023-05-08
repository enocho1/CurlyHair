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

		V3D d = 2.0 * (1.0 - beta) * d_1 - (1.0 - beta) * (1.0 - beta) * d_2 + beta * beta * (p1 - p2);

		auto next = d + smoothed[smoothed.size() - 1];
		smoothed.push_back(next);
		d_2 = d_1;
		d_1 = d;

	}

}

void Hair::computeSmoothVelocities(const dVector& v, double alpha)
{
	smooth_vels.clear();

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
	auto p0 = P3D(v[idx0], v[idx0 + 1], v[idx0 + 2]);
	auto p1 = P3D(v[idx1], v[idx1 + 1], v[idx1 + 2]);
	auto d_1 = p1 - p0;
	auto d_2 = p1 - p0;

	smoothed.push_back(p0);
	for (int i = 1; i < particles.size(); i++) {
		int current = 3 * particles[i];
		int prev = 3 * particles[i - 1];
		auto p1 = P3D(v[current], v[current + 1], v[current + 2]);
		auto p2 = P3D(v[prev], v[prev + 1], v[prev + 2]);

		V3D d = 2.0 * (1.0 - beta) * d_1 - (1.0 - beta) * (1.0 - beta) * d_2 + beta * beta * (p1 - p2);

		auto next = d + smoothed[smoothed.size() - 1];
		smoothed.push_back(next);
		d_2 = d_1;
		d_1 = d;

	}

}


void Hair::integrateDampingForces(const dVector& x, const dVector& v, dVector& f)
{
	updateDVecs(v, ALPHA_CORE, smooth_vels);
	updateDVecs(x, ALPHA_CORE, d_vecs);

	int root = 3 * particles[0];


	//spring forces
	for (int i = 0; i < particles.size() - 1; i++) {
		int index1 = 3 * particles[i];
		int index2 = 3 * particles[i + 1];
		auto p1 = P3D(x[index1], x[index1 + 1], x[index1 + 2]);
		auto p2 = P3D(x[index2], x[index2 + 1], x[index2 + 2]);
		auto v1 = V3D(v[index1], v[index1 + 1], v[index1 + 2]);
		auto v2 = V3D(v[index2], v[index2 + 1], v[index2 + 2]);

		V3D dv = v2 - v1;
		V3D e = p2 - p1;
		V3D t = t_vecs[i];


		V3D stretch = C_STRETCH * dv.dot(e.normalized()) * e.normalized();
		V3D bend = C_BEND * (dv - V3D(dv.dot(e.normalized()) * e.normalized()));
		V3D core = C_CORE * smooth_vels[i].dot(d_vecs[i].normalized()) * d_vecs[i].normalized();


		for (int k = 0; k < 3; k++) {
			f[index1 + k] += stretch[k];
			f[index2 + k] -= stretch[k];

			f[index1 + k] += bend[k];
			f[index2 + k] -= bend[k];

			f[index1 + k] += core[k];
			f[index2 + k] -= core[k];
		}
	}
	//root constraint
	for (int k = 0; k < 3; k++) {
		f[root + k] = 0;
	}
}

void Hair::initializeFrames(const dVector& x)

{
	//smooth
	smooth(x, ALPHA_BEND);
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

	rest_frames.clear();




	//parallel transport over smoothed curve
	for (int i = 0; i < particles.size(); i++) {
		if (i + 1 < particles.size()) {
			t = (smoothed[i + 1] - smoothed[i]).normalized();
		}
		else {
			t = (smoothed[i] - smoothed[i - 1]).normalized();
		}
		if (i + 2 < particles.size()) {
			t_1 = (smoothed[i + 2] - smoothed[i + 1]).normalized();
		}
		else {
			t_1 = t;
		}
		//t = first_tangent; // @enoch look
		bitangent = t.cross(t_1);

		if (bitangent.length() == 0) {
			normals.push_back(normals[normals.size() - 1]);
		}
		else {
			bitangent.normalize();
			double ddot = t.dot(t_1);
			ddot = min(1.0, max(-1.0, ddot));
			theta = acos(ddot);
			normals.push_back(rotateVec(normals[normals.size() - 1], theta, bitangent));
		}
		bitangent = t.cross(normals[i]);
		frame.col(0) = t;
		frame.col(1) = bitangent;
		frame.col(2) = normals[i];

		rest_frames.push_back(frame);


	}

	//compute t-vectors
	rest_t.clear();
	for (int i = 0; i < particles.size() - 1; i++)
	{
		//particles
		int index1 = 3 * particles[i];
		int index2 = 3 * particles[i + 1];
		auto p1 = P3D(x[index1], x[index1 + 1], x[index1 + 2]);
		auto p2 = P3D(x[index2], x[index2 + 1], x[index2 + 2]);
		V3D e = p2 - p1;

		V3D t = rest_frames[i].transpose() * e;

		rest_t.push_back(t);
	}
}

void Hair::updateFrames(const dVector& x)
{
	//smooth
	smooth(x, ALPHA_BEND);
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

	frames.clear();

	//parallel transport over smoothed curve
	for (int i = 0; i < particles.size(); i++) {

		if (i + 1 < particles.size()) {
			t = (smoothed[i + 1] - smoothed[i]).normalized();
		}
		else {
			t = (smoothed[i] - smoothed[i - 1]).normalized();
		}
		if (i + 2 < particles.size()) {
			t_1 = (smoothed[i + 2] - smoothed[i + 1]).normalized();
		}
		else {
			t_1 = t;
		}
		//t = first_tangent; // @enoch look
		bitangent = t.cross(t_1);
		bitangent.normalize();


		if (bitangent.length2() != 1.0) {
			normals.push_back(normals[normals.size() - 1]);
		}
		else {
			double ddot = t.dot(t_1);
			ddot = min(1.0, max(-1.0, ddot));
			theta = acos(ddot);
			normals.push_back(rotateVec(normals[normals.size() - 1], theta, bitangent));
		}
		bitangent = t.cross(normals[i]);
		frame.col(0) = t;
		frame.col(1) = bitangent;
		frame.col(2) = normals[i];

		frames.push_back(frame);

	}

	//compute t-vectors
	t_vecs.clear();
	for (int i = 0; i < particles.size() - 1; i++)
	{
		V3D t = frames[i] * rest_t[i];

		t_vecs.push_back(t);
	}

}

void Hair::updateDVecs(const dVector& x, double alpha, vector<V3D>& d_list)
{
	d_list.clear();
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
	d_list.push_back(d_1);
	for (int i = 1; i < particles.size(); i++) {
		int current = 3 * particles[i];
		int prev = 3 * particles[i - 1];
		auto p1 = P3D(x[current], x[current + 1], x[current + 2]);
		auto p2 = P3D(x[prev], x[prev + 1], x[prev + 2]);

		V3D d = 2.0 * (1.0 - beta) * d_1 - (1.0 - beta) * (1.0 - beta) * d_2 + beta * beta * (p1 - p2);
		d_list.push_back(d);
		d_2 = d_1;
		d_1 = d;

	}
}

void Hair::updateHead(dVector& x)
{
	//pin the root back to the head
	int index = 3 * particles[0];
	for (int k = 0; k < 3; k++) {
		x[index + k] = rest_x[0][k];
	}

}

void Hair::repulsion(const dVector& x, const dVector& v, dVector& f, int particle_count)
{
	int start = particles[0];
	int end = particles[particles.size() - 1];

	for (int i = 1; i < particles.size() - 1; i++) {
		int index1 = 3 * particles[i];
		auto p1 = P3D(x[index1], x[index1 + 1], x[index1 + 2]);

		//loop through all other hairs
		for (int j = 0; j < particle_count; j++) {

			//don't repel hairs on the same strand
			if (j<start || j>end) {
				int index2 = 3 * j;
				auto p2 = P3D(x[index2], x[index2 + 1], x[index2 + 2]);
				auto e = p1 - p2;
				double r2 = e.length2();
				V3D repulse;
				if (r2 > 0) {
					V3D repulse = CHARGE * e / r2;
				}
				else {
					V3D repulse = 0.0 * e;;
				}
				for (int k = 0; k < 3; k++) {
					f[index1 + k] += repulse[k];
				}

			}
		}
		int index2 = 3 * particles[i + 1];
		V3D v2 = P3D(v[index2], v[index2 + 1], v[index2 + 2]);
	}

}

void Hair::integrateForces(const dVector& x, const dVector& v, dVector& f)
{

	updateFrames(x);
	updateDVecs(x, ALPHA_CORE, d_vecs);

	int root = 3 * particles[0];


	//spring forces
	for (int i = 0; i < particles.size() - 1; i++) {
		int index1 = 3 * particles[i];
		int index2 = 3 * particles[i + 1];
		auto p1 = P3D(x[index1], x[index1 + 1], x[index1 + 2]);
		auto p2 = P3D(x[index2], x[index2 + 1], x[index2 + 2]);
		V3D v2 = P3D(v[index2], v[index2 + 1], v[index2 + 2]);


		V3D e = p2 - p1;
		V3D t = t_vecs[i];


		V3D stretch = K_STRETCH * (e.length() - 1) * e.normalized();
		V3D bend = K_BEND * (e - t);
		V3D core = K_CORE * max(0.0, (d_vecs[i].length() - rest_d[i].length())) * d_vecs[i].normalized();

		
		V3D damping = -DAMPING * v2;

		V3D wind = CHARGE * V3D(((double)rand()) / RAND_MAX, ((double)rand()) / RAND_MAX, ((double)rand()) / RAND_MAX).normalized();


		for (int k = 0; k < 3; k++) {
			f[index1 + k] += stretch[k];
			f[index2 + k] -= stretch[k];

			f[index1 + k] += bend[k];
			f[index2 + k] -= bend[k];

			f[index1 + k] += core[k];
			f[index2 + k] -= core[k];

			//damping
			//f[index2 + k] += damping[k];
			//wind/noise
			f[index2 + k] += wind[k];

		}
	}

	//external forces (wind, gravity etc)
	for (int i = 0; i < particles.size(); i++) {
		int index = 3 * particles[i];

		//gravity
		f[index + 1] -= 9.81;

	}
	//root constraint
	for (int k = 0; k < 3; k++) {
		f[root + k] = 0;
	}
}

void Hair::setPoints(vector<int> points)
{
	particles = points;
	Logger::consolePrint("points set.");

}

void Hair::initializeHairVars(const dVector& x)
{
	initializeFrames(x);
	updateDVecs(x, ALPHA_CORE, rest_d);

	//include rest points
	rest_x.clear();
	for (int i = 0; i < particles.size(); i++) {
		int index = 3 * particles[i];
		auto p1 = P3D(x[index], x[index + 1], x[index + 2]);
		rest_x.push_back(p1);
	}
}

void Hair::vis(const dVector& x)
{
	updateFrames(x);
}
