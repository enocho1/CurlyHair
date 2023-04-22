#pragma once
#include "MathLib/P3D.h"
#include "MathLib/V3D.h"
#include "MathLib/Matrix.h"
#include "MS_SparseSquare.h"
#include <vector>


/*
* i made this class to make it easier to think about hairs on an individual level. the aim is to have this just reference the particle system
* so the hair should know the indices of its rout and the particles on the strand.
* internal forces should be calculated through the hairs instead of through the spring classes directly
* smoothed versions of the hairs and hair velocities etc should be created here.
* frames (via parallel transport) should be initialized and updated here too.
*/
class Hair {
private:
	vector<int> particles;// the 0th entry should be the route and it's assumed that a spring exists between each connected particle.
	vector<V3D> rest_x;
	vector<V3D> rest_t;
	vector<V3D> t_vecs;
	vector<Matrix3x3> rest_frames;
	vector<Matrix3x3> frames;

	//returns the smoothed version of whatever attribute you pass it (usually pos, but sometimes vels)
	vector<V3D> smooth(const dVector& x);
	void integrateDamping(vector<V3D>& v, vector<V3D>& f);

public:
	void integrateInternalForces(double dtf, double dtd, const dVector& x, const dVector& v, dVector& f);
};