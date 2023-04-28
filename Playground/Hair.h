#pragma once
#include "MathLib/P3D.h"
#include "MathLib/V3D.h"
#include "MathLib/Matrix.h"
#include "MS_SparseSquare.h"
#include <vector>


#define K_STRETCH 5e06

#define DT_FORCE 4.62944e-05
#define DT_DAMPING 4.62944e-06 


/*
* i made this class to make it easier to think about hairs on an individual level. the aim is to have this just reference the particle system
* so the hair should know the indices of its rout and the particles on the strand.
* internal forces should be calculated through the hairs instead of through the spring classes directly
* smoothed versions of the hairs and hair velocities etc should be created here.
* frames (via parallel transport) should be initialized and updated here too.
*/
class Hair {
private:
	vector<V3D> rest_x;
	
	vector<V3D> rest_smooth;
	vector<V3D> smoothed;

	vector<Matrix3x3> rest_frames;

	vector<V3D> rest_t;
	vector<V3D> t_vecs;

	

	//returns the smoothed version of whatever attribute you pass it (usually pos, but sometimes vels)
	void smooth(const dVector& x, double alpha);
	void computeSmooth(const dVector& x, double alpha);
	void computeInitialSmooth(const dVector& x, double alpha);
	void integrateDamping(vector<V3D>& v, vector<V3D>& f);
	void initializeFrames(const dVector& x);
	void updateFrames(const dVector& x);

public:
	vector<int> particles;// the 0th entry should be the route and it's assumed that a spring exists between each connected particle.

	vector<Matrix3x3> frames;

	void integrateForces(const dVector& x, const dVector& v, dVector& f);
	void setPoints(vector<int> points);
	void initializeHairVars(const dVector &x);
	void vis(const dVector& x);
};