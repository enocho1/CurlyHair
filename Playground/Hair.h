#pragma once
#include "MathLib/P3D.h"
#include "MathLib/V3D.h"
#include "MathLib/Matrix.h"
#include "MS_SparseSquare.h"
#include <vector>


#define K_STRETCH 4.8e05
#define K_BEND 6.0e01//5.0e02 //6.0e01 //10.0e02//3.2e02//
#define K_CORE 1000//0

#define C_STRETCH 45//00
#define C_BEND 50
#define C_CORE 100

#define DAMPING 9.2e-01//1.1//2.14e-01 //3.14e-01 //
#define CHARGE 7.14e-02 //8.14e-01 //


#define ALPHA_BEND 3.0
#define ALPHA_CORE 3.0

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

	vector<V3D> v_smoothed;

	vector<Matrix3x3> rest_frames;
	vector<Matrix3x3> frames;

	vector<V3D> rest_t;
	vector<V3D> t_vecs;
	vector<V3D> rest_d;
	vector<V3D> d_vecs;
	
	vector<V3D> smooth_vels;



	//returns the smoothed version of whatever attribute you pass it (usually pos, but sometimes vels)
	void smooth(const dVector& x, double alpha);
	void computeSmoothVelocities(const dVector& v, double alpha);
	void initializeFrames(const dVector& x);
	void updateFrames(const dVector& x);
	void updateDVecs(const dVector& x, double alpha, vector<V3D>& d_list);

public:
	vector<int> particles;// the 0th entry should be the root and it's assumed that a spring exists between each connected particle.

	const vector<Matrix3x3>& const getFrames() { return frames; };
	const vector<V3D>& const getTVecs() { return t_vecs; };
	const vector<V3D>& const getDVecs() { return d_vecs; };

	void integrateForces(const dVector& x, const dVector& v, dVector& f);
	void integrateDampingForces(const dVector& x, const dVector& v, dVector& f);
	void setPoints(vector<int> points);
	void initializeHairVars(const dVector& x);
	void vis(const dVector& x);
	void updateHead(dVector& x);
	void repulsion(const dVector& x, const dVector& v, dVector& f, int particle_count);

};