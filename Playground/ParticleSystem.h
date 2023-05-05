#pragma once

#include "Spring.h"
#include "taz_StretchSpring.h"
#include "GUILib/OBJReader.h"
#include "ZeroLengthSpring.h"
#include "MS_SparseSquare.h"
#include <vector>
#include <list>
#include "GUILib/GLMesh.h"
#include <string>
#include <filesystem>
#include "Hair.h"

using namespace std;

struct Particle {
	P3D position;
	P3D velocity;
	double mass;
};

using namespace Eigen;

class ParticleSystem {
private:
	// Vectors containing particle states for the system.
	dVector positions;
	dVector meshPositions;
	dVector velocities;
	dVector masses;
	P3D center;
	V3D head_speed;
	int numParticles;
	float radius;
	string capturedData;
	int frameCount;
	vector<Hair> hairs;
	

	// A list of all generic springs in the system.
	vector<Spring> springs;
	
	// A list of all generic springs in the system.
	vector<Stretch> stretch_springs;

	// A list of all zero length springs in the system.
	list<ZeroLengthSpring> zeroLengthSprings;

	// A list of flags for each particle specifying if it's currently pinned.
	vector<bool> particlePinned;

	// Vectors to pass to OpenGL for drawing.
	// Each time step, the relevant data are copied into these lists.
	vector<double> positionArray;
	vector<double> meshPositionArray;
	vector<unsigned int> pointsIndexArray;
	vector<unsigned int> meshPointsIndexArray;
	vector<unsigned int> edgesIndexArray;
	vector<double> zlSpringPositionArray;
	
	
	//for frame visualisation
	vector<unsigned int> frameIndexArray;
	vector<double> framePositionArray;


	int count;

	GLMesh* mesh;
	
public:
	ParticleSystem(vector<Particle>& particles);
	void recordPositions();
	void addSpring(int p1, int p2, double stiffness);
	void addStretch(int p1, int p2, double stiffness, double damping);
	void addZeroLengthSpring(int p, P3D x0, double stiffness);
	void deleteZeroLengthSpring(int p);
	void togglePin(int p);
	P3D getPositionOf(int i);
	int particleCount();
	dVector computeForceVector(const dVector &x, const dVector &v);
	MS_SparseSquare computeForceJacobian(const dVector &x, const dVector &v);
	dVector computeAccelerations(const dVector &x, const dVector &v, const dVector &m);
	void integrate_FE(double delta);
	void integrate_SE(double delta);
	dVector newtonStep(double delta, dVector &positions, dVector &velocities, dVector &masses, dVector &v_guess);
	void integrate_BE(double delta);
	void integrate_Pxr(double outer);
	void integrate_Pxr_Alt(double outer);
	void setHairs(vector<Hair> h);
	string inputName;

	void manageCollisions();

	void addMesh(string filename);

	V3D reflectVector(V3D inputVector, V3D normal);

	// Functions for display and interactivity
	void drawParticleSystem();
	void setPosition(int i, P3D x);
	P3D getCenter();
	void setCenter(P3D x);
	void setHeadSpeed(V3D v);
	V3D getHeadSpeed();
	void setVelocity(int i, V3D v);
	V3D getVelocity(int i);
	void setMesh(GLMesh* m);

	void moveHead(double delta);

	void saveData();
	bool ready_to_quit;

	// Whether or not we should draw springs and particles as lines and dots respectively.
	static bool drawSprings;
	static bool drawStretchSprings;
	static bool drawParticles;
	static bool drawZeroLengthSprings;
	static bool drawFaces;
	static bool enableGravity;
	static bool enableCollision;


};

