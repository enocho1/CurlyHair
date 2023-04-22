#ifdef _WIN32
#include <include/glew.h>
#else
#include <GL/glew.h>
#endif

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

#include "ParticleSystem.h"
#include "Utils/Logger.h"
#include "Constants.h"
#include <math.h>
#include <iostream>
#include <fstream>

ParticleSystem::ParticleSystem(vector<Particle>& particles)
{
	center = P3D(0, 0, 0);
	head_speed = V3D(0,0,0);
	radius = 1.0f;
	drawSprings = true;
	drawParticles = true;
	drawZeroLengthSprings = true;
	count = 0;
	numParticles = particles.size();
	mesh = 0;
	capturedData = "";
	frameCount = 0;
	ready_to_quit = false;


	// Create vectors of this size to hold the particle states
	positions = dVector(numParticles * 3);
	velocities = dVector(numParticles * 3);
	masses = dVector(numParticles);

	particlePinned = vector<bool>(numParticles);

	for (int i = 0; i < numParticles; i++) {
		positions[3 * i] = particles[i].position.at(0);
		positions[3 * i + 1] = particles[i].position.at(1);
		positions[3 * i + 2] = particles[i].position.at(2);

		velocities[3 * i] = particles[i].velocity.at(0);
		velocities[3 * i + 1] = particles[i].velocity.at(1);
		velocities[3 * i + 2] = particles[i].velocity.at(2);

		masses[i] = particles[i].mass;

		particlePinned[i] = false;
	}
	recordPositions();

	// Arrays to be passed to OpenGL
	positionArray = vector<double>(numParticles * 3);
	pointsIndexArray = vector<unsigned int>(numParticles);

	for (int i = 0; i < numParticles; i++) {
		pointsIndexArray[i] = i;
	}
}

void ParticleSystem::recordPositions() {
	if (frameCount < 200) {
		frameCount += 1;
		string new_result_lines;
		//Logger::consolePrint("check enoch");
		for (int i = 0; i < 3; i++) {
			new_result_lines += to_string(center[i]);
			new_result_lines += " ";
		}

		new_result_lines += "\n";

		for (int j = 0; j < numParticles; j++) {
			new_result_lines += to_string(positions[3 * j]);
			new_result_lines += " ";
			new_result_lines += to_string(positions[3 * j + 1]);
			new_result_lines += " ";
			new_result_lines += to_string(positions[3 * j + 2]);
			new_result_lines += ",";
		}
		new_result_lines += "\n";

		capturedData += new_result_lines;
	}
	else {
		saveData();
	}
	

}

bool ParticleSystem::drawSprings = false;
bool ParticleSystem::drawStretchSprings = true;
bool ParticleSystem::drawParticles = true;
bool ParticleSystem::drawZeroLengthSprings = true;
bool ParticleSystem::drawFaces = true;
bool ParticleSystem::enableGravity = false;
bool ParticleSystem::enableCollision = false;

int count = 0;

P3D ParticleSystem::getPositionOf(int i) {
	return P3D(positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]);
}

// Set the position of particle i.
void ParticleSystem::setPosition(int i, P3D x) {
	positions[3 * i] = x.at(0);
	positions[3 * i + 1] = x.at(1);
	positions[3 * i + 2] = x.at(2);
}
P3D ParticleSystem::getCenter() {
	return center;
}

void ParticleSystem::setCenter(P3D x)
{
	center = x;
}

void ParticleSystem::setHeadSpeed(V3D v)
{
	head_speed = v;
}

V3D ParticleSystem::getHeadSpeed()
{
	return head_speed;
}

// Set the velocity of particle i.
void ParticleSystem::setVelocity(int i, V3D v) {
	velocities[3 * i] = v.at(0);
	velocities[3 * i + 1] = v.at(1);
	velocities[3 * i + 2] = v.at(2);
}

V3D ParticleSystem::getVelocity(int i) {
	return V3D(velocities[3 * i], velocities[3 * i + 1], velocities[3 * i + 2]);
}

int ParticleSystem::particleCount() {
	return numParticles;
}

// Adds a spring to the system. The rest length of the spring will be
// computed from the current positions of the two endpoints.
void ParticleSystem::addSpring(int p1, int p2, double stiffness) {
	Spring newSpring(p1, p2, stiffness, positions);
	springs.push_back(newSpring);
}

void ParticleSystem::addStretch(int p1, int p2, double stiffness, double damping)
{
	Stretch newSpring(p1, p2, stiffness,damping, positions);
	stretch_springs.push_back(newSpring);
}

// Add a zero-length spring (a "pin") to the system, attaching a particle
// to the given location.
void ParticleSystem::addZeroLengthSpring(int p, P3D x0, double stiffness) {
	if (particlePinned[p]) return;
	ZeroLengthSpring newSpring(p, x0, stiffness, positions);
	zeroLengthSprings.push_back(newSpring);
	particlePinned[p] = true;
}

// Delete the zero-length spring attached to p.
void ParticleSystem::deleteZeroLengthSpring(int p) {
	for (list<ZeroLengthSpring>::iterator i = zeroLengthSprings.begin(); i != zeroLengthSprings.end(); ++i) {
		if (p == i->getParticle()) {
			zeroLengthSprings.erase(i);
			break;
		}
	}
	particlePinned[p] = false;
}

// Either pins or unpins particle p to/from its current position.
void ParticleSystem::togglePin(int p) {
	if (particlePinned[p]) {
		// If pinned, unpin it
		Logger::consolePrint("Unpinned %d", p);
		deleteZeroLengthSpring(p);
	}
	else {
		Logger::consolePrint("Pinned %d", p);
		// If not pinned, pin it
		P3D pinPos = getPositionOf(p);
		addZeroLengthSpring(p, pinPos, PIN_STIFFNESS);
	}
}

// Gravitational constant.
const double G = -9.8;

// Compute the vector of forces applied to particles, given the current positions
// and velocities.
dVector ParticleSystem::computeForceVector(const dVector& x, const dVector& v) {
	// The force vector has the same length as the position vector.for abou
	dVector forceVector = dVector::Zero(x.size());
	recordPositions();

	// TODO (Part 2): Accumulate the forces from all springs in the system.
	// Note that there are two separate lists of springs:
	// springs and zeroLengthSprings. Don't forget to add up the forces from both.

	//add zero length springs to vector
	for (list<ZeroLengthSpring>::iterator i = zeroLengthSprings.begin(); i != zeroLengthSprings.end(); ++i)
	{
		i->addForcesToVector(x, forceVector);
	}

	//add normal springs to vector
	for (int i = 0; i < springs.size(); i++)
	{
		springs[i].addForcesToVector(x, forceVector);
	}
	
	//add stretch springs to vector
	for (int i = 0; i < stretch_springs.size(); i++)
	{
		stretch_springs[i].addForcesToVector(x, forceVector);
	}



	if (enableGravity) {
		// TODO (Part 1): Implement gravity.
		for (int i = 1; i < x.size(); i += 3) {
			forceVector[i] += G;
		}

	}



	// NOTE: Velocity currently unused because we aren't modeling drag forces.
	return forceVector;
}

// Compute the Jacobian matrix of the force vector, given the current positions
// and velocities.
MS_SparseSquare ParticleSystem::computeForceJacobian(const dVector& x, const dVector& v) {
	// The force Jacobian is a square matrix with 3x3 blocks, where the block
	// in row i, column j indicates the derivative of the force on particle i
	// with respect to a change in position of particle j.
	// Note that this matrix is symmetric.

	// The positions vector has 3n entries (where n is the number of particles),
	// so the force Jacobian has 3n x 3n entries.
	MS_SparseSquare forceJacobian(numParticles);

	// TODO (Part 3): Accumulate the force Jacobian blocks from all springs in the system.

	// add zero length springs jacobian
	for (list<ZeroLengthSpring>::iterator i = zeroLengthSprings.begin(); i != zeroLengthSprings.end(); ++i)
	{
		i->addBlocksToMatrix(x, forceJacobian);
	}

	// add non zero length springs jacobian
	for (int i = 0; i < springs.size(); i++)
	{
		springs[i].addBlocksToMatrix(x, forceJacobian);
	}




	// NOTE: Velocity currently unused because we aren't modeling drag forces.
	return forceJacobian;
}

// Compute the vector of accelerations experienced by each particle.
dVector ParticleSystem::computeAccelerations(const dVector& x, const dVector& v, const dVector& m) {
	

	// TODO (Part 1): Implement computation of accelerations from forces.
	dVector acceleration = dVector::Zero(x.size());

	//a = f/m
	dVector forces = computeForceVector(x, v);
	for (int i = 0; i < m.size(); i++) {
		acceleration[3 * i] += forces[3 * i] / m[i];
		acceleration[3 * i + 1] += forces[3 * i + 1] / m[i];
		acceleration[3 * i + 2] += forces[3 * i + 2] / m[i];

	}

	return acceleration;
}

// Integrate one time step with Forward Euler.
void ParticleSystem::integrate_FE(double delta) {

	// TODO (Part 1): Implement forward Euler.
	//enoch's code

	//p2 = p1+ vdt
	positions += velocities * delta;
	moveHead(delta);
	dVector a = computeAccelerations(positions, velocities, masses);
	//v=u+at
	velocities += a * delta;
	manageCollisions();

}

// Integrate one time step with Symplectic Euler.
void ParticleSystem::integrate_SE(double delta) {
	dVector force = computeForceVector(positions, velocities);

	// TODO (Part 2): Implement symplectic Euler.
	//@enoch
	for (int i = 0; i < positions.size(); i++)
	{
		velocities[i] = velocities[i] + (force[i] / masses[i / 3]) * delta;
		positions[i] = positions[i] + (velocities[i] * delta);
	}
	moveHead(delta);
	manageCollisions();

}

// Take one step of Newton's method.
// delta is the timestep.
// x_n, v_n, and masses are all just from the current state of the system.
// v_guess is our current estimate for the velocity from the next timestep.
// You should return the updated estimate for the next velocity.
dVector ParticleSystem::newtonStep(double delta, dVector& x_n, dVector& v_n, dVector& masses, dVector& v_guess) {

	// TODO (Part 3): Implement Newton's method.
	dVector new_v_guess = dVector::Zero(x_n.size());

	dVector x = x_n + delta * v_guess;

	//get force jacobian
	MS_SparseSquare A = computeForceJacobian(x, v_guess);
	dVector F = computeForceVector(x, v_guess);


	//linear equation
	A *= -1 * pow(delta, 2);
	A.addMassAlongDiagonal(masses);

	dVector temp = (v_n - v_guess) * masses;
	dVector b = delta * F;
	for (int i = 0; i < b.size(); i++)
	{
		b[i] += temp[i / 3];
	}




	new_v_guess = v_guess + A.solve(b);


	return new_v_guess;

}

// Integrate one time step with Backward Euler.
void ParticleSystem::integrate_BE(double delta) {

	// TODO (Part 3): Implement backward Euler.

	dVector new_v;
	dVector guess = velocities;


	for (int i = 0; i < 1; i++)
	{


		new_v = newtonStep(delta, positions, velocities, masses, guess);



		double mag = sqrt((new_v - guess).transpose() * (new_v - guess));
		if (mag >= 0 && mag < pow(10, -2))
		{
			break;
		}
		guess = new_v;
	}

	velocities = new_v;
	positions = positions + delta * velocities;
	moveHead(delta);
	manageCollisions();
}

void ParticleSystem::integrate_Pxr(double outer, double force_dt, double damping_dt)
{
	//check collisions
	//integrate forces: for loop
	//integrate damping: for loop
	//update
	manageCollisions();
}

//get hair to collide with the head-sphere
void ParticleSystem::manageCollisions() {
	if (enableCollision) {
		//go through particles
	int n = numParticles;
	auto c = getCenter();
	for (int i = 0; i < n; i++) {
		//get x and check to see whether or not it's inside the sphere
		V3D x = V3D(positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]) - c;
		double dist = x.norm();
 		if (dist < radius) {
			//particle is in the sphere, so im gonna push it back to the surface
			x.normalize();
			x *= radius; //basically have it be at the same anggle, kinda just pushed backwards if loooking at the center
			V3D new_pos = c + x;
			setPosition(i, P3D(new_pos));

			//now to figure out its new velocity. first, i need the current velocity
			V3D v = getVelocity(i);
			V3D v_2 = reflectVector(v, x.normalized());
			//the same momentum atm, but i think it will be better w some damping, how about v = 0.6 original v
			v_2 *= 0.6;
			setVelocity(i, v_2);
		}
	}
	}
	

}


V3D ParticleSystem::reflectVector(V3D inputVector, V3D normal) {
	double dn = 2 * inputVector.dot(normal);
	return inputVector - normal * dn;
}

void ParticleSystem::drawParticleSystem() {

	// Copy particle positions into array
	for (int i = 0; i < positions.size(); i++) {
		positionArray[i] = positions[i];
	}

	if (drawParticles && numParticles > 0) {
		// Draw all particles as red dots
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_DOUBLE, 0, &(positionArray.front()));

		glColor3d(0.8, 0.2, 0.2);
		glPointSize(16);
		glDrawElements(GL_POINTS, numParticles, GL_UNSIGNED_INT, &(pointsIndexArray.front()));

		glDisableClientState(GL_VERTEX_ARRAY);
	}

	if (mesh && drawFaces) {
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_DOUBLE, 0, &(positionArray.front()));

		glColor3d(0.2, 0.4, 0.6);
		mesh->drawMeshElements();

		glDisableClientState(GL_VERTEX_ARRAY);
	}

	if (drawSprings && springs.size() > 0) {
		// Draw all springs as green lines
		edgesIndexArray.clear();
		for (auto& spring : springs) {
			edgesIndexArray.push_back((unsigned int)spring.p0());
			edgesIndexArray.push_back((unsigned int)spring.p1());
		}

		glEnableClientState(GL_VERTEX_ARRAY);
		//		glEnable(GL_LINE_STIPPLE);
		//		glLineStipple(1, 0x00FF);
		glLineWidth(6.0);

		glVertexPointer(3, GL_DOUBLE, 0, &(positionArray.front()));
		glColor3d(0.2, 0.8, 0.2);
		glDrawElements(GL_LINES, springs.size() * 2, GL_UNSIGNED_INT, &(edgesIndexArray.front()));

		glDisable(GL_LINE_STIPPLE);
		glDisableClientState(GL_VERTEX_ARRAY);
	}

	if (drawStretchSprings && stretch_springs.size() > 0) {
		// Draw all springs as green lines
		edgesIndexArray.clear();
		for (auto& spring : stretch_springs) {
			edgesIndexArray.push_back((unsigned int)spring.p0());
			edgesIndexArray.push_back((unsigned int)spring.p1());
		}

		glEnableClientState(GL_VERTEX_ARRAY);
		//		glEnable(GL_LINE_STIPPLE);
		//		glLineStipple(1, 0x00FF);
		glLineWidth(6.0);

		glVertexPointer(3, GL_DOUBLE, 0, &(positionArray.front()));
		glColor3d(0.2, 0.8, 0.2);
		glDrawElements(GL_LINES, stretch_springs.size() * 2, GL_UNSIGNED_INT, &(edgesIndexArray.front()));

		glDisable(GL_LINE_STIPPLE);
		glDisableClientState(GL_VERTEX_ARRAY);
	}

	if (drawZeroLengthSprings && zeroLengthSprings.size() > 0) {
		edgesIndexArray.clear();
		zlSpringPositionArray.clear();
		int counter = 0;
		for (auto& spring : zeroLengthSprings) {
			int base = 3 * spring.getParticle();
			zlSpringPositionArray.push_back(positions[base]);
			zlSpringPositionArray.push_back(positions[base + 1]);
			zlSpringPositionArray.push_back(positions[base + 2]);
			zlSpringPositionArray.push_back(spring.getRestPosition().at(0));
			zlSpringPositionArray.push_back(spring.getRestPosition().at(1));
			zlSpringPositionArray.push_back(spring.getRestPosition().at(2));

			edgesIndexArray.push_back(counter);
			edgesIndexArray.push_back(counter + 1);
			counter += 2;
		}

		glEnableClientState(GL_VERTEX_ARRAY);
		//		glEnable(GL_LINE_STIPPLE);
		//		glLineStipple(1, 0x00FF);
		glLineWidth(6.0);

		glVertexPointer(3, GL_DOUBLE, 0, &(zlSpringPositionArray.front()));
		glColor3d(0.2, 0.2, 0.8);
		glDrawElements(GL_LINES, zeroLengthSprings.size() * 2, GL_UNSIGNED_INT, &(edgesIndexArray.front()));

		//		glDisable(GL_LINE_STIPPLE);
		glDisableClientState(GL_VERTEX_ARRAY);
	}
}

void ParticleSystem::setMesh(GLMesh* m) {
	mesh = m;
}

//move the center and update the static spring ends
void ParticleSystem::moveHead(double delta)
{
	center += head_speed * delta;
	for (list<ZeroLengthSpring>::iterator i = zeroLengthSprings.begin(); i != zeroLengthSprings.end(); ++i)
	{
		P3D new_scalp_spot = i->getRestPosition();
		new_scalp_spot += head_speed * delta;
		i->setRestPosition(new_scalp_spot);
		setPosition(i->getParticle(), new_scalp_spot);
	}
}

void ParticleSystem::saveData() {
	ofstream outfile("../results/" + inputName + ".txt");

	outfile << capturedData << std::endl;

	outfile.close();
	//ready_to_quit = true;
}
