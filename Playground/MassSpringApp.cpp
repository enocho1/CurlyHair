#include <GUILib/GLUtils.h>
#include "MassSpringApp.h"
#include "Constants.h"

enum IntegratorType {
	forwardEuler, symplecticEuler, backwardEuler, pixarTaz
};

IntegratorType integrator = symplecticEuler;

double MassSpringApp::k = DEFAULT_STIFFNESS;

MassSpringApp::MassSpringApp() {
	setWindowTitle("enoch's super awesome curly hair simulator.");
	TwAddSeparator(mainMenuBar, "sep2", "");

	TwAddVarRW(mainMenuBar, "Draw Particles", TW_TYPE_BOOLCPP, &ParticleSystem::drawParticles, "");
	TwAddVarRW(mainMenuBar, "Draw Springs", TW_TYPE_BOOLCPP, &ParticleSystem::drawSprings, "");
	TwAddVarRW(mainMenuBar, "Draw Pins", TW_TYPE_BOOLCPP, &ParticleSystem::drawZeroLengthSprings, "");
	TwAddVarRW(mainMenuBar, "Draw Faces", TW_TYPE_BOOLCPP, &ParticleSystem::drawFaces, "");
	TwAddVarRW(mainMenuBar, "Enable Gravity", TW_TYPE_BOOLCPP, &ParticleSystem::enableGravity, "");
	TwAddVarRW(mainMenuBar, "Enable Collisions", TW_TYPE_BOOLCPP, &ParticleSystem::enableCollision, "");

	TwAddVarRW(mainMenuBar, "Use Global Stiffness", TW_TYPE_BOOLCPP, &Spring::useGlobalK, "");
	TwAddVarRW(mainMenuBar, "Spring Stiffness", TW_TYPE_DOUBLE, &Spring::globalK, "");

	TwAddVarRW(mainMenuBar, "Use Global Pin Stiffness", TW_TYPE_BOOLCPP, &ZeroLengthSpring::useGlobalK, "");
	TwAddVarRW(mainMenuBar, "Pin Stiffness", TW_TYPE_DOUBLE, &ZeroLengthSpring::globalK, "");

	showGroundPlane = false;
	exper_iter = 0; // you can change this to start from a different point
	no_expers = 330; // how many experiment files you've got

	//particleSystem = ParticleSystemLoader::loadFromMSS("../meshes/triangle.mss");
	//particleSystem = ParticleSystemLoader::loadFromMSS("../meshes/batch/experiment" + to_string(exper_iter) + ".mss");
	particleSystem = ParticleSystemLoader::loadFromMSS("../meshes/single_curly_hair.mss");
	//particleSystem = ParticleSystemLoader::loadFromOBJ("../meshes/bunny200.obj");
	
	//this is how i load in the meshes: @enoch the colour is defined by list of fixed in the particle system, so messing around w/the order will mess w/ the colours
	/*particleSystem->addMesh("../meshes/headcap.obj");
	particleSystem->addMesh("../meshes/eyes.obj");
	particleSystem->addMesh("../meshes/hairband.obj");
	particleSystem->addMesh("../meshes/head.obj");*/
	//particleSystem = ParticleSystemLoader::loadFromMSS("../meshes/experiment1.mss");

	//particleSystem = ParticleSystemLoader::loadFromOBJ("../meshes/bunny200.obj");

	pickedParticle = -1;

}

MassSpringApp::~MassSpringApp(void) {
	delete particleSystem;
}

const double PICK_DISTANCE = 0.04;

//triggered when mouse moves
bool MassSpringApp::onMouseMoveEvent(double xPos, double yPos) {
	if (pickedParticle > -1) {
		double modelViewMatrix[16];
		double projMatrix[16];
		GLint viewport[4];

		glGetDoublev(GL_MODELVIEW_MATRIX, modelViewMatrix);
		glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
		glGetIntegerv(GL_VIEWPORT, viewport);

		// Find the depth of the original point, after projection to screen
		double xScreen, yScreen, zScreen;

		gluProject(pickedPosition.at(0), pickedPosition.at(1), pickedPosition.at(2),
			modelViewMatrix, projMatrix, viewport,
			&xScreen, &yScreen, &zScreen);

		// Use the same depth to determine the altered position from screen coordinates

		double xWorld, yWorld, zWorld;

		gluUnProject(xPos, viewport[3] - yPos, zScreen,
			modelViewMatrix, projMatrix, viewport,
			&xWorld, &yWorld, &zWorld);

		pickedPosition = P3D(xWorld, yWorld, zWorld);
		particleSystem->setPosition(pickedParticle, pickedPosition);

		return false;
	}
	if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;

	return false;
}

bool MassSpringApp::pickParticle(double screenX, double screenY) {
	Ray mouseRay = camera->getRayFromScreenCoords(screenX, screenY);
	int particlePicked = -1;
	P3D closestPosition;
	double closestDistance = PICK_DISTANCE;

	for (int i = 0; i < particleSystem->particleCount(); i++) {
		P3D x = particleSystem->getPositionOf(i);
		double distance = mouseRay.getDistanceToPoint(x, NULL);
		if (distance < closestDistance) {
			closestDistance = distance;
			closestPosition = x;
			particlePicked = i;
		}
	}

	if (particlePicked > -1) {
		pickedParticle = particlePicked;
		pickedPosition = closestPosition;
		return true;
	}
	return false;
}

//triggered when mouse buttons are pressed
bool MassSpringApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (button == 0) {
		// Left mouse
		if (action == 1) {
			// Down
			pickParticle(xPos, yPos);
		}
		else {
			// Up
			pickedParticle = -1;
		}
	}

	else if (button == 1) {
		// Right mouse
		if (action == 1) {
			// Down
			if (pickParticle(xPos, yPos)) {
				particleSystem->togglePin(pickedParticle);
				pickedParticle = -1;
			}
		}
	}

	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) {
		return true;
	}

	return false;
}

//triggered when using the mouse wheel
bool MassSpringApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool MassSpringApp::onKeyEvent(int key, int action, int mods) {
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	if (action == 1) {
		if (key == 'G') {
			if (ParticleSystem::enableGravity) {
				Logger::consolePrint("Disabled gravity");
				ParticleSystem::enableGravity = false;
			}
			else {
				Logger::consolePrint("Enabled gravity");
				ParticleSystem::enableGravity = true;
			}
		}
		else if (key == 'K') {
			if (ParticleSystem::enableCollision) {
				Logger::consolePrint("Disabled collisions");
				ParticleSystem::enableCollision = false;
			}
			else {
				Logger::consolePrint("Enabled collisions");
				ParticleSystem::enableCollision = true;
			}
		}
		else if (key == 'F') {
			Logger::consolePrint("Using forward Euler");
			integrator = forwardEuler;
		}
		else if (key == 'P') {
			Logger::consolePrint("using pixar's integration method");
			integrator = pixarTaz;
		}
		else if (key == 'S') {
			Logger::consolePrint("Using symplectic Euler");
			integrator = symplecticEuler;
		}
		else if (key == 'B') {
			Logger::consolePrint("Using backward Euler");
			integrator = backwardEuler;
		}
		else if (key == 'R') {
			Logger::consolePrint("saving data...");
			particleSystem->saveData();
		}
		else if (key == 'J') {
			P3D current_center = particleSystem->getCenter();
			Logger::consolePrint("enoch current center: (%f, %f, %f), ", current_center[0], current_center[1], current_center[2]);
			V3D current_vel = particleSystem->getHeadSpeed();
			Logger::consolePrint("enoch current head velocity: (%f, %f, %f), ", current_vel[0], current_vel[1], current_vel[2]);

		}
	}

	return false;
}

bool MassSpringApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

	return false;
}


void MassSpringApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);
}

void MassSpringApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}


// Run the App tasks
void MassSpringApp::process() {

	if (particleSystem->ready_to_quit) {
		restart();
	}
	// Take enough steps so that we are always running in (close to) real time (24fps)
	int numSteps;
	if (integrator == pixarTaz)
	{
		numSteps = 60;
	}
	else {
		numSteps = (int)((1. / 24.) / DELTA_T);
	}


	if (numSteps < 1) numSteps = 1;
	for (int i = 0; i < numSteps; i++) {
		switch (integrator) {
		case forwardEuler: particleSystem->integrate_FE(DELTA_T); break;
		case symplecticEuler: particleSystem->integrate_SE(DELTA_T); break;
		case backwardEuler: particleSystem->integrate_BE(DELTA_T); break;
		case pixarTaz: particleSystem->integrate_Pxr(DT_OUTER); break;
		}
		if (pickedParticle > -1) {
			particleSystem->setVelocity(pickedParticle, V3D(0, 0, 0));
			particleSystem->setPosition(pickedParticle, pickedPosition);
		}
	}
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void MassSpringApp::drawScene() {
	particleSystem->drawParticleSystem();
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void MassSpringApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void MassSpringApp::restart() {
	appIsRunning = false;
	exper_iter += 1;
	if (exper_iter < no_expers) {
		//particleSystem = ParticleSystemLoader::loadFromMSS("../meshes/batch/experiment" + to_string(exper_iter) + ".mss");
		particleSystem = ParticleSystemLoader::loadFromMSS("../meshes/single_curly_hair.mss");
		//particleSystem = ParticleSystemLoader::loadFromMSS("../meshes/batch/experiment" + to_string(0) + ".mss");
		appIsRunning = true;
	}
	else {
		appIsRunning = false;
	}


}

bool MassSpringApp::processCommandLine(const std::string& cmdLine) {

	istringstream iss(cmdLine);

	string command, argument;

	if ((iss >> command >> argument)) {
		if (command == "load") {
			if (argument.length() < 5) {
				return true;
			}
			string extension = argument.substr(argument.length() - 4, argument.length() - 1);
			if (extension == ".obj") {
				ParticleSystem* ps = ParticleSystemLoader::loadFromOBJ(argument);
				if (ps) {
					delete particleSystem;
					particleSystem = ps;
					ParticleSystem::enableGravity = false;
					ParticleSystem::enableCollision = false;

				}
				else {
					Logger::consolePrint("Failed to load particle system from %s", argument.c_str());
				}
			}
			else if (extension == ".mss") {
				ParticleSystem* ps = ParticleSystemLoader::loadFromMSS(argument);
				if (ps) {
					delete particleSystem;
					particleSystem = ps;
					ParticleSystem::enableGravity = false;
					ParticleSystem::enableCollision = false;

				}
				else {
					Logger::consolePrint("Failed to load particle system from %s", argument.c_str());
				}
			}
			else {
				Logger::consolePrint("Extension not recognized: %s", extension.c_str());
			}
			return true;
		}
		else if (GLApplication::processCommandLine(cmdLine)) return true;
	}

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

