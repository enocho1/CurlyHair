#pragma once

#define DEFAULT_STIFFNESS 100
#define PIN_STIFFNESS 100
#define DEFAULT_MASS 0.1

////explicit stiff timestep
//#define DELTA_T 0.0001

////regular stiff timestep
//#define DELTA_T 0.016

//backward euler/implicit integration timestep
#define DELTA_T 0.0001


#define NEWTON_TOLERANCE 0.01