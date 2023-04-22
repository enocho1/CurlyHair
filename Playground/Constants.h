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

//okay here's the pixar stuff

//curly hair vars
#define DT_OUTER 0.000694416
#define DT_FORCE 4.62944e-05
#define DT_DAMPING 4.62944e-06 




#define NEWTON_TOLERANCE 0.01