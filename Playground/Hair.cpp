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

void Hair::integrateInternalForces(double dtf, double dtd, const dVector& x, const dVector& v, dVector& f)
{
}
