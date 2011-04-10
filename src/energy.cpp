#include "energy.hpp"

#include <Eigen/Geometry>
#include <math.h>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/constants/constants.hpp>

#include <iostream>

const float ENERGY_THRESHOLD = 0.0001f;
const float ZERO_THRESHOLD   = 0.000000000001f;
const double PI = boost::math::constants::pi<double>();

inline bool is_improvement(float orig, float changed) {
    return orig > changed && abs(orig - changed) > ENERGY_THRESHOLD;
}

/* ENERGY ===================================================================*/
Energy::Energy(float step_size_start, float step_size_end, int numparams) :
    step_size    (step_size_start),
    step_size_end(step_size_end  ),
    numparams(numparams) {}

bool Energy::iterate() {
    if (step_size <= step_size_end) { return true; }

    step_size = update_step_size(step_size, step_size_end);
    if (step_size <= step_size_end) { return true; }

    // every coordinate of a vector will all equal coordinates and a norm of
    // step_size has the following value:
    float dparam = step_size / sqrt((float) numparams);

    float energy_now = calc_energy();

    VectorXf *diff = new VectorXf(numparams);
    VectorXf *chg  = new VectorXf(numparams);

    for (int np = 0; np < numparams; np++) {
        (*diff)[np] = 0.0f;
    }

    bool *done = new bool[numparams];

    for (int np = 0; np < numparams; np++) {
        (*chg )[np] = 0;
        done[np] = false;

        (*diff)[np] = dparam;
        apply_change(diff);
        float energy_large = calc_energy();

        (*diff)[np] = -2 * dparam;
        apply_change(diff);
        float energy_small = calc_energy();

        (*diff)[np] = dparam;
        apply_change(diff);

        (*diff)[np] = 0;

        if (!boost::math::isnan(energy_small) &&
            !boost::math::isnan(energy_large)) {
            if (!is_improvement(energy_now, energy_large) &&
                !is_improvement(energy_now, energy_small)) {
                if (step_size <= step_size_end) { done[np] = true; }
                continue;
            }

            (*chg)[np] = energy_small - energy_large;
        }
        else {
            (*chg)[np] = 0.0f;
        }
    }

    bool chg_all_zeros = true;
    for (int np = 0; np < numparams; np++) {
        chg_all_zeros = chg_all_zeros && ((*chg)[np] == 0.0f);
    }

    if (!chg_all_zeros) {
        chg->normalize();
        (*chg) *= step_size;
        apply_change(chg);

        float energy_new = calc_energy();

        if (!is_improvement(energy_now, energy_new) &&
            step_size > step_size_end) {
            (*chg) *= -1;
            apply_change(chg);
            step_size /= 2;
        }

        log_iteration(step_size);
    }
    else if (step_size > step_size_end) {
        step_size /= 2;
    }
    
    delete chg ;
    delete diff;

    for (int np = 0; np < numparams; np++) {
        if (!done[np]) { delete done; return false; }
    }
    delete done;
    return true;
}

/* LINE ENERGY ===============================================*/

LineEnergy::LineEnergy(
    SplineCoaster *torus) :
    Energy(0.1f, 0.00001f, torus->getNumControlPoints() * 3), //multiply by 3 for vec3
    torus(torus) {}

void LineEnergy::apply_change(VectorXf *chg) {
	for (int pi = 0; pi < chg->size(); pi += 3) {
		double dx = (*chg)[pi];
		double dy = (*chg)[pi+1];
		double dz = (*chg)[pi+2];

		torus->changePoint(pi/3, dx, dy, dz);
	}
}

float LineEnergy::calc_energy() {
    // The surface energy of a simple torus is the path integral of the square
    // of the principal curvatures along the arm (as opposed to around it).
    double energy = 0.0;

	int numPoints = torus->getNumControlPoints();
	double dt = 1.0/numPoints;

    for (int v = 0; v < numPoints; v++) {
        energy += compute_integrand(((double) v)/numPoints, dt);
    }

    return (float) energy;
}

double LineEnergy::compute_integrand(double t, double dt) {
    // This calculation is exactly the same as the corresponding one in
    // ParameterizedTorusEnergy, but it only computes along the length of the
    // arm. This makes this integral a path integral instead of a surface
    // integral.

	SplinePoint point1 = torus->sample(t-dt);
	SplinePoint point2 = torus->sample(t);
	SplinePoint point3 = torus->sample(t+dt);

	vec3 pv1 = point1.point;
	vec3 pv2 = point2.point;
	vec3 pv3 = point3.point;

	vec3 strut1 = pv1 - pv2;
	vec3 strut2 = pv3 - pv2;

	double bending = PI - acos(strut1 * strut2 / (strut1.length() * strut2.length()));
	double al = 0.5 * (strut1.length() + strut2.length());
    double k1 = bending / al;

    // With the principal curvature around the arm computed, we need to compute
    // the integrand itself, then integrate over the local length. The former
    // is simply the square of the principal curvature. To approximate
    // integration over the path, we compute the averaged length of the four
    // struts that meet at the central point in question and multiply the
    // integration term by that value.
    
    return (k1*k1) * al;
}

void LineEnergy::log_iteration(float step_size) {
    float e = calc_energy();
    std::cout << "Iterated. Energy: " << e << ", step_size = " << step_size << std::endl;
}

float LineEnergy::update_step_size(float old, float end) {
	return old;
}
