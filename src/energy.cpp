#include "energy.hpp"

#include <Eigen/Geometry>
#include <math.h>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/constants/constants.hpp>

#include <iostream>

const float ENERGY_THRESHOLD = 0.000001f;
const float ZERO_THRESHOLD   = 0.000000000001f;
const double PI = boost::math::constants::pi<double>();

const double ELASTICITY = 0.20;
const double STRUT_REST_LENGTH = 2.5;

inline bool is_improvement(float orig, float changed) {
    return orig > changed && abs(orig - changed) > ENERGY_THRESHOLD;
}

/* ENERGY ===================================================================*/
Energy::Energy(float step_size_start, float step_size_end, int numparams) :
    step_size    (step_size_start),
    step_size_end(step_size_end  ),
    numparams(numparams),
    speed(1.0) {}

bool Energy::iterate() {
    if (step_size <= step_size_end) { return true; }

    step_size = update_step_size(step_size, step_size_end);
    if (step_size <= step_size_end) { return true; }

    // every coordinate of a vector will all equal coordinates and a norm of
    // step_size has the following value:
    float dparam = step_size;

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
        (*chg) *= step_size * speed;
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

    log_energies();
    
    delete chg ;
    delete diff;

    for (int np = 0; np < numparams; np++) {
        if (!done[np]) { delete done; return false; }
    }
    delete done;
    return true;
}

void Energy::set_speed(double speed) {
    this->speed = speed;
}

double Energy::get_speed() {
    return speed;
}

/* LINE ENERGY ===============================================*/

LineEnergy::LineEnergy(
    SplineCoaster *torus, double twist_weight) :
    Energy(0.1f, 0.00001f, torus->getNumControlPoints() * 3), //multiply by 3 for vec3
    torus(torus),
    twist_weight(twist_weight) {}

void LineEnergy::apply_change(VectorXf *chg) {
	for (int pi = 0; pi < chg->size(); pi += 3) {
		double dx = (*chg)[pi];
		double dy = (*chg)[pi+1];
		double dz = (*chg)[pi+2];

		torus->changePoint(pi/3, dx, dy, dz, 0.0, 0.0);
	}
    torus->compensateTwist();
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

    double twist = torus->getGlobalTwist() * PI / 180;
    energy += twist_weight * (twist*twist);

    return (float) energy;
}

double LineEnergy::compute_integrand(double t, double dt) {
    // We calculate the line integral of the bending energy of the curve as
    // follows. First, the deviation from PI radians of the bending angle
    // between two adjacent structs is computed. Dividing this by the average
    // length of the two struts yields the curvature at the given point, since
    // curvate is bending per unit length.

	SplinePoint point1 = torus->sample(t-dt);
	SplinePoint point2 = torus->sample(t);
	SplinePoint point3 = torus->sample(t+dt);

	vec3 pv1 = point1.point;
	vec3 pv2 = point2.point;
	vec3 pv3 = point3.point;

	vec3 strut1 = pv1 - pv2;
	vec3 strut2 = pv3 - pv2;
    double strut1l = strut1.length();
    double strut2l = strut2.length();

    double normdot = strut1 * strut2 / (strut1l * strut2l);
	double k1 = PI - acos(CLAMP(normdot, -1.0, 1.0));

    // Next, the energy due to stretching or squashing the struts away from
    // their rest lengths is computed. The energies of the two adjacent struts
    // is averaged. This penalizes deviation from the rest length, which in
    // turn prevents unbounded growth or shrinkage.
    //
    // Note that this formulation comes directly from Hooke's law.
    
    double stretch1 = STRUT_REST_LENGTH - strut1l;
    double stretch2 = STRUT_REST_LENGTH - strut2l;
    double springpe1 = 0.5 * ELASTICITY * stretch1 * stretch1;
    double springpe2 = 0.5 * ELASTICITY * stretch2 * stretch2;
    double avgpe = (springpe1 + springpe2) / 2.0;

    // With the principal curvature around the arm computed, we need to compute
    // the integrand itself, then integrate over the local length. The former
    // is simply the square of the principal curvature. To approximate
    // integration over the path, we compute the averaged length of the four
    // struts that meet at the central point in question and multiply the
    // integration term by that value.
    //
    // We also add in the square potential energy of the stretched or squashed
    // struts to prevent unbounded growth.

    double bending = (1-twist_weight) * (k1*k1);
    double stretch = avgpe * avgpe;

    return bending + stretch;
}

void LineEnergy::log_iteration(float step_size) {
    float e = calc_energy();
    std::cout << "Iterated. Energy: " << e <<
               ", step_size = " << step_size <<
               ", twist = " << torus->getGlobalTwist() << std::endl;
}

void LineEnergy::log_energies() {
	int numPoints = torus->getNumControlPoints();
	double dt = 1.0/numPoints;

    double t;
    double bending = 0.0;
    double stretch = 0.0;
    for (int v = 0; v < numPoints; v++) {
        t = ((double) v)/numPoints;

        SplinePoint point1 = torus->sample(t-dt);
        SplinePoint point2 = torus->sample(t);
        SplinePoint point3 = torus->sample(t+dt);

        vec3 pv1 = point1.point;
        vec3 pv2 = point2.point;
        vec3 pv3 = point3.point;

        vec3 strut1 = pv1 - pv2;
        vec3 strut2 = pv3 - pv2;
        double strut1l = strut1.length();
        double strut2l = strut2.length();

        double normdot = strut1 * strut2 / (strut1l * strut2l);
        double k1 = PI - acos(CLAMP(normdot, -1.0, 1.0));

        double stretch1 = STRUT_REST_LENGTH - strut1l;
        double stretch2 = STRUT_REST_LENGTH - strut2l;
        double springpe1 = 0.5 * ELASTICITY * stretch1 * stretch1;
        double springpe2 = 0.5 * ELASTICITY * stretch2 * stretch2;
        double avgpe = (springpe1 + springpe2) / 2.0;

        bending += (1-twist_weight) * (k1*k1);
        stretch += avgpe * avgpe;
    }

    double twist = torus->getGlobalTwist() * PI / 180;
    double tenergy = twist_weight * (twist*twist);

    cout << "B: " << bending <<
          ", T: " << tenergy <<
          ", S: " << stretch << endl;
}

float LineEnergy::update_step_size(float old, float end) {
    // Let the optimization loop deal with the step size; we don't need to
    // enforce additional constraints.
	return old;
}
/* KBM TORUS ENERGY =========================================================*/

KBMEnergy::KBMEnergy(
    KBMTorus *torus, double twist_weight) :
    // each control point has x, y and z components, plus a cross-section
    // scale. Additionally, there is a tilt for the end cap.
    Energy(0.1f, 0.00001f, torus->getNumMovableControlPoints() * 4 + 1),
    torus(torus),
    twist_weight(twist_weight) {
    log_energies();
}

void KBMEnergy::apply_change(VectorXf *chg) {
    int numParams = chg->size();
    int numPointsLeft = torus->getNumMovableControlPoints(KBMTorus::LEFTARM);
    int numPointsRght = torus->getNumMovableControlPoints(KBMTorus::RGHTARM);

	for (int pi = 0; pi < numPointsLeft * 4; pi += 4) {
		double dx   = (*chg)[pi  ];
		double dy   = (*chg)[pi+1];
		double dz   = (*chg)[pi+2];
        double dcss = (*chg)[pi+3];

		torus->changePoint(KBMTorus::LEFTARM,
            pi/4 + 3, dx, dy, dz, dcss, 0);
	}

	for (int pi = 0; pi < numPointsRght * 4; pi += 4) {
		double dx   = (*chg)[numPointsLeft*4+pi  ];
		double dy   = (*chg)[numPointsLeft*4+pi+1];
		double dz   = (*chg)[numPointsLeft*4+pi+2];
        double dcss = (*chg)[numPointsLeft*4+pi+3];

		torus->changePoint(KBMTorus::RGHTARM,
            pi/4 + 3, dx, dy, dz, dcss, 0);
	}

    torus->changeTorTilt((*chg)[numParams - 1]);

    torus->compensateTwist();
}

float KBMEnergy::calc_energy() {
    return calc_arm_energy(KBMTorus::LEFTARM) +
           calc_arm_energy(KBMTorus::RGHTARM);
}

float KBMEnergy::calc_arm_energy(KBMTorus::ArmType whicharm) {
	int numPoints = torus->getNumControlPoints(whicharm);

    double bending = 0.0;
    double lenchgs = 0.0;
    double csschgs = 0.0;
    double twistpn = 0.0;
    for (int t = 1; t < numPoints - 1; t++) {
        SplinePoint point1 = torus->getPoint(whicharm, t-1);
        SplinePoint point2 = torus->getPoint(whicharm, t);
        SplinePoint point3 = torus->getPoint(whicharm, t+1);

        vec3 pv1 = point1.point;
        vec3 pv2 = point2.point;
        vec3 pv3 = point3.point;

        vec3 strut1 = pv1 - pv2;
        vec3 strut2 = pv3 - pv2;
        double strut1l = strut1.length();
        double strut2l = strut2.length();

        double devlen = abs(log(strut1l / strut2l));
        lenchgs += devlen * devlen;

        double normdot = strut1 * strut2 / (strut1l * strut2l);
        double k1 = PI - acos(CLAMP(normdot, -1.0, 1.0));

        bending += k1 * k1;

        double r2 = point2.crossSectionScale;
        double r3 = point3.crossSectionScale;
        double rchg = (max(r2,r3) / min(r2, r3) - 1) / strut2l;
        csschgs += rchg * rchg;
    }

    twistpn = torus->getGlobalTwist(whicharm);
    twistpn = twistpn * twistpn;
    twistpn *= 10; // TODO: needs to be variable

    return bending + lenchgs + csschgs + twistpn;
}

void KBMEnergy::log_iteration(float step_size) {
    float e = calc_energy();
    std::cout << "Iterated. Energy: " << e <<
               ", step_size = " << step_size << std::endl;
}

void KBMEnergy::log_energies() {
    // TODO
    KBMTorus::ArmType whicharm = KBMTorus::RGHTARM;
	int numPoints = torus->getNumControlPoints(whicharm);

    cout << ">>> ";
    for (int t = 1; t < numPoints - 1; t++) {
        SplinePoint point1 = torus->getPoint(whicharm, t-1);
        SplinePoint point2 = torus->getPoint(whicharm, t);
        SplinePoint point3 = torus->getPoint(whicharm, t+1);

        vec3 pv1 = point1.point;
        vec3 pv2 = point2.point;
        vec3 pv3 = point3.point;

        vec3 strut1 = pv1 - pv2;
        vec3 strut2 = pv3 - pv2;
        double strut1l = strut1.length();
        double strut2l = strut2.length();

        double normdot = strut1 * strut2 / (strut1l * strut2l);
        double angle = acos(CLAMP(normdot, -1.0, 1.0));

        double r2 = point2.crossSectionScale;
        double r3 = point3.crossSectionScale;
        double rchg = (max(r2,r3) / min(r2, r3) - 1) / strut2l;

        double angleDeg = angle * 180.0 / PI;

        if (t == 1)
            cout << " (" << strut1l << ") ";
        cout << t << "->" << angleDeg << "," << point2.azimuth;
        cout << " (" << strut2l << ", " << rchg << " (" << r2 << "->" << r3 << ")" << ") ";
    }

    cout << " " << torus->getGlobalTwist(whicharm);
    cout << " " << torus->getTorTilt();
    cout << endl;
}

float KBMEnergy::update_step_size(float old, float end) {
    // Let the optimization loop deal with the step size; we don't need to
    // enforce additional constraints.
	return old;
}
