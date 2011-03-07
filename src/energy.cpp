#include "energy.hpp"

#include "util.hpp"

#include <Eigen/Geometry>
#include <math.h>

#include <iostream>

const float ENERGY_THRESHOLD = 0.001f;

inline bool is_improvement(float orig, float changed) {
    return orig > changed && abs(orig - changed) > ENERGY_THRESHOLD;
}

Energy::Energy(int nargs, ...) :
    step_size    (new VectorXf(nargs / 2)),
    step_size_end(new VectorXf(nargs / 2)) {
    va_list args;
    va_start(args, nargs);

    for (int r = 0; r < nargs / 2; r++) {
        double ssstart = va_arg(args, double);
        double ssend   = va_arg(args, double);
        std::cout << r << ": [" << ssstart << ", " << ssend << "]" << std::endl;
        (*step_size    )[r] = ssstart;
        (*step_size_end)[r] = ssend  ;
    }
}

bool Energy::iterate() {
    if (!step_size) { return true; }

    step_size = update_step_size(step_size, step_size_end);
    if (!step_size) { return true; }

    float energy_now = calc_energy();

    VectorXf *diff = new VectorXf(step_size->size());
    VectorXf *chg  = new VectorXf(step_size->size());

    bool *done = new bool[step_size->size()];

    for (int np = 0; np < step_size->size(); np++) {
        (*chg )[np] = 0;
        (*diff)[np] = 0;
        done[np] = false;

        (*diff)[np] = (*step_size)[np];
        apply_change(diff);
        float energy_large = calc_energy();

        (*diff)[np] = -2 * (*step_size)[np];
        apply_change(diff);
        float energy_small = calc_energy();

        (*diff)[np] = (*step_size)[np];
        apply_change(diff);

        (*diff)[np] = 0;

        if (!is_improvement(energy_now, energy_large) &&
            !is_improvement(energy_now, energy_small)) {
            if ((*step_size)[np] > (*step_size_end)[np]) {
                (*step_size)[np] /= 2;
            }
            else { done[np] = true; }
            continue;
        }
        if (energy_large > energy_small) { (*chg)[np] = -(*step_size)[np]; }
        else                             { (*chg)[np] =  (*step_size)[np]; }
    }

    apply_change(chg);
    log_iteration(step_size);

    for (int np = 0; np < step_size->size(); np++) {
        if (!done[np]) { return false; }
    }
    return true;
}

SimpleTorusEnergy::SimpleTorusEnergy(SimpleTorus *simple_torus) :
    Energy(2, 0.1f, 0.00001f),
    simple_torus(simple_torus) {}

float SimpleTorusEnergy::calc_energy() {
    // The surface energy of a simple torus is the integral of the sum of the
    // squares of the principal curvatures over the entire surface.
    double energy = 0.0;

    for (int a = 0; a < simple_torus->arm_vert; a++) {
        for (int v = 0; v < simple_torus->ring_vert; v++) {
            energy += compute_integrand(v, a);
        }
    }

    return (float) energy;
}

void SimpleTorusEnergy::apply_change(VectorXf *chg) {
    simple_torus->ring_radius += (*chg)[0];
}

VectorXf *SimpleTorusEnergy::update_step_size(VectorXf *old, VectorXf *end) {
    float ss = (*old)[0];
    const float END_STEP_SIZE = (*end)[0];

    while (simple_torus->ring_radius <= ss) {
        if (ss > END_STEP_SIZE) { ss /= 2; }
        else { return NULL; }
    }

    VectorXf *v = new VectorXf(1);
    (*v)[0] = ss;
    return v;
}

void SimpleTorusEnergy::log_iteration(VectorXf *step_size) {
    float e = calc_energy();
    std::cout << "Iterated. Energy: " << e << ", Radius: " << simple_torus->ring_radius << " (step_size = " << (*step_size)[0] << ")" << std::endl;
}

double SimpleTorusEnergy::compute_integrand(int v, int a) {
    // This function does quite a bit, but that's because all of the
    // computations below make use of the same data over and over again. Thus,
    // it's better to just calculate it once.

    // We consider five points: the vertex itself, and the four vertices around
    // it that are found by varying either v or a (but not both) by plus or
    // minus 1.
    //
    // The principal curvature in one direction is approximated by the bending
    // angle between the two struts formed by the three points with the same
    // a, and the principal curvature in the other direction by the struts
    // formed by the three points with the same v. Each time, the angle is
    // divided by the averaged lengths of the two struts in question. Note that
    // in computing the angle, we want to measure the deviation from the planar
    // configuration, not the actual angle between th struts themselves.

    // First with the same a.
    Vector3f av1 = *simple_torus->get_point(v - 1, a)->point,
             av2 = *simple_torus->get_point(v    , a)->point,
             av3 = *simple_torus->get_point(v + 1, a)->point;
    Vector3f as1 = av1 - av2,
             as2 = av3 - av2;
    double aa = PI - acos(as1.dot(as2) / (as1.norm() * as2.norm()));
    double al = 0.5 * (as1.norm() + as2.norm());
    double k1 = aa / al;

    // Then with the same v.
    Vector3f vv1 = *simple_torus->get_point(v, a - 1)->point,
             vv2 = av2,
             vv3 = *simple_torus->get_point(v, a + 1)->point;
    Vector3f vs1 = vv1 - vv2,
             vs2 = vv3 - vv2;
    double va = PI - acos(vs1.dot(vs2) / (vs1.norm() * vs2.norm()));
    double vl = 0.5 * (vs1.norm() + vs2.norm());
    double k2 = va / vl;

    // With the principal curvatures computed, we need to compute the integrand
    // itself, then integrate over the local area. The former is simply the sum
    // of the squares of the principal curvatures. To approximate integration
    // over the surface, we compute the averaged area of the four quads that
    // meet at the central point in question and multiply the integration term
    // by that value.
    
    double qa1 = as1.cross(vs1).norm(),
           qa2 = vs1.cross(as2).norm(),
           qa3 = as2.cross(vs2).norm(),
           qa4 = vs2.cross(as1).norm();

    return (k1*k1 + k2*k2) * (qa1 + qa2 + qa3 + qa4) / 4;
}
