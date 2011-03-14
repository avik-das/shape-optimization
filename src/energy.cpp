#include "energy.hpp"

#include "util.hpp"

#include <Eigen/Geometry>
#include <math.h>

#include <iostream>

const float ENERGY_THRESHOLD = 0.001f;

inline bool is_improvement(float orig, float changed) {
    return orig > changed && abs(orig - changed) > ENERGY_THRESHOLD;
}

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
    float dparam = step_size / sqrt(numparams);

    float energy_now = calc_energy();

    VectorXf *diff = new VectorXf(numparams);
    VectorXf *chg  = new VectorXf(numparams);

    bool *done = new bool[numparams];

    for (int np = 0; np < numparams; np++) {
        (*chg )[np] = 0;
        (*diff)[np] = 0;
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

        if (!is_improvement(energy_now, energy_large) &&
            !is_improvement(energy_now, energy_small)) {
            if (step_size <= step_size_end) { done[np] = true; }
            continue;
        }

        if (energy_large > energy_small) { (*chg)[np] = -dparam; }
        else                             { (*chg)[np] =  dparam; }
    }

    apply_change(chg);
    float energy_new = calc_energy();
    log_iteration(step_size);

    if (!is_improvement(energy_now, energy_new) &&
        step_size > step_size_end) { step_size /= 2; }
    
    delete chg ;
    delete diff;

    for (int np = 0; np < numparams; np++) {
        if (!done[np]) { delete done; return false; }
    }
    delete done;
    return true;
}

SimpleTorusEnergy::SimpleTorusEnergy(SimpleTorus *simple_torus) :
    Energy(0.1f, 0.00001f, 1 + simple_torus->ring_vert),
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

    for (int oi = 0; oi < simple_torus->ring_vert; oi++) {
        simple_torus->set_radial_offset(
            oi, simple_torus->get_radial_offset(oi) + (*chg)[oi + 1]);
    }
}

float SimpleTorusEnergy::update_step_size(float old, float end) {
    while (simple_torus->ring_radius <= old) {
        if (old > end) { old /= 2; }
        else { return -1.0f; }
    }

    return old;
}

void SimpleTorusEnergy::log_iteration(float step_size) {
    float e = calc_energy();
    std::cout << "Iterated. Energy: " << e << ", Radius: " << simple_torus->ring_radius << " (step_size = " << step_size << ")" << std::endl;
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
    // divided by the averaged lengths of the two struts in question.

    // First with the same a.
    Vector3f *av1 = simple_torus->get_point(v - 1, a),
             *av2 = simple_torus->get_point(v    , a),
             *av3 = simple_torus->get_point(v + 1, a);
    Vector3f as1 = *av1 - *av2,
             as2 = *av3 - *av2;
    double aa = PI - acos(as1.dot(as2) / (as1.norm() * as2.norm()));
    double al = 0.5 * (as1.norm() + as2.norm());
    double k1 = aa / al;

    // Then with the same v.
    Vector3f *vv1 = simple_torus->get_point(v, a - 1),
             *vv2 = av2,
             *vv3 = simple_torus->get_point(v, a + 1);
    Vector3f vs1 = *vv1 - *vv2,
             vs2 = *vv3 - *vv2;
    double va = PI - acos(vs1.dot(vs2) / (vs1.norm() * vs2.norm()));
    double vl = 0.5 * (vs1.norm() + vs2.norm());
    double k2 = va / vl;

    delete av1;
    delete av2;
    delete av3;
    delete vv1;
    delete vv2;
    delete vv3;

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

SimpleTorusEnergyStd::SimpleTorusEnergyStd(SimpleTorus *simple_torus) :
    SimpleTorusEnergy(simple_torus) {}

double SimpleTorusEnergyStd::compute_integrand(int v, int a) {
    // This calculation is conceptually very similar to the corresponding
    // SimpleTorusEnergy calculation, with the only difference being in the way
    // the bending angles are calculated. Instead of calculating the angle
    // between two opposite struts as the angle relative to each other, the
    // angle of deviation away from the tangential plane is calculated.

    // First with the same a.
    PointNormal *av1pn = simple_torus->get_point_normal(v - 1, a),
                *av2pn = simple_torus->get_point_normal(v    , a),
                *av3pn = simple_torus->get_point_normal(v + 1, a);
    Vector3f av1p = *av1pn->point,
             av2p = *av2pn->point,
             av3p = *av3pn->point;
    Vector3f an = *av2pn->normal;
    Vector3f as1 = av1p - av2p,
             as2 = av3p - av2p;
    
    double aa1 = acos(as1.dot(an) / as1.norm()) - PI / 2;
    double aa2 = acos(as2.dot(an) / as1.norm()) - PI / 2;
    double al = 0.5 * (as1.norm() + as2.norm());
    double k1 = (aa1 + aa2) / al;

    // Then with the same v.
    PointNormal *vv1pn = simple_torus->get_point_normal(v, a - 1),
                *vv3pn = simple_torus->get_point_normal(v, a + 1);
    Vector3f vv1p = *vv1pn->point,
             vv2p = av2p,
             vv3p = *vv3pn->point;
    Vector3f vn = an;
    Vector3f vs1 = vv1p - vv2p,
             vs2 = vv3p - vv2p;

    double va1 = acos(vs1.dot(vn) / vs1.norm()) - PI / 2;
    double va2 = acos(vs2.dot(vn) / vs1.norm()) - PI / 2;
    double vl = 0.5 * (vs1.norm() + vs2.norm());
    double k2 = (va1 + va2) / vl;

    delete av1pn;
    delete av2pn;
    delete av3pn;
    delete vv1pn;
    delete vv3pn;

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
