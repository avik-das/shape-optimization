#include "energy.hpp"

#include "util.hpp"

#include <Eigen/Geometry>
#include <math.h>

#include <iostream>

const float ENERGY_THRESHOLD = 0.0001f;
const float ZERO_THRESHOLD   = 0.000000000001f;

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
    float dparam = step_size / sqrt(numparams);

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

        if (!isnan(energy_small) && !isnan(energy_large)) {
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

/* PARAMETERIZED TORUS ENERGY ===============================================*/

ParameterizedTorusEnergy::ParameterizedTorusEnergy(
    ParameterizedTorus *torus) :
    Energy(0.1f, 0.00001f, torus->numparams()),
    torus(torus) {}

void ParameterizedTorusEnergy::apply_change(VectorXf *chg) {
    torus->apply_change(chg);
}

float ParameterizedTorusEnergy::update_step_size(float old, float end) {
    return torus->update_step_size(old, end);
}

float ParameterizedTorusEnergy::calc_energy() {
    // The surface energy of a simple torus is the integral of the sum of the
    // squares of the principal curvatures over the entire surface.
    double energy = 0.0;

    for (int a = 0; a < torus->arm_vert; a++) {
        for (int v = 0; v < torus->ring_vert; v++) {
            energy += compute_integrand(v, a);
        }
    }

    return (float) energy;
}

void ParameterizedTorusEnergy::log_iteration(float step_size) {
    float e = calc_energy();
    std::cout << "Iterated. Energy: " << e << ", Torus: [" << torus->str() << "], (step_size = " << step_size << ")" << std::endl;
}

double ParameterizedTorusEnergy::compute_integrand(int v, int a) {
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
    Vector3f *av1 = torus->get_point(v - 1, a),
             *av2 = torus->get_point(v    , a),
             *av3 = torus->get_point(v + 1, a);
    Vector3f as1 = *av1 - *av2,
             as2 = *av3 - *av2;
    double aa = PI - acos(as1.dot(as2) / (as1.norm() * as2.norm()));
    double al = 0.5 * (as1.norm() + as2.norm());
    double k1 = aa / al;

    // Then with the same v.
    Vector3f *vv1 = torus->get_point(v, a - 1),
             *vv2 = av2,
             *vv3 = torus->get_point(v, a + 1);
    Vector3f vs1 = *vv1 - *vv2,
             vs2 = *vv3 - *vv2;
    double va = PI - acos(vs1.dot(vs2) / (vs1.norm() * vs2.norm()));
    double vl = 0.5 * (vs1.norm() + vs2.norm());
    double k2 = va / vl;

    delete av1;
    delete av2;
    delete av3;
    delete vv1;
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

/* PARAMETERIZED TORUS ENERGY STD ===========================================*/

ParameterizedTorusEnergyStd::ParameterizedTorusEnergyStd(
    ParameterizedTorus *torus) :
    ParameterizedTorusEnergy(torus) {}

double ParameterizedTorusEnergyStd::compute_integrand(int v, int a) {
    // This calculation is conceptually very similar to the corresponding
    // ParameterizedTorusEnergy calculation, with the only difference being in
    // the way the bending angles are calculated. Instead of calculating the
    // angle between two opposite struts as the angle relative to each other,
    // the angle of deviation away from the tangential plane is calculated.

    // First with the same a.
    PointNormal *av1pn = torus->get_point_normal(v - 1, a),
                *av2pn = torus->get_point_normal(v    , a),
                *av3pn = torus->get_point_normal(v + 1, a);
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
    PointNormal *vv1pn = torus->get_point_normal(v, a - 1),
                *vv3pn = torus->get_point_normal(v, a + 1);
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

/* ENERGY ===================================================================*/

ParameterizedLineEnergy::ParameterizedLineEnergy(
    ParameterizedTorus *torus) :
    ParameterizedTorusEnergy(torus) {}

float ParameterizedLineEnergy::calc_energy() {
    // The surface energy of a simple torus is the path integral of the square
    // of the principal curvatures along the arm (as opposed to around it).
    double energy = 0.0;

    for (int v = 0; v < torus->ring_vert; v++) {
        energy += compute_integrand(v, 0.0f);
    }

    return (float) energy;
}

void ParameterizedLineEnergy::apply_change(VectorXf *chg) {
    // To prevent the unbounded growth of the torus, we ensure that the average
    // change among all the radial offsets is zero. To do this, we compute the
    // current average change among the radial offsets and subtract that number
    // from each of the radial offsets.
    float avg = 0.0f;
    unsigned int numparams  = chg->size();
    for (unsigned int ci = 1; ci < numparams; ++ci) {
        avg += (*chg)[ci];
    }
    avg /= numparams - 1;
    for (unsigned int ci = 1; ci < numparams; ++ci) { (*chg)[ci] -= avg; }
    
    // Also, the change in the global radius is zero.
    (*chg)[0] = 0.0f;
    torus->apply_change(chg);
}

double ParameterizedLineEnergy::compute_integrand(int v, int a) {
    // This calculation is exactly the same as the corresponding one in
    // ParameterizedTorusEnergy, but it only computes along the length of the
    // arm. This makes this integral a path integral instead of a surface
    // integral.

    Vector3f *av1 = torus->get_point(v - 1, a),
             *av2 = torus->get_point(v    , a),
             *av3 = torus->get_point(v + 1, a);
    Vector3f as1 = *av1 - *av2,
             as2 = *av3 - *av2;
    double aa = PI - acos(as1.dot(as2) / (as1.norm() * as2.norm()));
    double al = 0.5 * (as1.norm() + as2.norm());
    double k1 = aa / al;

    delete av1;
    delete av2;
    delete av3;

    // With the principal curvature around the arm computed, we need to compute
    // the integrand itself, then integrate over the local length. The former
    // is simply the square of the principal curvature. To approximate
    // integration over the path, we compute the averaged length of the four
    // struts that meet at the central point in question and multiply the
    // integration term by that value.
    
    double sl1 = as1.norm(),
           sl2 = as2.norm();

    return (k1*k1) * (sl1 + sl2) / 2;
}
