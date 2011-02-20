#include "energy.hpp"

#include "util.hpp"

#include <Eigen/Geometry>
#include <math.h>

SimpleTorusEnergy::SimpleTorusEnergy(SimpleTorus *simple_torus) :
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
    Vector3f av1 = get_point(v - 1, a),
             av2 = get_point(v    , a),
             av3 = get_point(v + 1, a);
    Vector3f as1 = av1 - av2,
             as2 = av3 - av2;
    double aa = acos(as1.dot(as2) / (as1.norm() * as2.norm()));
    double al = 0.5 * (as1.norm() + as2.norm());
    double k1 = aa / al;

    // Then with the same v.
    Vector3f vv1 = get_point(v, a - 1),
             vv2 = av2,
             vv3 = get_point(v, a + 1);
    Vector3f vs1 = vv1 - vv2,
             vs2 = vv3 - vv2;
    double va = acos(vs1.dot(vs2) / (vs1.norm() * vs2.norm()));
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

Vector3f SimpleTorusEnergy::get_point(int v, int a) {
    float r_ang = 2 * PI / simple_torus->ring_vert;
    float a_ang = 2 * PI / simple_torus->arm_vert;

    float d1 = simple_torus->ring_radius -
        simple_torus->ARM_RADIUS * cos(a * a_ang);
    return Vector3f(d1 * cos(v * r_ang), d1 * sin(v * r_ang), sin(a * a_ang));
}

#include <iostream>

void SimpleTorusEnergy::iterate() {
    const float STEP_SIZE = 0.1f;
    if (simple_torus->ring_radius <= STEP_SIZE) { return; }

    float energy_now = calc_energy();

    simple_torus->ring_radius += STEP_SIZE;
    float energy_large = calc_energy();

    simple_torus->ring_radius -= STEP_SIZE * 2;
    float energy_small = calc_energy();

    simple_torus->ring_radius += STEP_SIZE;
    if (energy_now < energy_large && energy_now < energy_small) { return; }
    simple_torus->ring_radius +=
        energy_large > energy_small ? -STEP_SIZE : +STEP_SIZE;

    float e = calc_energy();
    std::cout << "Iterated. Energy: " << e << ", Radius: " << simple_torus->ring_radius << std::endl;
}
