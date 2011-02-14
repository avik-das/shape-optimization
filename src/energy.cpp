#include "energy.hpp"

#include "util.hpp"
#include <math.h>

SimpleTorusEnergy::SimpleTorusEnergy(SimpleTorus *simple_torus) :
    simple_torus(simple_torus) {}

float SimpleTorusEnergy::calc_energy() {
    // The surface energy of a simple torus is the integral of the square of
    // the principal curvatures over the entire surface.
    //
    // TODO: precise terms
    // Firstly, at each point of the torus, the principal curvatures are the
    // curvature around two circles: the one around the "hole" and the one
    // around the "arm".
    //
    // Secondly, the integral is approximated by summing over only a subset of
    // the points on the torus, namely the vertices around the toroidal ring,
    // multiplied by the circumference of the toroidal ring.
    //
    // Because the torus is so simple, every vertex to be considered is
    // identical. The vertex is on the inside of the toroidal ring. The
    // principal curvatures at that vertex are the multiplicative inverse of the
    // radius of the toroidal ring, and the multiplicative inverse of the radius
    // of the arm. The latter radius is simply 1.0, while the the former radius
    // is stored with the torus.
    //
    // There are as many vertices as specified in the torus, but each one is
    // weighted by the inverse of the number of vertices, so it is sufficient to
    // calculate only the energy at one vertex.
    float r = simple_torus->ring_radius;

    float pc = 1.0f / r;
    float circ = PI * r * r;
    return (1 + pc * pc) * circ;
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
