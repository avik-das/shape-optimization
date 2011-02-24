#ifndef MODEL_HPP
#define MODEL_HPP

#include "util.hpp"

#include <Eigen/Core>
USING_PART_OF_NAMESPACE_EIGEN

/* TODO: document properly */

/**
 * The simplest possible torus. The only degree of freedom is the radius of the
 * toroidal ring.
 */
class SimpleTorus {
public:
    SimpleTorus(float ring_radius, int resolution);

    float ring_radius;
    const float ARM_RADIUS;
    int ring_vert;
    int arm_vert;

    Vector3f get_point(int v, int a);
};

#endif
