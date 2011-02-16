#ifndef MODEL_HPP
#define MODEL_HPP

/* TODO: document properly */

/**
 * The simplest possible torus. The only degree of freedom is the radius of the
 * toroidal ring.
 */
class SimpleTorus {
public:
    SimpleTorus(float ring_radius, int resolution);

    float ring_radius;
    int ring_vert;
    int arm_vert;
};

#endif
