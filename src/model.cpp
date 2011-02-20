#include "model.hpp"

SimpleTorus::SimpleTorus(float ring_radius, int resolution) :
    ring_radius(ring_radius), ARM_RADIUS(0.5),
    ring_vert(resolution), arm_vert(resolution) {}
