#include "model.hpp"

SimpleTorus::SimpleTorus(float ring_radius, int resolution) :
    ring_radius(ring_radius), ring_vert(resolution), arm_vert(resolution) {}
