#include "model.hpp"

SimpleTorus::SimpleTorus(float ring_radius, int resolution) :
    ring_radius(ring_radius), ARM_RADIUS(0.5),
    ring_vert(resolution), arm_vert(resolution) {}

Vector3f SimpleTorus::get_point(int v, int a) {
    float r_ang = 2 * PI / ring_vert;
    float a_ang = 2 * PI / arm_vert;

    float d1 = ring_radius - ARM_RADIUS * cos(a * a_ang);
    return Vector3f(d1 * cos(v * r_ang), d1 * sin(v * r_ang), sin(a * a_ang));
}
