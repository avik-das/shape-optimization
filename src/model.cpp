#include "model.hpp"

PointNormal::PointNormal(Vector3f *point, Vector3f *normal) :
    point(point), normal(normal) {}

#include <iostream>

PointNormal::~PointNormal() {
    delete point;
    delete normal;
}

SimpleTorus::SimpleTorus(float ring_radius, int resolution) :
    ring_radius(ring_radius), ARM_RADIUS(0.5),
    ring_vert(resolution), arm_vert(resolution) {}

PointNormal *SimpleTorus::get_point(int v, int a) {
    float r_ang = 2 * PI / ring_vert;
    float a_ang = 2 * PI / arm_vert;

    float d1 = ring_radius - ARM_RADIUS * cos(a * a_ang);
    Vector3f *p =
        new Vector3f(d1 * cos(v * r_ang), d1 * sin(v * r_ang), sin(a * a_ang));

    Vector3f radial =
        ring_radius * Vector3f(cos(v * r_ang), sin(v * r_ang), 0.0f);

    Vector3f *n = new Vector3f((*p) - radial);
    n->normalize();

    return new PointNormal(p, n);
}
