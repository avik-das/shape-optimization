#include "model.hpp"
#include "Eigen/Geometry"

#include <iostream>

PointNormal::PointNormal(Vector3f *point, Vector3f *normal) :
    point(point), normal(normal) {}

PointNormal::~PointNormal() {
    delete point;
    delete normal;
}

SimpleTorus::SimpleTorus(float ring_radius, int resolution) :
    ring_radius(ring_radius), ARM_RADIUS(0.5),
    ring_vert(resolution), arm_vert(resolution) {}

PointNormal *SimpleTorus::get_point_normal(int v, int a) {
    Vector3f *p = get_point(v, a);

    // now, we get the the four points around "p". Using these points, we look
    // at the four struts, which we can use construct the vectors normal to
    // each of the faces described by two adjacent struts. Averaging these
    // vectors gives us the normal at "p" itself.
    Vector3f pa1 = *get_point(v, a - 1),
             pa2 = *get_point(v, a + 1),
             pv1 = *get_point(v - 1, a),
             pv2 = *get_point(v + 1, a);

    Vector3f n1 = (pv1 - *p).cross(pa2 - *p),
             n2 = (pa2 - *p).cross(pv2 - *p),
             n3 = (pv2 - *p).cross(pa1 - *p),
             n4 = (pa1 - *p).cross(pv1 - *p);
    Vector3f *n = new Vector3f((n1 + n2 + n3 + n4) / 4.0f);
    n->normalize();

    return new PointNormal(p, n);
}

Vector3f *SimpleTorus::get_point(int v, int a) {
    float r_ang = 2 * PI / ring_vert;
    float a_ang = 2 * PI / arm_vert;

    float d1 = ring_radius - ARM_RADIUS * cos(a * a_ang);
    Vector3f *p =
        new Vector3f(d1 * cos(v * r_ang),
                     d1 * sin(v * r_ang),
                     ARM_RADIUS * sin(a * a_ang));

    return p;
}
