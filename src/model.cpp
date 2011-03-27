#include "model.hpp"
#include "Eigen/Geometry"

#include <iostream>
#include <sstream>

PointNormal::PointNormal(Vector3f *point, Vector3f *normal) :
    point(point), normal(normal) {}

PointNormal::~PointNormal() {
    delete point;
    delete normal;
}

ParameterizedTorus::ParameterizedTorus(int resolution) :
    ring_vert(resolution), arm_vert(resolution),
    ARM_RADIUS(0.5) {}

PointNormal *ParameterizedTorus::get_point_normal(int v, int a) {
    Vector3f *p = get_point(v, a);

    // now, we get the the four points around "p". Using these points, we look
    // at the four struts, which we can use construct the vectors normal to
    // each of the faces described by two adjacent struts. Averaging these
    // vectors gives us the normal at "p" itself.
    Vector3f *pa1 = get_point(v, a - 1),
             *pa2 = get_point(v, a + 1),
             *pv1 = get_point(v - 1, a),
             *pv2 = get_point(v + 1, a);

    Vector3f n1 = (*pv1 - *p).cross(*pa2 - *p),
             n2 = (*pa2 - *p).cross(*pv2 - *p),
             n3 = (*pv2 - *p).cross(*pa1 - *p),
             n4 = (*pa1 - *p).cross(*pv1 - *p);
    Vector3f *n = new Vector3f((n1 + n2 + n3 + n4) / 4.0f);
    n->normalize();

    // FIXME:
    // There's a weird bug if the intermediate vectors are deleted that results
    // in a bunch of strange values (including nan's). I need to fix it, but I
    // can't figure it out!
    delete pa1;
    delete pa2;
    delete pv1;
    delete pv2;

    return new PointNormal(p, n);
}

RadialTorus::RadialTorus(float ring_radius, int resolution) :
    ParameterizedTorus(resolution),
    ring_radius(ring_radius),
    radial_offsets(new std::vector<float>()) {
    float r_ang = 2 * PI / ring_vert;

    for (int oi = 0; oi < ring_vert; oi++) {
        radial_offsets->push_back(sin(2 * r_ang * oi));
    }
}

Vector3f *RadialTorus::get_point(int v, int a) {
    float r_ang = 2 * PI / ring_vert;
    float a_ang = 2 * PI / arm_vert;

    float d1 = ring_radius -
               ARM_RADIUS * cos(a * a_ang) +
               get_radial_offset(v);
    Vector3f *p =
        new Vector3f(d1 * cos(v * r_ang),
                     d1 * sin(v * r_ang),
                     ARM_RADIUS * sin(a * a_ang));

    return p;
}

void RadialTorus::apply_change(VectorXf *chg) {
    ring_radius += (*chg)[0];

    for (int oi = 0; oi < ring_vert; oi++) {
        set_radial_offset(oi, get_radial_offset(oi) + (*chg)[oi + 1]);
    }
}

float RadialTorus::update_step_size(float old, float end) {
    while (ring_radius <= old) {
        if (old > end) { old /= 2; }
        else { return -1.0f; }
    }

    return old;
}

int RadialTorus::numparams() {
    return 1 +        // the global radius
           ring_vert; // the radial offsets
}

float RadialTorus::get_radial_offset(int v) {
    return (*radial_offsets)[v % radial_offsets->size()];
}

void  RadialTorus::set_radial_offset(int v, float val) {
    (*radial_offsets)[v % radial_offsets->size()] = val;
}

std::string RadialTorus::str() {
    std::stringstream ss;
    ss << "Radius: " << ring_radius << ", (radial offsets omitted)";
    return ss.str();
}
