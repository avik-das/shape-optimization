#include "draw.hpp"

#include "util.hpp"
#include <math.h>

SimpleTorusDrawer::SimpleTorusDrawer(SimpleTorus *simple_torus) :
    simple_torus(simple_torus), ARM_RESOLUTION(10) {}

void SimpleTorusDrawer::draw() {
    glColor3f(0.0f, 0.0f, 0.0f);

    float r_ang = 2 * PI / simple_torus->num_vert;
    float a_ang = 2 * PI / ARM_RESOLUTION;

    float r = simple_torus->ring_radius;

    for (int a = 0; a < ARM_RESOLUTION; a++) {
        glBegin(GL_QUAD_STRIP);
            // we loop num_vert + 1 times, the last time being to finish off
            // the quad strip
            for (int i = 0; i <= simple_torus->num_vert; i++) {
                Vector3f radial =
                    (r + 0.5) * Vector3f(cos(i * r_ang), sin(i * r_ang), 0.0f);
                // set_normal(i * r_ang, a * a_ang);
                float d1 = r + 0.5 * (1 - cos(a * a_ang));
                Vector3f vd1 = Vector3f(
                    d1 * cos(i * r_ang), d1 * sin(i * r_ang), sin(a * a_ang));
                Vector3f n1 = vd1 - radial;
                glNormal3f(n1.x(), n1.y(), n1.z());
                glVertex3f(vd1.x(), vd1.y(), vd1.z());

                set_normal(i * r_ang, (a + 1) * a_ang);
                float d2 = r + 0.5 * (1 - cos((a + 1) * a_ang));
                Vector3f vd2 = Vector3f(
                    d2 * cos(i * r_ang), d2 * sin(i * r_ang),
                    sin((a + 1) * a_ang));
                Vector3f n2 = vd2 - radial;
                glNormal3f(n2.x(), n2.y(), n2.z());
                glVertex3f(vd2.x(), vd2.y(), vd2.z());
            }
        glEnd();
    }
}

void SimpleTorusDrawer::set_normal(float t_angle, float a_angle) {
    // First, we consider a cross-section of the arm. We compute the normal
    // vector radiating out from the center of the arm to the point in
    // question.  Then, we rotate it on the xy-plane to make the cross-section
    // line up with the toroidal angle.

    // Vector3f arm = Vector3f(-cos(a_angle), 0.0f, sin(a_angle));

    // Matrix3f rot;
    // rot << cos(t_angle), -sin(t_angle), 0,
    //        sin(t_angle),  cos(t_angle), 0,
    //                   0,             0, 1;
    // Vector3f n = rot * arm;
    // 
    // glNormal3f(n.x(), n.y(), n.z());

    // // tangent vector with respect to toroidal ring
    // Vector3f t = Vector3f(-sin(t_angle), cos(t_angle), 0);
    // // tangent vector with respect to arm
    // Vector3f r
    // sx = cos(t_angle)*(-sin(a_angle));
    // sy = sin(t_angle)*(-sin(a_angle));
    // sz = cos(a_angle);
    // /* normal is cross-product of tangents */
    // nx = ty*sz - tz*sy;
    // ny = tz*sx - tx*sz;
    // nz = tx*sy - ty*sx;
}
