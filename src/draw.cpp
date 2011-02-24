#include "draw.hpp"

#include "util.hpp"
#include <math.h>

SimpleTorusDrawer::SimpleTorusDrawer(SimpleTorus *simple_torus) :
    simple_torus(simple_torus) {}

void SimpleTorusDrawer::draw() {
    glColor3f(0.0f, 0.0f, 0.0f);

    float r_ang = 2 * PI / simple_torus->ring_vert;
    float rr = simple_torus->ring_radius;

    for (int a = 0; a < simple_torus->arm_vert; a++) {
        glBegin(GL_QUAD_STRIP);
            // we loop num_vert + 1 times, the last time being to finish off
            // the quad strip
            for (int i = 0; i <= simple_torus->ring_vert; i++) {
                Vector3f radial =
                    rr * Vector3f(cos(i * r_ang), sin(i * r_ang), 0.0f);

                Vector3f vd1 = simple_torus->get_point(i, a);
                Vector3f n1 = vd1 - radial;
                n1.normalize();
                glNormal3f(n1.x(), n1.y(), n1.z());

                glVertex3f(vd1.x(), vd1.y(), vd1.z());

                Vector3f vd2 = simple_torus->get_point(i, a + 1);

                Vector3f n2 = vd2 - radial;
                n2.normalize();
                glNormal3f(n2.x(), n2.y(), n2.z());

                glVertex3f(vd2.x(), vd2.y(), vd2.z());
            }
        glEnd();
    }
}
