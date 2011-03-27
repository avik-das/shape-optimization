#include "draw.hpp"

#include "util.hpp"
#include <math.h>

ParameterizedTorusDrawer::ParameterizedTorusDrawer(
    ParameterizedTorus *torus) :
    torus(torus) {}

void ParameterizedTorusDrawer::draw() {
    glColor3f(0.0f, 0.0f, 0.0f);

    for (int a = 0; a < torus->arm_vert; a++) {
        glBegin(GL_QUAD_STRIP);
            // we loop num_vert + 1 times, the last time being to finish off
            // the quad strip
            for (int i = 0; i <= torus->ring_vert; i++) {
                PointNormal *pn1 = torus->get_point_normal(i, a);
                Vector3f *n1 = pn1->normal;
                Vector3f *p1 = pn1->point ;

                glNormal3f(n1->x(), n1->y(), n1->z());
                glVertex3f(p1->x(), p1->y(), p1->z());

                PointNormal *pn2 = torus->get_point_normal(i, a + 1);
                Vector3f *n2 = pn2->normal;
                Vector3f *p2 = pn2->point ;

                glNormal3f(n2->x(), n2->y(), n2->z());
                glVertex3f(p2->x(), p2->y(), p2->z());

                delete pn1;
                delete pn2;
            }
        glEnd();
    }
}
