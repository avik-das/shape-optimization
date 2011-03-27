#ifndef DRAW_HPP
#define DRAW_HPP

#include "model.hpp"

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <Eigen/Core>
USING_PART_OF_NAMESPACE_EIGEN

/* TODO: document properly */

class Drawer {
public:
    virtual void draw() = 0;
};

class ParameterizedTorusDrawer : public Drawer {
public:
    ParameterizedTorusDrawer(ParameterizedTorus *torus);
    void draw();

private:
    ParameterizedTorus *torus;
};

#endif
