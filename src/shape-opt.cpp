#include <iostream>

#ifdef _WIN32
#	include <windows.h>
#else
#	include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include "util.hpp"

#include <Eigen/Core>
USING_PART_OF_NAMESPACE_EIGEN

using namespace std;

/**
 * The title of the main window.
 */
const char *TITLE = "Shape Optimization of Gridded Surfaces";

// Global Variables ===========================================================
Viewport viewport;

void reshape_window(int w, int h) {
    viewport.w = w;
    viewport.h = h;

    glViewport(0, 0, viewport.w, viewport.h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float scale = min(w, h);
    glOrtho(-w / scale, w / scale, -h / scale, h / scale, -1, 1);
}

void init_scene(){
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    reshape_window(viewport.w, viewport.h);
}

void draw_scene() {
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glBegin(GL_TRIANGLES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(-1.0f, 0.000f, 0.0f);

        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f( 1.0f, 0.000f, 0.0f);

        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f( 0.0f, 0.866f, 0.0f);
    glEnd();

    glFlush();
    glutSwapBuffers();
}

void move_frame() {
#ifdef _WIN32
    Sleep(10);
#endif
    glutPostRedisplay();
}

void handle_keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case ' ':
            exit(0);
            break;
    }
}

int main(int argc, char *argv[]) {
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

    viewport.w = 400;
    viewport.h = 400;

    glutInitWindowSize(viewport.w, viewport.h);
    glutInitWindowPosition(0, 0);
    glutCreateWindow(TITLE);

    init_scene();

    glutDisplayFunc (draw_scene     );
    glutReshapeFunc (reshape_window );
    glutIdleFunc    (move_frame     );
    glutKeyboardFunc(handle_keyboard);
    glutMainLoop();

    return 0;
}
