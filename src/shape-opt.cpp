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
#include "model.hpp"
#include "draw.hpp"
#include "energy.hpp"

/**
 * The title of the main window.
 */
const char *TITLE = "Shape Optimization of Gridded Surfaces";

static const float D_ANGLE     = 22.50f; // In degrees
static const float D_TRANSLATE =  0.25f;

// Global Variables ===========================================================
Viewport viewport;

Drawer *drawer = NULL;
Energy *energy = NULL;

bool done = false;

void reshape_window(int w, int h) {
    viewport.w = w;
    viewport.h = h;

    glViewport(0, 0, viewport.w, viewport.h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float scale = min(w, h);
    glOrtho(-w / scale, w / scale, -h / scale, h / scale, -1, 1);

    glScalef(0.25f, 0.25f, 0.25f);
}

void init_lights() {
    glEnable(GL_LIGHTING);
    glShadeModel(GL_SMOOTH);
    
    GLfloat light_ambient [] = {0.2, 0.2 , 0.2, 1.0};
    GLfloat light_diffuse [] = {1.0, 0.95, 0.9, 1.0};
    GLfloat light_specular[] = {1.0, 1.0 , 1.0, 1.0};
    GLfloat light_position[] = {5.0, 5.0 , 5.0, 0.0};

    glLightfv(GL_LIGHT0, GL_AMBIENT , light_ambient );
    glLightfv(GL_LIGHT0, GL_DIFFUSE , light_diffuse );
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    glEnable(GL_LIGHT0);
}

void init_scene(){
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    init_lights();

    // glEnable(GL_DEPTH_TEST);
    // glDepthMask(GL_TRUE);
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK , GL_FILL);

    reshape_window(viewport.w, viewport.h);
}

void draw_scene() {
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(viewport.rotx, 1.0f, 0.0f, 0.0f);
    glRotatef(viewport.rotz, 0.0f, 0.0f, 1.0f);

    drawer->draw();

    glFlush();
    glutSwapBuffers();
}

void move_frame() {
#ifdef _WIN32
    Sleep(10);
#endif
    glutPostRedisplay();
    if (!done) { done = energy->iterate(); }
}

void handle_keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 27 /* ESC */:
            exit(0);
        case 'e':
            std::cout << energy->calc_energy() << std::endl;
    }
}

static float get_rotation(float angle, float delta) {
    angle += delta;

    while (angle <    0) { angle += 360; }
    while (angle >= 360) { angle -= 360; }

    return angle;
}

static void handle_special(int key, int x, int y) {
    // Get the compiler to stop complaining!
    x = y;

    switch (key) {
        case GLUT_KEY_UP:
            if ((glutGetModifiers()) == GLUT_ACTIVE_SHIFT) {
                // Translate.
                viewport.tz += D_TRANSLATE;
                break;
            }

            // Rotate.
            viewport.rotx = get_rotation(viewport.rotx, -D_ANGLE);
            break;
        case GLUT_KEY_DOWN:
            if ((glutGetModifiers()) == GLUT_ACTIVE_SHIFT) {
                // Translate.
                viewport.tz -= D_TRANSLATE;
                break;
            }

            // Rotate.
            viewport.rotx = get_rotation(viewport.rotx, D_ANGLE);
            break;
        case GLUT_KEY_LEFT:
            if ((glutGetModifiers()) == GLUT_ACTIVE_SHIFT) {
                // Translate.
                viewport.tx -= D_TRANSLATE;
                break;
            }

            // Rotate.
            viewport.rotz = get_rotation(viewport.rotz, -D_ANGLE);
            break;
        case GLUT_KEY_RIGHT:
            if ((glutGetModifiers()) == GLUT_ACTIVE_SHIFT) {
                // Translate.
                viewport.tx += D_TRANSLATE;
                break;
            }

            // Rotate.
            viewport.rotz = get_rotation(viewport.rotz, D_ANGLE);
            break;
    }
}

int main(int argc, char *argv[]) {
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

    viewport.w = 400;
    viewport.h = 400;
    viewport.rotx = 0.0f;
    viewport.rotz = 0.0f;
    viewport.tx = 0.0f;
    viewport.tz = 0.0f;

    glutInitWindowSize(viewport.w, viewport.h);
    glutInitWindowPosition(0, 0);
    glutCreateWindow(TITLE);

    init_scene();
    SimpleTorus *st = new SimpleTorus(2.0f, 10);
    drawer = new SimpleTorusDrawer(st);
    energy = new SimpleTorusEnergy(st);

    glutDisplayFunc (draw_scene     );
    glutReshapeFunc (reshape_window );
    glutIdleFunc    (move_frame     );
    glutKeyboardFunc(handle_keyboard);
    glutSpecialFunc (handle_special );
    glutMainLoop();

    return 0;
}
