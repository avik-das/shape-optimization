#include "main.h"
#include "energy.hpp"
#include <algorithm>

using namespace std;

#include <boost/filesystem.hpp>

using namespace boost::filesystem;

//****************************************************
// Some Classes
//****************************************************
class Viewport {
public:
    Viewport(): mousePos(0.0,0.0) { orientation = identity3D(); };
	int w, h; // width and height
	vec2 mousePos;
    mat4 orientation;
};

//****************************************************
// Global Variables
//****************************************************
Viewport viewport;
UCB::ImageSaver * imgSaver;
int frameCount = 0;
enum {VIEW_FIRSTPERSON, VIEW_THIRDPERSON, VIEW_MAX};
int viewMode = VIEW_THIRDPERSON;
float zoom = 0.0f;

TwBar* tbar = NULL;

vector<string> tracks;
int currTrack = 0;

Energy* energy = NULL;
double twist_weight = 0.5;
bool running = false;
bool paused  = true ;

double speed = 1.0;
bool smooth = false;

//****************************************************
// Potential models
//****************************************************
SplineCoaster *coaster = NULL;
KBMTorus *kbmtorus = NULL;

//****************************************************
// Forward Declarations (not exhaustive)
//****************************************************
void setup_anttweakbar();
void setup_anttweakbar_tracks();
void find_tracks();
void load_curr_track();

// A simple helper function to load a mat4 into opengl
void applyMat4(mat4 &m) {
	double glmat[16];
	int idx = 0;
	for (int j = 0; j < 4; j++) 
		for (int i = 0; i < 4; i++)
			glmat[idx++] = m[i][j];
	glMultMatrixd(glmat);
}

// replace this with something cooler!
void drawCart(double size) {
    glutSolidTeapot(size);
}


//-------------------------------------------------------------------------------
/// You will be calling all of your drawing-related code from this function.
/// Nowhere else in your code should you use glBegin(...) and glEnd() except code
/// called from this method.
///
/// To force a redraw of the screen (eg. after mouse events or the like) simply call
/// glutPostRedisplay();
void display() {

	//Clear Buffers
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);					// indicate we are specifying camera transformations
	glLoadIdentity();							// make sure transformation is "zero'd"


    if (viewMode == VIEW_THIRDPERSON) {
        glTranslatef(0,-2,-35 - zoom);
        applyMat4(viewport.orientation);
    }

    glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(10.0, 0.0, 0.0);

        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 10.0, 0.0);
        
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 10.0);
    glEnd();

    int samples_per_pt = smooth ? 100 : 1;
    if (coaster) {
        coaster->renderWithDisplayList(samples_per_pt);
    }
    else if (kbmtorus) {
        kbmtorus->renderWithDisplayList(samples_per_pt);
    }

    //Draw the AntTweakBar
    TwDraw();

	//Now that we've drawn on the buffer, swap the drawing buffer and the displaying buffer.
	glutSwapBuffers();

}


//-------------------------------------------------------------------------------
/// \brief	Called when the screen gets resized.
/// This gives you the opportunity to set up all the relevant transforms.
///
void reshape(int w, int h) {
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, ((double)w / MAX(h, 1)), 1.0, 100.0);
	//glOrtho(-10,10,-10,10,1,100);

    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

    //Send the new window size to AntTweakBar
    TwWindowSize(w, h);
}

void myIdleFunc() {
	if (running && !paused) {
		running = !energy->iterate();
		glutPostRedisplay();
	}
}

//-------------------------------------------------------------------------------
/// Called to handle keyboard events.
void myKeyboardFunc (unsigned char key, int x, int y) {
    if (TwEventKeyboardGLUT(key, x, y))
        return;

	switch (key) {
		case 27:			// Escape key
			exit(0);
			break;
        case 'S':
        case 's':
    	    imgSaver->saveFrame();
            break;
        case 'V':
        case 'v':
            viewMode = (viewMode+1)%VIEW_MAX;
            break;
        case '+':
             if (coaster)
                 coaster->incrGlobalTwist(10.0);
             break;
		case '=':
             if (coaster)
                 coaster->incrGlobalTwist(10.0);
			 break;
        case '-':
             if (coaster)
                 coaster->incrGlobalTwist(-10.0);
             break;
        case 'U':
        case 'u':
             if (kbmtorus)
                 kbmtorus->toggleUpVectors();
             break;
        case 'D':
        case 'd':
             if (kbmtorus)
                 kbmtorus->dumpPoints();
             if (coaster)
                 coaster->dumpPoints();
             break;
	}
}

//-------------------------------------------------------------------------------
/// Called whenever the mouse moves while a button is pressed
void myActiveMotionFunc(int x, int y) {
    if (TwEventMouseMotionGLUT(x, y)) {
        glutPostRedisplay();
        return;
    }

    // Rotate viewport orientation proportional to mouse motion
    vec2 newMouse = vec2((double)x / glutGet(GLUT_WINDOW_WIDTH),(double)y / glutGet(GLUT_WINDOW_HEIGHT));
    vec2 diff = (newMouse - viewport.mousePos);
    double len = diff.length();
    if (len > .001) {
        vec3 axis = vec3(diff[1]/len, diff[0]/len, 0);
        viewport.orientation = rotation3D(axis, 180 * len) * viewport.orientation;
    }

    //Record the mouse location for drawing crosshairs
    viewport.mousePos = newMouse;

    //Force a redraw of the window.
    glutPostRedisplay();
}


//-------------------------------------------------------------------------------
/// Called whenever the mouse moves without any buttons pressed.
void myPassiveMotionFunc(int x, int y) {
    if (TwEventMouseMotionGLUT(x, y))
        return;
    
    //Record the mouse location for drawing crosshairs
    viewport.mousePos = vec2((double)x / glutGet(GLUT_WINDOW_WIDTH),(double)y / glutGet(GLUT_WINDOW_HEIGHT));

    //Force a redraw of the window.
    glutPostRedisplay();
}

//-------------------------------------------------------------------------------
/// Called whenever a mouse button is pressed
void myMouseFunc(int button, int state, int x, int y) {
    if (TwEventMouseButtonGLUT(button, state, x, y))
        return;

    // mouse wheel event
    if (button == 3 || button == 4) {
        if (state == GLUT_UP)
            return; // disregard redundant GLUT_UP events

        if (button == 3)
            zoom += 0.1f;
        else
            zoom -= 0.1f;

        glutPostRedisplay();
    }
}

//-------------------------------------------------------------------------------
/// Called to update the screen at 30 fps.
void frameTimer(int value) {
    frameCount++;
    glutPostRedisplay();
    glutTimerFunc(1000/30, frameTimer, 1);
}


//-------------------------------------------------------------------------------
/// Called right before the application exits
void app_terminate() {
    TwTerminate();
}

//****************************************************
// AntTweakBar configuration
//****************************************************

void TW_CALL atb_run_optimization (void*) {
    cout << "Running Optimization" << endl;

    load_curr_track();
    running = true ;
    paused  = false;

    TwDefine("'Optimization Parameters'/pause_opt_button visible=true ");
}

void TW_CALL atb_pause_optimization (void*) {
    if (!running) { return; }

    paused = !paused;

    if (paused) {
        TwSetParam(tbar, "pause_opt_button", "label", TW_PARAM_CSTRING, 1,
            "CONTINUE");
    }
    else {
        TwSetParam(tbar, "pause_opt_button", "label", TW_PARAM_CSTRING, 1,
            "PAUSE");
    }
}

void TW_CALL atb_set_track(const void *value, void *clientData) { 
    currTrack = *(const int *)value;
    load_curr_track();
    glutPostRedisplay();
}

void TW_CALL atb_get_track(void *value, void *clientData) { 
    *(int *)value = currTrack;
}

void TW_CALL atb_set_speed(const void *value, void *clientData) { 
    speed = *(const double *)value;
    if (energy) { energy->set_speed(speed); }
}

void TW_CALL atb_get_speed(void *value, void *clientData) { 
    *(double *)value = speed;
}

void TW_CALL atb_set_smooth(const void *value, void *clientData) { 
    smooth = *(const bool *)value;
    if (coaster) { coaster->clearDisplayList(); }
    if (kbmtorus) { kbmtorus->clearDisplayList(); }
    glutPostRedisplay();
}

void TW_CALL atb_get_smooth(void *value, void *clientData) { 
    *(bool *)value = smooth;
}

//-------------------------------------------------------------------------------
/// Initialize AntTweakBar and set up all the configuration items.
void setup_anttweakbar() {
    TwInit(TW_OPENGL, NULL);
    // Send 'glutGetModifers' function pointer to AntTweakBar; required because
    // the GLUT key event functions do not report key modifiers states.
    TwGLUTModifiersFunc(glutGetModifiers);

    tbar = TwNewBar("Optimization Parameters");

    TwDefine(" 'Optimization Parameters' size='500 150' "
                                    "position=' 50  25' "
                                 "valueswidth=300       ");

    TwAddVarRW(tbar, "twist_weight_field", TW_TYPE_DOUBLE, &twist_weight,
        "label='Twist Penalty Weight' "
        "help='What percentage of the total penalty is derived from the twist "
            "penalty' "
        "min=0 max=1 step=0.001 precision=10");

    TwAddVarCB(tbar, "opt_speed", TW_TYPE_DOUBLE,
        atb_set_speed, atb_get_speed, NULL,
        "label='Optimization Speed' "
        "help='How fast the optimization runs' "
        "min=0.3 max=2 step=0.1 precision=1");

    setup_anttweakbar_tracks();

    TwAddButton(tbar, "run_opt_button", atb_run_optimization, NULL,
        "label='RUN' "
        "help='Restart the optimization with the configured parameters'");
    
    TwAddButton(tbar, "pause_opt_button", atb_pause_optimization, NULL,
        "label='PAUSE' "
        "help='Pause or continue the current optimization'");

    TwAddSeparator(tbar, "render_sep", NULL);

    TwAddVarCB(tbar, "smooth", TW_TYPE_BOOLCPP,
        atb_set_smooth, atb_get_smooth, NULL,
        "label='Render Smoothly' "
        "help='Whether the curve is rendered smoothly, or simply as a polygon "
            "with the control points as the vertices' ");
}

void setup_anttweakbar_tracks() {
    tracks.clear();
    find_tracks();

    TwEnumVal *trackEV =
        (TwEnumVal*) malloc(tracks.size() * sizeof(TwEnumVal));

    int ti = 0;
    for (vector<string>::iterator it = tracks.begin();
         it < tracks.end(); it++) {
        trackEV[ti] = (TwEnumVal) {ti, it->c_str()};
        ti++;
    }
    
    TwType trackType = TwDefineEnum("TrackType", trackEV, tracks.size());

    TwAddVarCB(tbar, "Track", trackType, atb_set_track, atb_get_track, NULL,
        "label='Track' "
        "help='Change the track that will be optimized.' ");
    
}

//-------------------------------------------------------------------------------
/// Find all the tracks in the current directory
void find_tracks() {
    path dir(".");
    if (!exists(dir)) { return; }

    directory_iterator end_itr;
    for (directory_iterator itr(dir); itr != end_itr; itr++) {
        if (is_directory(itr->status())) { continue; }

        string name = itr->path().filename().string();

        if (name.rfind(".trk") == name.size() - 4)
            tracks.push_back (name);
    }

    sort(tracks.begin(), tracks.end());
}

//-------------------------------------------------------------------------------
/// Set the current track index based on the user-given track name
void find_curr_track(char* track) {
    path trackpath(track);

    int i = 0;
    for (vector<string>::iterator ti = tracks.begin();
         ti < tracks.end(); ti++) {
        path tipath(*ti);
        if (equivalent(tipath, trackpath)) {
            currTrack = i;
            break;
        }
        i++;
    }
}

//-------------------------------------------------------------------------------
/// Changes the current track based on the index in currTrack. Stops the
/// current optimization, then deletes the current track and the current
/// iterator. Loads the new track and the new iterator.
void load_curr_track() {
    running = false;

    TwDefine("'Optimization Parameters'/pause_opt_button visible=false ");

    if (coaster ) { delete coaster ; }
    if (kbmtorus) { delete kbmtorus; }
    if (energy  ) { delete energy  ; }

    // FIXME: check filename, then load correct model based on that
    //coaster = new SplineCoaster(tracks[currTrack].c_str());
    //if (coaster->bad()) {
    //    cout << "Coaster file appears to not have a proper coaster in it"
    //         << endl;
    //    return;
    //}
	//energy = new LineEnergy(coaster, twist_weight);

    kbmtorus = new KBMTorus();
    energy = new KBMEnergy(kbmtorus, twist_weight);

    energy->set_speed(speed);
}

//-------------------------------------------------------------------------------
/// Initialize the environment
int main(int argc,char** argv) {
	//Initialize OpenGL
	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA|GLUT_DEPTH);

	//Set up global variables
	viewport.w = 600;
	viewport.h = 600;

	//Initialize the screen capture class to save BMP captures
	//in the current directory, with the prefix "coaster"
	imgSaver = new UCB::ImageSaver("./", "coaster");

	//Create OpenGL Window
	glutInitWindowSize(viewport.w,viewport.h);
	glutInitWindowPosition(0,0);
	glutCreateWindow("CS184 Framework");

	//Register event handlers with OpenGL.
	glutIdleFunc(myIdleFunc);

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(myKeyboardFunc);
	glutMotionFunc(myActiveMotionFunc);
	glutPassiveMotionFunc(myPassiveMotionFunc);
    glutMouseFunc(myMouseFunc);
    frameTimer(0);

    atexit(app_terminate);

    //Any event not handled by our code can be handled directly by AntTweakBar
    glutSpecialFunc((GLUTspecialfun)TwEventSpecialGLUT);

    glClearColor(0,0,0,0);

    // set some lights
    {
       float ambient[3] = { .5f, .5f, .5f };
       float diffuse[3] = { .5f, .5f, .5f };
       float pos[4] = { 0, 5, -5, 0 };
       glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT1, GL_POSITION, pos);
       glEnable(GL_LIGHT1);
    }
    {
       float ambient[3] = { .1f, .1f, .1f };
       float diffuse[3] = { .5f, .2f, .5f };
       float pos[4] = { 5, 0, -5, 0 };
       glLightfv(GL_LIGHT2, GL_AMBIENT, ambient);
       glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse);
       glLightfv(GL_LIGHT2, GL_POSITION, pos);
       glEnable(GL_LIGHT2);
    }
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

    // set up the AntTweakBar
    setup_anttweakbar();
    if (argc > 1) { find_curr_track(argv[1]); }
    load_curr_track();


    // make sure the initial window shape is set
    reshape(viewport.w, viewport.h);


	//And Go!
	glutMainLoop();
}
