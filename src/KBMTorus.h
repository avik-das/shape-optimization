/*
 * KBMTorus.h
 *
 *  Created on: August 29, 2011
 *      Author: Avik Das
 */

#ifndef KBMTORUS_H_
#define KBMTORUS_H_


#include <cmath>
#include <vector>

#include "algebra3.h"
#include "main.h"

#include "SplineCoaster.h"

/*
 * Based on a SLIDE file by Carlo Sequin.
 *
 * Combine two sweeps and two half Clifford tori, forming klein-bottle mouths.
 * Enforce point symmetry through origin. Use two copies to form a toroidal
 * loop to make a useful half-way point for torus eversion.
 */
class KBMTorus {
public:
    /* TODO: Add parameters? */
    KBMTorus();
    ~KBMTorus();

    // renders the KBM torus
    void render(int samplesPerPt);

    // renders the KBM torus with a cache.  Ignores parameters after display
    // list is set; use clearDisplayList() before updating parameters
    void renderWithDisplayList(int samplesPerPt) {
        if (!hasDL) {
            DLid = glGenLists(1);
            glNewList(DLid, GL_COMPILE);
            render(samplesPerPt);
            glEndList();
            hasDL = true;
        }
        glCallList(DLid);
    }

    void clearDisplayList() {
        if (hasDL) {
            glDeleteLists(DLid, 1);
            hasDL = false;
        }
    }

	/*Returns number of parameters **ENERGY-RELATED** */
    enum ArmType {LEFTARM, RGHTARM};

	int getNumControlPoints();
	int getNumControlPoints(ArmType whicharm);

	int getNumMovableControlPoints();
	int getNumMovableControlPoints(ArmType whicharm);

    void compensateTwist();
    double getGlobalTwist(ArmType whicharm);

	void changePoint(ArmType whicharm, int index,
        double dx, double dy, double dz, double dcss, double drot);
	SplinePoint getPoint(ArmType whicharm, int index);

    void toggleUpVectors();
    void dumpPoints();

    double getTorTilt();
    void changeTorTilt(double dtilt);
    void changeTorposX(double dtorposx);
    void changeTorposY(double dtorposy);
    void changeTorposZ(double dtorposz);

private:
    /* High-level parameters */
    float torposx;
    float torposy;
    float torposz;
    float tortilt;

    float looprad ;
    float looptilt;

    // display list for caching sweep geometry
    GLuint DLid;
    bool hasDL;

    SplineCoaster *leftarm;
    SplineCoaster *rghtarm;

    void move_left_end_segments();
    void move_rght_end_segments();

    void create_leftarm();
    void create_rghtarm();
};

#endif

