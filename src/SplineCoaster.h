/*
 * SplineCoaster.h
 *
 *  Created on: March 19, 2009
 *      Author: jima (referencing jfhamlin) 
 */

#ifndef SPLINECOASTER_H_
#define SPLINECOASTER_H_


#include <cmath>
#include <vector>

#include "algebra3.h"
#include "main.h"

using namespace std;

// control point for the sweep defining the track
struct SplinePoint {
    vec3 point;
    double azimuth;

    SplinePoint() {}
    SplinePoint(vec3 pt, double az = 0) : point(pt), azimuth(az) {}

    static SplinePoint lerp(double t, SplinePoint &a, SplinePoint &b) {
        return SplinePoint( (1-t)*a.point + t*b.point, (1-t)*a.azimuth + t*b.azimuth );
    }

    static SplinePoint sampleBSpline(vector<SplinePoint*>& cps, double t, bool closed = true, int degree = 3);
};


// class to render and sample track
class SplineCoaster {
public:
    SplineCoaster(string filename);

    // renders the coaster
    void render(int samplesPerPt, double crossSectionScale=.2, int supportsPerPt=3, double supportSize=.1, double groundY=0.0);
    // renders the coaster with a cache.  Ignores parameters after display list is set; use clearDisplayList() before updating parameters
    void renderWithDisplayList(int samplesPerPt, double crossSectionScale=.2, int supportsPerPt=3, double supportSize=.1, double groundY=0.0) {
        if (!hasDL) {
            DLid = glGenLists(1);
            glNewList(DLid, GL_COMPILE);
            render(samplesPerPt, crossSectionScale, supportsPerPt, supportSize, groundY);
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

    // --- these functions can provide a local frame along the track
    // sample the curve at a point
    SplinePoint sample(double t);
    // get the forward direction
    vec3 sampleForward(double t, double step = .001);
    // get the forward direction, with some additional search if the first approximation turns out to be zero
    vec3 sampleForward_nonzero(double t, double step = .01, int searchdist = 50);
    // get the up direction
    vec3 sampleUp(double t, double step = .01);

    // use this to check for an invalid (empty) coaster
    bool bad() {
        return (bsplinePts.size() == 0);
    }

    // --- these functions affect the value of the parameters of the track
    // increase the global twist 
    void incrGlobalTwist(double dgt);
    // increase the global azimuthal rotation 
    void incrGlobalAzimuth(double daz);

private:
    vector<SplinePoint*> bsplinePts; // control points for the bspline
    double globalTwist; // twists the whole frame
    double globalAzimuth; // global azimuth rotates the whole frame
    vector<vec2> profile; // the cross section to be swept

    // display list for caching sweep geometry
    GLuint DLid;
    bool hasDL;

    // --- internal helper functions
    void renderSupports(int supportsPerPt, double supportSize, double groundY);
    void renderSweep(vector<SplinePoint*> &pts, double crossSectionScale);
    void createPolyline(vector<SplinePoint*> &pts, int totalSamples); // sample bspline
    void freePolyline(vector<SplinePoint*> &pts); // cleanup samples
    vec3 getFirstUp(); // helper to get initial frame (default to frenet, fallback to 'up=+Y')
    // rotates vector from RMF to account for twist, azimuth, etc:
    void orientVectorInFrame(const vec3 &dir, double percent, double localAz, vec3 &inFrame);

};


#endif

