#include "SplineCoaster.h"

#include "global.h"

#include <fstream>

#define EPSILON (.00000000001)
#define FULL_ROTATION (360.0)

#define TWIST_JUMP (20.0)

namespace {
    // Used to advance the rotation minimizing frame forward
    // formula from [wang et al. 07]
    inline vec3 advanceFrame(const vec3 &xi, const vec3 &xi1,
                                const vec3 &ti, const vec3 &si, const vec3 &ri,
                                  const vec3 &ti1) {
        vec3 v1 = xi1 - xi;
        double c1 = v1 * v1;
        if (c1 == 0)
            return ri;
        vec3 riL = ri - (2/c1)*(v1*ri)*v1;
        vec3 tiL = ti - (2/c1)*(v1*ti)*v1;
        vec3 v2 = ti1 - tiL;
        double c2 = v2 * v2;
        if (c2 == 0)
            return ri;
        return riL - (2/c2)*(v2*riL)*v2;
    }
}


SplinePoint SplinePoint::sampleBSpline(vector<SplinePoint*>& cps, double t, bool closed, int degree)
{
    if (cps.empty()) {
        UCBPrint("sampleBSpline", "need at least one control point");
        return SplinePoint();
    }

    // get into 0,1 range
    if (t > 1.0 || t < 0.0)
        t = fmod(t, 1.0);
    if (t < 0.0)
        t += 1.0;
    
    // adjust degree down as needed to get a curve, if too few control points and not on closed loop
    int numCPs = int(closed ? cps.size() + degree : cps.size());
    if (degree >= numCPs)
        degree = numCPs - 1;

    // rescale t to minSupport,maxSupport range
    double minSupport = degree;
    double maxSupport = numCPs;
    t = (1-t)*minSupport + t*maxSupport;

    // 'recursive' form of b-spline is done iteratively here
    SplinePoint *bases = new SplinePoint[degree+1];
    int k = (int)t;

    for (int i = 0; i <= degree; ++i) {
        bases[i] = *cps[(k - degree + i) % cps.size()];
    }

    for (int power = 1; power <= degree; ++power) {
        for (int i = 0; i <= degree - power; ++i) {
            int knot = k - degree + power + i;
            double u_i = (double)knot;
            double u_ipr1 = double(knot + degree - power + 1);
            double a = (t - u_i) / (u_ipr1 - u_i);
            bases[i] = SplinePoint::lerp(a, bases[i], bases[i+1]);
        }
    }

    SplinePoint result = bases[0];

    delete [] bases;

    return result;
}

SplineCoaster::SplineCoaster(string filename) : globalTwist(0), initialGlobalTwist(0), globalAzimuth(0), hasDL(false) {
    ifstream f(filename.c_str());
    if (!f) {
        UCBPrint("SplineCoaster", "Couldn't load file " << filename);
        return;
    }
    string line;
    while (getline(f,line)) {
        stringstream linestream(line);
        string op;
        linestream >> op;
        if (op.empty() || op[0] == '#') // comments are marked by # at the start of a line
            continue;
        if (op == "p") { // p marks profile points (2d cross section vertices)
            vec2 v;
            linestream >> v;
            profile.push_back(v);
        } else if (op == "v") { // v marks bspline control points with optional azimuth info
            vec3 v(0.0);
            linestream >> v;
            double az;
            if (linestream >> az) {
                bsplinePts.push_back(new SplinePoint(v, az));
            } else {
                bsplinePts.push_back(new SplinePoint(v));
            }
        } else if (op == "twist") {
            linestream >> globalTwist;
        } else if (op == "azimuth") {
            linestream >> globalAzimuth;
        }
    }
    initialGlobalTwist = globalTwist;
    normalizeStruts();
    compensateTwist();
}

// clean up memory (helper for the big render function)
void SplineCoaster::freePolyline(vector<SplinePoint*> &pts) {
    for(vector<SplinePoint*>::iterator it = pts.begin(); it != pts.end(); ++it)
        delete *it;
}

// create a polyline that samples the curve (helper for the big render function)
void SplineCoaster::createPolyline(vector<SplinePoint*> &polyline, int totalSamples) {
    if (totalSamples == 0)
        return; // ... no samples is easy!

    vec3 lastGood(0.0);
    for (int i = 0; i < totalSamples + 3; i++) {
        int loc = i % totalSamples;
        double t = loc / double(totalSamples);
        SplinePoint sp = SplinePoint::sampleBSpline(bsplinePts, t);
        if (!polyline.empty() && (sp.point - lastGood).length2() < .0001) {
            continue; // wait for the samples to get a bit further apart ... !
        } else {
            polyline.push_back(new SplinePoint(sp));
            lastGood = sp.point;
        }
    }
}

// render supporting pillars for the coaster (helper for the big render function)
void SplineCoaster::renderSupports(int supportsPerPt, double supportSize, double groundY) {
    int totalSupports = (int) bsplinePts.size() * supportsPerPt;
    glColor3f(0,0,1); // blue supports ...
    for (int i = 0; i < totalSupports; i++) {
        int loc = i % totalSupports;
        double t = loc / double(totalSupports);
        SplinePoint sp = sample(t);
        if (sp.point[1] < groundY) continue; // don't add supports if we're underground
        vec3 upCurve = sampleUp(t, .01);
        if (upCurve * vec3(0,1,0) < .3) continue; // don't add supports if we're upside-down
        vec3 pos = sp.point;
        vec3 right(1,0,0);
        vec3 fwd(0,0,1);
        glBegin(GL_QUAD_STRIP);
        for (double circ = 0; circ <= 2 * M_PI + .05; circ += .1) {
            vec3 n = right * cos(circ) + fwd * sin(circ);
            vec3 p = pos + n * supportSize;
            glNormal3f(n[0],n[1],n[2]);
            glVertex3d(p[0],p[1]-1,p[2]);
            glVertex3d(p[0],groundY,p[2]);
        }
        glEnd();
    }
}

// sample the curve at a point
SplinePoint SplineCoaster::sample(double t) {
    return SplinePoint::sampleBSpline(bsplinePts, t);
}
// get the forward direction
vec3 SplineCoaster::sampleForward(double t, double step) {
    return sample(t+step).point - sample(t).point;
}
// search for a non-zero forward direction
vec3 SplineCoaster::sampleForward_nonzero(double t, double step, int searchdist) {
    int k = 1;
    vec3 dir;
    do {
        dir = sampleForward(t,step*k);
    } while (dir.length2() < EPSILON && k++ < searchdist);
    return dir;
}
vec3 SplineCoaster::sampleUp(double t, double step) {
        t = fmod(t, 1.0);
        if (t < 0.0) t+=1.0;
        vec3 up = getFirstUp();
        vec3 lastpos = sample(0).point;
        vec3 lastdir = sampleForward_nonzero(0);
        lastdir.normalize();
        vec3 dir = lastdir;
        vec3 pos = lastpos;
        for (double st = step; st <= t + step*.5; st+=step) {
            double tt = min(st, t);
            dir = sampleForward(tt);
            if (dir.length2() < EPSILON)
                continue;
            dir.normalize();
            pos = sample(tt).point;
            vec3 right = lastdir ^ up;
            right.normalize();
            up = advanceFrame(lastpos, pos, lastdir, right, up, dir);
            right = dir ^ up;
            up = right ^ dir;
            up.normalize();
            lastpos = pos; lastdir = dir;
        }

        
        // orthonormalize the frame
        dir = sampleForward(t);
        if (dir.length2() < EPSILON)
            dir = lastdir;
        dir.normalize();
        vec3 right = dir^up;
        up = right^dir;
        up.normalize();
        
        orientVectorInFrame(-dir, fmod(t,1.0), sample(t).azimuth, up);

        
        return up;
    }


// rotates a vector according to the global azimuth, local azimuth, twist, direction, and location on curve
void SplineCoaster::orientVectorInFrame(const vec3 &dir, double percent, double localAz, vec3 &inFrame) {
    double rot = globalAzimuth + globalTwist * percent + localAz;
    inFrame = rotation3D(dir, rot) * inFrame;
}

vec3 SplineCoaster::getFirstUp() {
    vec3 leg1 = sampleForward_nonzero(0).normalize();
    vec3 leg2 = sampleForward_nonzero(0,-.001).normalize();
    vec3 up = leg1+leg2; // start with the frenet frame
    if (up.length2() < .0001) { // if that doesn't work, try something else
        up = leg1 ^ vec3(0,1,0);
        if (up.length2() < .0001) {
            up = leg1 ^ vec3(.1,1,0);
        }
    }
    up.normalize();
    return up;
}



// sweep the cross section along the curve (helper for the big render function)
void SplineCoaster::renderSweep(vector<SplinePoint*> &polyline, double crossSectionScale) {
    SplinePoint* pts[3]; // pts[1] is us, pts[0] and pts[3] surround us
    vector<vec2> & crossSection = profile;
    int size = (int) polyline.size();
    vec3 * newSlice = new vec3[crossSection.size()];
    vec3 * oldSlice = new vec3[crossSection.size()];
    vec3 oldDir(0.0), right(0.0);
    vec3 up(0.0);
    bool firstDir = true;
    for (int i = 1; i < size-1; i++) {
        double percent = double(i % size) / (double(size-3));
        for (int c = -1; c <= 1; c++) { // populate local pts
            pts[c+1] = polyline[ (i + size + c) %  size ];
        }

		vec3 leg1 = (pts[0]->point - pts[1]->point).normalize();
        vec3 leg2 = (pts[2]->point - pts[1]->point).normalize();
        vec3 dir = (leg2 - leg1);
        if (dir.length2() < .0001)
            dir = pts[2]->point - pts[1]->point;
		dir.normalize();
        
        if (firstDir) { // first time around use a special routine to find the up dir
            up = getFirstUp();
            firstDir = false;
        }
        else { // after the first frame, advance with the rotation minimizing frame
            up = advanceFrame(pts[0]->point, pts[1]->point,
                        oldDir, right, up, dir);
        }
        right = dir ^ up;
        up = right ^ dir;
        up.normalize(); right.normalize();
        oldDir = dir;

        double rot = globalAzimuth + globalTwist * percent + pts[1]->azimuth;

        vec3 bisect = leg1 + leg2;
        double len = bisect.length();
        bool scaleSect = false;
        double scaleTrans = 0;
        if (len > .0001) { // only scale if not going straight already
            scaleSect = true;
            bisect = bisect/len;
            double dot = -leg1*leg2;
            double angle = acos(CLAMP(dot,-1.0,1.0));
            double scale = 1.0 / MAX(cos(.5*angle ),.1);
            scaleTrans = scale - 1.0;
		}

        double s = crossSectionScale;
        int ind = 0;
        for (vector<vec2>::iterator it = crossSection.begin(); it != crossSection.end(); ++it, ++ind) {
            vec2 pos2d = rotation2D(vec2(0,0),rot) * (*it);
            vec3 pt = right * pos2d[0] * s + up * pos2d[1] * s;
            if (scaleSect) {
                pt = pt + scaleTrans * (pt * bisect) * bisect;
            }
            newSlice[ind] = pts[1]->point + pt;
        }

        if (i > 1) {
            glBegin(GL_QUADS);
            for (int v = 0; v < (int) crossSection.size(); v++) {
                int vn = (v + 1) % int(crossSection.size());
                glColor3f(v%4!=0,v%4!=1,v%4!=2);
				vec3 n = (newSlice[v] - oldSlice[v])^(newSlice[vn] - oldSlice[v]);
				n.normalize();
				glNormal3dv(&n[0]);
                glVertex3dv(&oldSlice[v][0]);
                glVertex3dv(&newSlice[v][0]);
                glVertex3dv(&newSlice[vn][0]);
                glVertex3dv(&oldSlice[vn][0]);
            }
            glEnd();
        }

        // swap new and old lists
        vec3 *temp = newSlice;
        newSlice = oldSlice;
        oldSlice = temp;
    }
    delete [] newSlice;
    delete [] oldSlice;
}

// the big render function
void SplineCoaster::render(int samplesPerPt, double crossSectionScale, int supportsPerPt, double supportSize, double groundY) {
    int totalSamples = (int) bsplinePts.size() * samplesPerPt;

    vector<SplinePoint*> polyline;
    createPolyline(polyline, totalSamples);

    int size = (int) polyline.size();
    if (size < 2) { // a polyline with only one point is pretty lame!
        freePolyline(polyline);
        cout << "not enough curve to sweep ..." << endl;
        return;
    }

    //renderSupports(supportsPerPt, supportSize, groundY);

    renderSweep(polyline, crossSectionScale);

    freePolyline(polyline);
}


// increase the global twist 
void SplineCoaster::incrGlobalTwist(double dgt) {
    globalTwist += dgt;
    clearDisplayList();
}

// increase the global azimuthal rotation 
void SplineCoaster::incrGlobalAzimuth(double daz) {
    globalAzimuth += daz;
    clearDisplayList();
}

/* Returns number of control points **ENERGY-RELATED** */
int SplineCoaster::getNumControlPoints() {
	return bsplinePts.size();
}

void SplineCoaster::changePoint(int index, double dx, double dy, double dz) {
	SplinePoint* point = bsplinePts[index];

	point->point[0] += dx;
	point->point[1] += dy;
	point->point[2] += dz;

	clearDisplayList();
}

void SplineCoaster::normalizeStruts() {
    // First, calculate the actual average strut length
    int numpoints = getNumControlPoints();

    double avglen = 0.0;
    for (int pi = 0; pi < numpoints; pi++) {
        SplinePoint* p1 = bsplinePts[ pi      % numpoints];
        SplinePoint* p2 = bsplinePts[(pi + 1) % numpoints];
        avglen += (p1->point - p2->point).length();
    }
    avglen /= numpoints;

    double ratio = STRUT_REST_LENGTH / avglen;

    for (int pi = 0; pi < numpoints; pi++) {
        SplinePoint* p = bsplinePts[pi];
        p->point[0] *= ratio;
        p->point[1] *= ratio;
        p->point[2] *= ratio;
    }
}

void SplineCoaster::compensateTwist() {
    vec3 firstUp = sampleUp(0.0);
    vec3  lastUp = sampleUp(1.0-EPSILON);
    
    // Rotate the up vector so by the initial global twist, which accounts for
    // the fact that the two up vectors may be mismatched by that amount and
    // still be considered matched. This is important for twists that are not
    // multiples of 360 degrees, such as 180 degrees. In such a case, the ends
    // will only match up (though the colors will not) if the cross sections
    // are a specific shape, such as a rectangle.
    vec3 lastForward = sampleForward(1.0-EPSILON);
    lastUp = rotation3D(lastForward, initialGlobalTwist) * lastUp;

    double normdot = firstUp*lastUp / (firstUp.length()*lastUp.length());
    double angle = acos(CLAMP(normdot, -1.0, 1.0)) * 180 / acos(-1.0);

    // The first check to make is whether or not the correct angle is greater
    // than or less than 180 degrees. This is because the range of acos is 0 to
    // 180, not 0 to 360.
    // 
    // To perform this check, we check the cross product of the last up vector
    // with the first up vector against the forward vector at the starting
    // point. They should point in same direction if the actual mismatch is
    // within 180 degrees.
    vec3 compForward = lastUp ^ firstUp;
    vec3 realForward = sampleForward(1.0-EPSILON);
    if (compForward * realForward >= 0.0) { angle *= -1; }

    globalTwist += angle;
}

double SplineCoaster::getGlobalTwist() {
    return globalTwist;
}
