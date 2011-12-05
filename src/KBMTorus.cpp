#include "KBMTorus.h"

#include "global.h"

KBMTorus::KBMTorus() :
    torposy( 8.0), torposz(0.0),
    tortilt(-1.0),
    looprad( 5.0), looptilt(0.0),
    hasDL(false) {
    create_leftarm();
    create_rghtarm();
}

KBMTorus::~KBMTorus() {
    if (leftarm) { delete leftarm; }
    if (rghtarm) { delete rghtarm; }
}

void KBMTorus::render(int samplesPerPt, double crossSectionScale) {
    leftarm->render(samplesPerPt, crossSectionScale);
    rghtarm->render(samplesPerPt, crossSectionScale);
}

int KBMTorus::getNumControlPoints() {
    return getNumControlPoints(LEFTARM) +
           getNumControlPoints(RGHTARM);
}

int KBMTorus::getNumControlPoints(ArmType whicharm) {
    switch (whicharm) {
        case LEFTARM:
            return leftarm->getNumControlPoints();
        case RGHTARM:
            return rghtarm->getNumControlPoints();
    }
}

void KBMTorus::changePoint(ArmType whicharm, int index,
                           double dx, double dy, double dz) {
    switch (whicharm) {
        case LEFTARM:
            leftarm->changePoint(index, dx, dy, dz);
            break;
        case RGHTARM:
            rghtarm->changePoint(index, dx, dy, dz);
            break;
    }

	clearDisplayList();
}

SplinePoint KBMTorus::getPoint(ArmType whicharm, int index) {
    switch (whicharm) {
        case LEFTARM:
            return leftarm->getPoint(index);
        case RGHTARM:
            return rghtarm->getPoint(index);
    }
}

void KBMTorus::create_leftarm() {
    vector<vec2> profile;
    profile.push_back(vec2(-1.0,  0.0));
    profile.push_back(vec2( 0.0,  1.0));
    profile.push_back(vec2( 1.0,  0.0));
    profile.push_back(vec2( 0.0, -1.0));

    vec3 q0 = vec3(0, -torposy + sin(tortilt) * 2, -torposz - cos(tortilt) * 2);
    vec3 q1 = vec3(0, -torposy, -torposz);
    vec3 q2 = vec3(0, -torposy - sin(tortilt) * 2, -torposz + cos(tortilt) * 2);

    vec3 q3 = vec3(-0.3 * looprad,
                   -torposy - 3 * sin(tortilt),
                   -torposz + 3 * cos(tortilt));
    vec3 q4 = vec3(-looprad, -cos(looptilt) * 2, -sin(looptilt) * 2);
    vec3 q5 = vec3(-looprad,  cos(looptilt) * 2,  sin(looptilt) * 2);
    vec3 q6 = vec3(-0.3 * looprad,
                   torposy + 3 * sin(tortilt),
                   torposz - 3 * cos(tortilt));

    vec3 q7 = vec3(0, torposy + sin(tortilt) * 2, torposz - cos(tortilt) * 2);
    vec3 q8 = vec3(0, torposy, torposz);
    vec3 q9 = vec3(0, torposy - sin(tortilt) * 2, torposz + cos(tortilt) * 2);

    vector<vec3> pts;
    pts.push_back(q0); pts.push_back(q1); pts.push_back(q2);
    pts.push_back(q3); pts.push_back(q4); pts.push_back(q5); pts.push_back(q6);
    pts.push_back(q7); pts.push_back(q8); pts.push_back(q9);

    leftarm = new SplineCoaster(pts, profile, 0.0, 0.0);
    leftarm->setClosed(false);
}

void KBMTorus::create_rghtarm() {
    vector<vec2> profile;
    profile.push_back(vec2(-1.0,  0.0));
    profile.push_back(vec2( 0.0,  1.0));
    profile.push_back(vec2( 1.0,  0.0));
    profile.push_back(vec2( 0.0, -1.0));

    vec3 q0 = vec3(0, torposy - sin(tortilt) * 2, torposz + cos(tortilt) * 2);
    vec3 q1 = vec3(0, torposy, torposz);
    vec3 q2 = vec3(0, torposy + sin(tortilt) * 2, torposz - cos(tortilt) * 2);

    vec3 q3 = vec3(0.3 * looprad,
                   torposy + 3 * sin(tortilt),
                   torposz - 3 * cos(tortilt));
    vec3 q4 = vec3(looprad,  cos(looptilt) * 2,  sin(looptilt) * 2);
    vec3 q5 = vec3(looprad, -cos(looptilt) * 2, -sin(looptilt) * 2);
    vec3 q6 = vec3(0.3 * looprad,
                   -torposy - 3 * sin(tortilt),
                   -torposz + 3 * cos(tortilt));

    vec3 q7 = vec3(0, -torposy - sin(tortilt) * 2, -torposz + cos(tortilt) * 2);
    vec3 q8 = vec3(0, -torposy, -torposz);
    vec3 q9 = vec3(0, -torposy + sin(tortilt) * 2, -torposz - cos(tortilt) * 2);

    vector<vec3> pts;
    pts.push_back(q0); pts.push_back(q1); pts.push_back(q2);
    pts.push_back(q3); pts.push_back(q4); pts.push_back(q5); pts.push_back(q6);
    pts.push_back(q7); pts.push_back(q8); pts.push_back(q9);

    rghtarm = new SplineCoaster(pts, profile, 0.0, 0.0);
    rghtarm->setClosed(false);
}
