#include "KBMTorus.h"

#include "global.h"

#include <boost/math/constants/constants.hpp>
const double PI = boost::math::constants::pi<double>();

KBMTorus::KBMTorus() :
    torposx(0.0), torposy( 8.0), torposz(0.0),
    tortilt(-PI/2),
    looprad( 5.0), looptilt(0.0),
    hasDL(false) {
    create_leftarm();
    create_rghtarm();
}

KBMTorus::~KBMTorus() {
    if (leftarm) { delete leftarm; }
    if (rghtarm) { delete rghtarm; }
}

void KBMTorus::render(int samplesPerPt) {
    leftarm->render(samplesPerPt);
    rghtarm->render(samplesPerPt);
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
        default     :
            return rghtarm->getNumControlPoints();
    }
}

int KBMTorus::getNumMovableControlPoints() {
    return getNumMovableControlPoints(LEFTARM) +
           getNumMovableControlPoints(RGHTARM);
}

int KBMTorus::getNumMovableControlPoints(ArmType whicharm) {
    // subtract 3 control points for each of the two mouths, which for now will
    // not move.
    switch (whicharm) {
        case LEFTARM:
            return leftarm->getNumControlPoints() - 6;
        case RGHTARM:
        default     :
            return rghtarm->getNumControlPoints() - 6;
    }
}

void KBMTorus::compensateTwist() {
    leftarm->compensateTwist();
    rghtarm->setStartFrameRotation(leftarm->getGlobalTwist());
    rghtarm->compensateTwist();
}

double KBMTorus::getGlobalTwist(ArmType whicharm) {
    switch (whicharm) {
        case LEFTARM:
            return leftarm->getGlobalTwist();
        case RGHTARM:
        default     :
            return rghtarm->getGlobalTwist();
    }
}

void KBMTorus::changePoint(ArmType whicharm, int index,
                           double dx, double dy, double dz,
                           double dcss, double drot) {
    switch (whicharm) {
        case LEFTARM:
            leftarm->changePoint(index, dx, dy, dz, dcss, drot);
            break;
        case RGHTARM:
            rghtarm->changePoint(index, dx, dy, dz, dcss, drot);
            break;
    }

	clearDisplayList();
}

SplinePoint KBMTorus::getPoint(ArmType whicharm, int index) {
    switch (whicharm) {
        case LEFTARM:
            return leftarm->getPoint(index);
        case RGHTARM:
        default     :
            return rghtarm->getPoint(index);
    }
}

void KBMTorus::toggleUpVectors() {
    leftarm->toggleUpVectors();
    rghtarm->toggleUpVectors();
	clearDisplayList();
}

void KBMTorus::dumpPoints() {
    cout << "Left Arm:" << endl;
    leftarm->dumpPoints();
    cout << "Right Arm:" << endl;
    rghtarm->dumpPoints();
}

double KBMTorus::getTorTilt() {
    return tortilt;
}

void KBMTorus::changeTorTilt(double dtilt) {
    tortilt += dtilt;
    move_left_end_segments();
    move_rght_end_segments();
}

void KBMTorus::changeTorposX(double dtorposx) {
    torposx += dtorposx;
    move_left_end_segments();
    move_rght_end_segments();
}

void KBMTorus::changeTorposY(double dtorposy) {
    torposy += dtorposy;
    move_left_end_segments();
    move_rght_end_segments();
}

void KBMTorus::changeTorposZ(double dtorposz) {
    // Changing the z-coordinate of the end cap results in the entire structure
    // essentially rotating in the z-y plane, which doesn't affect the optimal
    // configuration, but destabilizes the optimization. So, we turn it off.
    return;
    torposz += dtorposz;
    move_left_end_segments();
    move_rght_end_segments();
}

void KBMTorus::move_left_end_segments() {
    leftarm->setPoint(0,
            torposx, -torposy + sin(tortilt) * 2, -torposz - cos(tortilt) * 2,
            0.2, 0);
    leftarm->setPoint(1,
            torposx, -torposy, -torposz,
            0.2, 0);
    leftarm->setPoint(2,
            torposx, -torposy - sin(tortilt) * 2, -torposz + cos(tortilt) * 2,
            0.2, 0);

    leftarm->setPoint(3,
            -torposx, torposy + sin(tortilt) * 2, torposz - cos(tortilt) * 2,
            1.0, 0);
    leftarm->setPoint(4,
            -torposx, torposy, torposz,
            1.0, 0);
    leftarm->setPoint(5,
            -torposx, torposy - sin(tortilt) * 2, torposz + cos(tortilt) * 2,
            1.0, 0);

    leftarm->setStartFrameRotation(0);
    leftarm->setFinalFrameRotation(0);
    leftarm->compensateTwist();
}

void KBMTorus::create_leftarm() {
    vector<vec2> profile;
    profile.push_back(vec2(-1.0,  0.0));
    profile.push_back(vec2( 0.0,  1.0));
    profile.push_back(vec2( 1.0,  0.0));
    profile.push_back(vec2( 0.0, -1.0));

    vec3 q0 = vec3(0, 0, 0);
    vec3 q1 = vec3(0, 0, 0);
    vec3 q2 = vec3(0, 0, 0);

    // The following points are left in as an example of how to add points in
    // the middle of the arm for optimization. The move_left_end_segments
    // method would have to be updated to make sure the right indices are used
    // for the end segments if these points were added.

    // vec3 q3 = vec3(-0.3 * looprad,
    //                -torposy - 3 * sin(tortilt),
    //                -torposz + 3 * cos(tortilt));
    // vec3 q4 = vec3(-looprad, -cos(looptilt) * 2, -sin(looptilt) * 2);
    // vec3 q5 = vec3(-looprad,  cos(looptilt) * 2,  sin(looptilt) * 2);
    // vec3 q6 = vec3(-0.3 * looprad,
    //                torposy + 3 * sin(tortilt),
    //                torposz - 3 * cos(tortilt));

    // One could use the above points as is, or use the single point below.

    // vec3 qmid = (q3 + q4 + q5 + q6) / 4;

    vec3 q7 = vec3(0, 0, 0);
    vec3 q8 = vec3(0, 0, 0);
    vec3 q9 = vec3(0, 0, 0);

    vector<vec3> pts;
    pts.push_back(q0); pts.push_back(q1); pts.push_back(q2);
    pts.push_back(q7); pts.push_back(q8); pts.push_back(q9);

    vector<double> scales;
    scales.push_back(0.0); scales.push_back(0.0); scales.push_back(0.0);
    scales.push_back(0.0); scales.push_back(0.0); scales.push_back(0.0);

    leftarm = new SplineCoaster(pts, profile, scales, 0.0, 0.0);
    leftarm->setClosed(false);
    move_left_end_segments();
}

void KBMTorus::move_rght_end_segments() {
    rghtarm->setPoint(0,
            -torposx, torposy - sin(tortilt) * 2, torposz + cos(tortilt) * 2,
            0.2, 0);
    rghtarm->setPoint(1,
            -torposx, torposy, torposz,
            0.2, 0);
    rghtarm->setPoint(2,
            -torposx, torposy + sin(tortilt) * 2, torposz - cos(tortilt) * 2,
            0.2, 0);

    rghtarm->setPoint(12,
            torposx, -torposy - sin(tortilt) * 2, -torposz + cos(tortilt) * 2,
            1.0, 0);
    rghtarm->setPoint(13,
            torposx, -torposy, -torposz,
            1.0, 0);
    rghtarm->setPoint(14,
            torposx, -torposy + sin(tortilt) * 2, -torposz - cos(tortilt) * 2,
            1.0, 0);

    rghtarm->setStartFrameRotation(PI);
    rghtarm->setFinalFrameRotation(0);
    rghtarm->compensateTwist();
}

void KBMTorus::create_rghtarm() {
    vector<vec2> profile;
    profile.push_back(vec2(-1.0,  0.0));
    profile.push_back(vec2( 0.0,  1.0));
    profile.push_back(vec2( 1.0,  0.0));
    profile.push_back(vec2( 0.0, -1.0));

    vec3 q0 = vec3(0, 0, 0);
    vec3 q1 = vec3(0, 0, 0);
    vec3 q2 = vec3(0, 0, 0);

    vec3 q3 = vec3(0.3 * looprad,
                   torposy + 3 * sin(tortilt),
                   torposz - 3 * cos(tortilt));
    vec3 q4 = vec3(looprad,  cos(looptilt) * 2,  sin(looptilt) * 2);
    vec3 q5 = vec3(looprad, -cos(looptilt) * 2, -sin(looptilt) * 2);
    vec3 q6 = vec3(0.3 * looprad,
                   -torposy - 3 * sin(tortilt),
                   -torposz + 3 * cos(tortilt));

    vec3 q7 = vec3(0, 0, 0);
    vec3 q8 = vec3(0, 0, 0);
    vec3 q9 = vec3(0, 0, 0);

    vector<vec3> pts;
    pts.push_back(q0); pts.push_back(q1); pts.push_back(q2);
                       pts.push_back((q2 + q3) / 2 + vec3(0, 0, 0.1));
    pts.push_back(q3); pts.push_back((q3 + q4) / 2 + vec3(0, 0, 0.1));
    pts.push_back(q4); pts.push_back((q4 + q5) / 2 + vec3(0, 0, 0.1));
    pts.push_back(q5); pts.push_back((q5 + q6) / 2 + vec3(0, 0, 0.1));
    pts.push_back(q6); pts.push_back((q6 + q7) / 2 + vec3(0, 0, 0.1));
    pts.push_back(q7); pts.push_back(q8); pts.push_back(q9);

    vector<double> scales;
    scales.push_back(0.0); scales.push_back(0.0); scales.push_back(0.0);
    scales.push_back(0.2); scales.push_back(0.2);
    scales.push_back(0.2); scales.push_back(0.2);
    scales.push_back(0.2);
    scales.push_back(0.2); scales.push_back(0.2);
    scales.push_back(0.2); scales.push_back(0.2);
    scales.push_back(0.0); scales.push_back(0.0); scales.push_back(0.0);

    rghtarm = new SplineCoaster(pts, profile, scales, 0.0, 0.0);
    rghtarm->setClosed(false);
    move_rght_end_segments();
}
