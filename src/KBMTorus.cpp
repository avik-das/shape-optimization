#include "KBMTorus.h"

#include "global.h"

#include <boost/math/constants/constants.hpp>
const double PI = boost::math::constants::pi<double>();

KBMTorus::KBMTorus() :
    torposx(0.0), torposy( 8.0), torposz(0.0),
    tortilt(-PI/2),
    looprad( 5.0), looptilt(0.0),
    hasDL(false) {

    // To avoid local minima in the optimization, we can start the optimization
    // at a state from which it will proceed in the direction we want. Often,
    // this starting state is obtained by running the optimization with a
    // specific set of weights and taking the final state. Then, the new state
    // can be further optimized with different weights or even a more refined
    // objective.
    //
    // This is one such state achieved by allowing the end cap position to
    // move around. Once this is achieved, the end caps can be fixed and the
    // other parameters optimized separately.
    torposx = 0      ;
    torposy = 3.2332 ;
    torposz = 0      ;
    tortilt = -PI / 2;

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
    rghtarm->setStartFrameRotation(leftarm->getGlobalTwist() * PI / 180 - PI);
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

    leftarm->setStartFrameRotation(PI);
    leftarm->setFinalFrameRotation(0);
    leftarm->compensateTwist();
}

void KBMTorus::create_leftarm() {
    vector<vec2> profile;
    for (float t = 0.0f; t < 2 * PI; t += PI / 8)
        profile.push_back(vec2(cos(t + PI/2), sin(t + PI/2)));

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

    leftarm->setColors(Color(0.75,1,0.75), Color(0,1,0));
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
    for (float t = 0.0f; t < 2 * PI; t += PI / 8)
        profile.push_back(vec2(cos(-t + PI/2), sin(-t + PI/2)));

    // As mentioned in the constructor, the optimized state starting at a
    // certain configuration can be used as the starting point for another
    // optimization under different parameters. Below is the general starting
    // point, and below that are alternative starting points based on data
    // collected from optimizing the first optimization.

    // vec3 q0 = vec3(0, 0, 0);
    // vec3 q1 = vec3(0, 0, 0);
    // vec3 q2 = vec3(0, 0, 0);

    // vec3 q3 = vec3(0.3 * looprad,
    //                torposy + 3 * sin(tortilt),
    //                torposz - 3 * cos(tortilt));
    // vec3 q4 = vec3(looprad,  cos(looptilt) * 2,  sin(looptilt) * 2);
    // vec3 q5 = vec3(looprad, -cos(looptilt) * 2, -sin(looptilt) * 2);
    // vec3 q6 = vec3(0.3 * looprad,
    //                -torposy - 3 * sin(tortilt),
    //                -torposz + 3 * cos(tortilt));

    // vec3 q7 = vec3(0, 0, 0);
    // vec3 q8 = vec3(0, 0, 0);
    // vec3 q9 = vec3(0, 0, 0);

    // vector<vec3> pts;
    // pts.push_back(q0); pts.push_back(q1); pts.push_back(q2);
    //                    pts.push_back((q2 + q3) / 2 + vec3(0,  2, 0.1));
    // pts.push_back(q3); pts.push_back((q3 + q4) / 2 + vec3(0,  0, 0.1));
    // pts.push_back(q4); pts.push_back((q4 + q5) / 2 + vec3(0,  0, 0.1));
    // pts.push_back(q5); pts.push_back((q5 + q6) / 2 + vec3(0,  0, 0.1));
    // pts.push_back(q6); pts.push_back((q6 + q7) / 2 + vec3(0, -2, 0.1));
    // pts.push_back(q7); pts.push_back(q8); pts.push_back(q9);

    // These end caps would be overwritten by move_rght_end_segments, but are
    // presented here for documentation purposes. They can be used, for
    // example, to infer the values of the torpos* parameters.
    vec3  q0(0, 5.2332, 0);
    vec3  q1(0, 3.2332, 0);
    vec3  q2(0, 1.2332, 0);

    // These points correspond to a configuration with a good deal of bending,
    // but is between the original starting state and the case where the right
    // arm has 360 degrees of twist. It can sometimes lead to interesting
    // ending configurations.

    // vec3  q3( 1.26247,  0.706094, -0.66221 );
    // vec3  q4( 2.65104,  1.00887 , -1.01403 );
    // vec3  q5( 4.03713,  1.20907 , -0.942492);
    // vec3  q6( 5.21445,  0.790136, -0.475025);
    // vec3  q7( 5.69695, -0.218763,  0.279934);
    // vec3  q8( 5.18405, -1.29349 ,  1.153   );
    // vec3  q9( 3.88804, -1.71263 ,  1.8182  );
    // vec3 q10( 2.34332, -1.30503 ,  1.88599 );
    // vec3 q11( 0.97616, -0.754706,  1.11436 );

    // These points correspond to a configuration in which the twist in the
    // right arm is 270 degrees. Unlike the points below, this configuration
    // is not terribly pleasing in its lack of symmetry.

    // vec3  q3(0.87388, -0.686299, -0.749497);
    // vec3  q4(2.42933, -2.09514 , -1.61288 );
    // vec3  q5(4.28153, -2.82464 , -1.91695 );
    // vec3  q6(5.85939, -2.84271 , -1.39271 );
    // vec3  q7(6.58867, -2.28751 , -0.306117);
    // vec3  q8(6.19908, -1.49546 ,  0.736241);
    // vec3  q9(4.89114, -0.890667,  1.13686 );
    // vec3 q10(3.21958, -0.798966,  0.630432);
    // vec3 q11(1.47744, -0.521628,  0       );

    // These points correspond to a suitable configuration in which the twist
    // in the right arm is 270 degrees.

    vec3  q3(1.2533 , -0.286404,  0       );
    vec3  q4(3.12454, -1.02497 , -0.694888);
    vec3  q5(5.006  , -1.17663 , -1.17067 );
    vec3  q6(6.47057, -0.95954 , -0.807342);
    vec3  q7(7.01271, -0.474653,  0.207576);
    vec3  q8(6.40876,  0.106312,  1.14899 );
    vec3  q9(4.94206,  0.568519,  1.34665 );
    vec3 q10(3.10503,  0.700904,  0.735122);
    vec3 q11(1.25964,  0.178051,  0       );

    // These points correspond to a suitable configuration in which the twist
    // in the right arm is 360 degrees. This was obtained by attempting to
    // acquire this much twist in the right arm while allowing the end cap to
    // move in the Y direction. As such, the end caps in this configuration
    // were fairly close together. Fixing the end caps farther apart and
    // optimizing for 360 degrees of twist will leave the shape intact while
    // shrinking the loop to compensate for the new end cap positions.

    // vec3  q3(1.03951, -2.73942 ,  0       );
    // vec3  q4(2.84641, -3.67611 , -0.138622);
    // vec3  q5(4.87569, -3.48998 , -0.359469);
    // vec3  q6(6.48027, -2.21311 , -0.578368);
    // vec3  q7(7.12872, -0.242812, -0.71425 );
    // vec3  q8(6.59604,  1.78181 , -0.716681);
    // vec3  q9(5.0491 ,  3.19548 , -0.583956);
    // vec3 q10(2.99666,  3.5368  , -0.364957);
    // vec3 q11(1.11102,  2.70644 , -0.141622);

    vec3 q12(0, -1.2332, 0);
    vec3 q13(0, -3.2332, 0);
    vec3 q14(0, -5.2332, 0);

    vector<vec3> pts;

    // Given the initial points at the beginning of this function, additional
    // points would have to be added.

    // pts.push_back(q0); pts.push_back(q1); pts.push_back(q2);
    //                    pts.push_back((q2 + q3) / 2 + vec3(0, 0, 0.1));
    // pts.push_back(q3); pts.push_back((q3 + q4) / 2 + vec3(0, 0, 0.1));
    // pts.push_back(q4); pts.push_back((q4 + q5) / 2 + vec3(0, 0, 0.1));
    // pts.push_back(q5); pts.push_back((q5 + q6) / 2 + vec3(0, 0, 0.1));
    // pts.push_back(q6); pts.push_back((q6 + q7) / 2 + vec3(0, 0, 0.1));
    // pts.push_back(q7); pts.push_back(q8); pts.push_back(q9);

    pts.push_back( q0); pts.push_back( q1); pts.push_back( q2);
    pts.push_back( q3); pts.push_back( q4); pts.push_back( q5);
    pts.push_back( q6); pts.push_back( q7); pts.push_back( q8);
    pts.push_back( q9); pts.push_back(q10); pts.push_back(q11);
    pts.push_back(q12); pts.push_back(q13); pts.push_back(q14);

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

    rghtarm->setColors(Color(1,0.5,0.5), Color(1,0,0));
}
