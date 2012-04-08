#ifndef ENERGY_HPP
#define ENERGY_HPP

#include <Eigen/Core>
using namespace Eigen;

#include "SplineCoaster.h"

/* TODO: document properly */

/**
 * The smallest change in surface energy that is considered significant. Any
 * changes to the energy that fall below this threshold are considered equal,
 * suggesting the simulation has converged to a stable minimum.
 */
extern const float ENERGY_THRESHOLD;

/**
 * The smallest float that we will consider as non-zero.
 */
extern const float ZERO_THRESHOLD;

/**
 * The spring constant used to determine how energetic a stretched strut is.
 */
extern const double ELASTICITY;

/**
 * The base class for all surface energy-based simulators. Typically, an
 * implementation of this class will hold onto a geometric structure, and will
 * perform calculations on that structure in order to calculate the surface
 * energy of that structure.
 *
 * Additionally, implementations of this class will also vary the parameters of
 * the geometric structure and descend along the gradient of greater surface
 * energy decrease, one step at a time, adapting the step size in order to
 * eventually converge on a stable solution..
 */
class Energy {
public:
    virtual float calc_energy() = 0;
    virtual bool iterate();

    void   set_speed(double speed);
    double get_speed();

protected:
    Energy(float step_size_start, float step_size_end, int numparams);
    virtual void apply_change(VectorXf *chg) = 0;
    virtual float update_step_size(float old, float end) = 0;

    virtual void log_iteration(float step_size) = 0;
    virtual void log_energies() = 0;

private:
    float step_size;
    float step_size_end;
    int numparams;

    double speed;
};

class LineEnergy : public Energy {
public:
    LineEnergy(SplineCoaster *torus, double twist_weight);
    float calc_energy();

protected:
    void apply_change(VectorXf *chg);
    float update_step_size(float old, float end);

    void log_iteration(float step_size);
    void log_energies();

    SplineCoaster *torus;
    double compute_integrand(double t, double dt);

    double twist_weight;
};
#endif
