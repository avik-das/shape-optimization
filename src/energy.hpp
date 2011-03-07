#ifndef ENERGY_HPP
#define ENERGY_HPP

#include "model.hpp"

#include <Eigen/Core>
USING_PART_OF_NAMESPACE_EIGEN

#include <cstdarg>

/* TODO: document properly */

/**
 * The smallest change in surface energy that is considered significant. Any
 * changes to the energy that fall below this threshold are considered equal,
 * suggesting the simulation has converged to a stable minimum.
 */
extern const float ENERGY_THRESHOLD;

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

protected:
    Energy(int nparams, ...);
    virtual void apply_change(VectorXf *chg) = 0;
    virtual VectorXf *update_step_size(VectorXf *old, VectorXf *end) = 0;

    virtual void log_iteration(VectorXf *step_size) = 0;

private:
    VectorXf *step_size;
    VectorXf *step_size_end;
};

/**
 * The bending surface energy of a <code>SimpleTorus</code>.
 */
class SimpleTorusEnergy : public Energy {
public:
    SimpleTorusEnergy(SimpleTorus *simple_torus);
    float calc_energy();

protected:
    void apply_change(VectorXf *chg);
    VectorXf *update_step_size(VectorXf *old, VectorXf *end);

    void log_iteration(VectorXf *step_size);

private:
    SimpleTorus *simple_torus;

    double compute_integrand(int v, int a);
};

#endif
