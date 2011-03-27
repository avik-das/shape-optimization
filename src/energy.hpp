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
 * The smallest float that we will consider as non-zero.
 */
extern const float ZERO_THRESHOLD;

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
    Energy(float step_size_start, float step_size_end, int numparams);
    virtual void apply_change(VectorXf *chg) = 0;
    virtual float update_step_size(float old, float end) = 0;

    virtual void log_iteration(float step_size) = 0;

private:
    float step_size;
    float step_size_end;
    int numparams;
};

class ParameterizedTorusEnergy : public Energy {
public:
    ParameterizedTorusEnergy(ParameterizedTorus *torus);
    virtual float calc_energy();

protected:
    void apply_change(VectorXf *chg);
    float update_step_size(float old, float end);

    void log_iteration(float step_size);

    ParameterizedTorus *torus;
    virtual double compute_integrand(int v, int a);
};

class ParameterizedTorusEnergyStd : public ParameterizedTorusEnergy {
public:
    ParameterizedTorusEnergyStd(ParameterizedTorus *torus);

protected:
    double compute_integrand(int v, int a);
};

/**
 * The goal of this functional is to straighten out the torus as much as
 * possible. Because doing so can cause the torus to expand without bound, the
 * change in the parameters of the torus will be normalized at each iteration
 * so that the average fluctuation in the radial offsets is zero. This will
 * keep the average radius constant, which is different from the other
 * functionals that also aim to converge on an optimal average radius.
 */
class ParameterizedLineEnergy : public ParameterizedTorusEnergy {
public:
    ParameterizedLineEnergy(ParameterizedTorus *torus);
    float calc_energy();

protected:
    void apply_change(VectorXf *chg);
    double compute_integrand(int v, int a);
};

#endif
