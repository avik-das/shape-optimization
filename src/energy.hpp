#ifndef ENERGY_HPP
#define ENERGY_HPP

#include "model.hpp"

#include <Eigen/Core>
USING_PART_OF_NAMESPACE_EIGEN

/* TODO: document properly */

extern const float ENERGY_THRESHOLD;

class Energy {
public:
    virtual float calc_energy() = 0;
    virtual bool iterate() = 0;
};

class SimpleTorusEnergy : public Energy {
public:
    SimpleTorusEnergy(SimpleTorus *simple_torus);
    float calc_energy();
    bool iterate();

private:
    SimpleTorus *simple_torus;

    const float START_STEP_SIZE;
    const float   END_STEP_SIZE;
    float step_size;
    double compute_integrand(int v, int a);
};

#endif
