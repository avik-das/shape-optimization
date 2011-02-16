#ifndef ENERGY_HPP
#define ENERGY_HPP

#include "model.hpp"

#include <Eigen/Core>
USING_PART_OF_NAMESPACE_EIGEN

/* TODO: document properly */

class Energy {
public:
    virtual float calc_energy() = 0;
    virtual void iterate() = 0;
};

class SimpleTorusEnergy : public Energy {
public:
    SimpleTorusEnergy(SimpleTorus *simple_torus);
    float calc_energy();
    void iterate();

private:
    SimpleTorus *simple_torus;
    const int ARM_RESOLUTION;

    double compute_integrand(int v, int a);

    Vector3f get_point(int v, int a);
};

#endif
