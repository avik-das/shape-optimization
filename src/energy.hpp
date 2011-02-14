#ifndef ENERGY_HPP
#define ENERGY_HPP

#include "model.hpp"

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
};

#endif
