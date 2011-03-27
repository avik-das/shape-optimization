#ifndef MODEL_HPP
#define MODEL_HPP

#include "util.hpp"
#include <string>
#include <vector>

#include <Eigen/Core>
USING_PART_OF_NAMESPACE_EIGEN

/* TODO: document properly */

/**
 * A 3D point as well as the surface normal vector at that point.
 */
class PointNormal {
public:
    PointNormal(Vector3f *point, Vector3f *normal);
    ~PointNormal();

    Vector3f *point;
    Vector3f *normal;
};

/**
 * A ParameterizedTorus is a torus, the surface points of which can be
 * addressed using exactly two parameters. Regardless of the shape of the torus
 * or the internal representation, there is one parameter representing the
 * distance along the arm of the torus and one parameter representing the angle
 * around the arm.
 * 
 * Subclasses of will then provide methods of calculating the 3D point indexed
 * by these two parameters.
 */
class ParameterizedTorus {
public:
    /**
     * Create a new <code>ParamaterizedTorus</code>.
     *
     * @param resolution  the number of points along the arm <em>and</em> the
     *                    number of points around the arm. Each index used to
     *                    address points on the torus' surface must be between
     *                    0 and resolution, including 0 but not including
     *                    resolution.
     */
    ParameterizedTorus(int resolution);
    
    /**
     * Calculates the location of the addressed point in 3D, as well as the
     * normal to the torus' surface at that point. Depends on the subclass'
     * implementation of <code>get_point(int, int)</code>.
     *
     * @param v  the index along the arm
     * @param a  the index around the arm
     * @return  the point addressed by <v, a>, as well as the normal to the
     *          surface at that point
     */
    PointNormal *get_point_normal(int v, int a);

    /**
     * Calculates the location of the addressed point in 3D. Must be
     * implemented by the subclass.
     *
     * @param v  the index along the arm
     * @param a  the index around the arm
     * @return  the point addressed by <v, a>
     */
    virtual Vector3f *get_point(int v, int a) = 0;

    /**
     * Applies the changes to the parameters of this torus as specified by the
     * given vector.
     *
     * The optimization process varies the parameters of this torus, and uses
     * these variations to determine what changes yield the lowest energy
     * configuration. The optimizer does not know anything about the number or
     * nature of the parameters that define this torus, so the torus may
     * interpret this vector in any way. The only restrictions that this
     * interpretation must be consistent over time, and the changes be linear,
     * i.e. that applying this change vector twice is the same as applying
     * double this change vector.
     *
     * @param chg  the change vector, with each coordinate representing a
     *             linear change to one of the parameters. The size of this
     *             vector is gauranteed to be the same as the number returned
     *             by <code>numparams()</code>.
     */
    virtual void apply_change(VectorXf *chg) = 0;

    /**
     * TODO
     */
    virtual float update_step_size(float old, float end) = 0;

    /**
     * The number of parameters that are involved in the optimization process.
     * It is gauranteed that the change vectors passed to
     * <code>apply_change()</code> have exactly this many entries in them.
     *
     * @return  the number of parameters involved
     */
    virtual int numparams() = 0;

    /**
     * Constructs a string representation of this torus. Typically, this will
     * be used to log the progress of the optimization, so certain pieces of
     * important information should be printed, but not all (since there will
     * typically be a large amount of information).
     */
    virtual std::string str() = 0;

    /**
     * TODO
     */
    int ring_vert;

    /**
     * TODO
     */
    int arm_vert;

    /**
     * TODO
     */
    const float ARM_RADIUS;
};

/**
 * A torus that represents perturbations to a perfectly circular torus. There
 * is a global radius. Additionally, because there is a finite resolution,
 * there are a finite number of points along the arm, each of which represents
 * cross-section of the arm. The center of each cross-section can then be
 * perturbed by some amount along the radial direction, i.e. toward the center
 * of the torus or away from it.
 */
class RadialTorus : public ParameterizedTorus {
public:
    /**
     * TODO
     */
    RadialTorus(float ring_radius, int resolution);

    // implementations of virtual methods
    Vector3f *get_point(int v, int a);
    void apply_change(VectorXf *chg);
    float update_step_size(float old, float end);
    int numparams();
    std::string str();

private:
    /**
     * TODO
     */
    float get_radial_offset(int v);

    /**
     * TODO
     */
    void  set_radial_offset(int v, float val);

    /**
     * The global radius of the torus. All perturbations to the individual
     * points along the arm are relative to this distance.
     */
    float ring_radius;

    /**
     * TODO
     */
    std::vector<float> *radial_offsets;
};

#endif
