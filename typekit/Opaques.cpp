/* Generated from orogen/lib/orogen/templates/typekit/Opaques.cpp */

#include <ugv_nav4d/typekit/OpaqueTypes.hpp>
#include <ugv_nav4d/typekit/Opaques.hpp>

    /** Returns the intermediate value that is contained in \c real_type */
    /** Stores \c intermediate into \c real_type. \c intermediate is owned by \c
     * real_type afterwards. */
    /** Release ownership of \c real_type on the corresponding intermediate
     * pointer.
     */


void orogen_typekits::toIntermediate(::wrappers::DiscreteTheta& intermediate, DiscreteTheta const& real_type)
{
    intermediate.theta = real_type.getTheta();
    intermediate.numAngles = M_PI * 2.0 * real_type.getTheta() / real_type.getRadian();
}

void orogen_typekits::fromIntermediate(DiscreteTheta& real_type, ::wrappers::DiscreteTheta const& intermediate)
{
    real_type = DiscreteTheta(intermediate.theta, intermediate.numAngles);
}

