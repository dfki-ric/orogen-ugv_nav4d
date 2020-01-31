#pragma once

#include <base/Pose.hpp>
#include <maps/grid/Index.hpp>
#include <trajectory_follower/SubTrajectory.hpp>
#include <ugv_nav4d/PreComputedMotions.hpp>
#include <ugv_nav4d/OrientedBox.hpp>

namespace ugv_nav4d
{
}


namespace wrappers
{
    struct DiscreteTheta
    {
        int theta;
        int numAngles;
    };     
}
