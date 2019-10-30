#pragma once

#include <base/Pose.hpp>
#include <maps/grid/Index.hpp>
#include <trajectory_follower/SubTrajectory.hpp>
#include <ugv_nav4d/PreComputedMotions.hpp>

namespace ugv_nav4d
{
    struct TrajWMotions
    {
        std::vector<base::Trajectory> trajectories;
        std::vector<Motion> motions;
    };
}


namespace wrappers
{
    struct DiscreteTheta
    {
        int theta;
        int numAngles;
    };     
}
