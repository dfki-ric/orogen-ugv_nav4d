/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PathPlanner.hpp"
#include "ugv_nav4dTypes.hpp"
#include <ugv_nav4d/Planner.hpp>
#include <envire_core/items/SpatioTemporal.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <trajectory_follower/SubTrajectory.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

using namespace ugv_nav4d;

using Eigen::Vector3d;
using Eigen::Affine3d;
using Eigen::Translation3d;
using maps::grid::TraversabilityNodeBase;
using maps::grid::TraversabilityBaseMap3d;
using envire::core::SpatioTemporal;

PathPlanner::PathPlanner(std::string const& name)
    : PathPlannerBase(name), planner(nullptr)
{
}

PathPlanner::PathPlanner(std::string const& name, RTT::ExecutionEngine* engine)
    : PathPlannerBase(name, engine), planner(nullptr)
{
}

PathPlanner::~PathPlanner()
{
}

void PathPlanner::setIfNotSet(const PathPlannerBase::States& newState)
{
    if(state() != newState)
        state(newState);
}


int32_t PathPlanner::triggerPathPlanning()
{
    if(!gotMap)
        return 0;

    // read start pose from input port
    _start_pose_samples.read(start_pose);
    _goal_pose_samples.read(stop_pose);

    

    _planning_start.write(start_pose);
    _planning_goal.write(stop_pose);

    executePlanning = true;

    return 1;
}

int32_t PathPlanner::triggerPathPlanningRelative()
{
    if(!gotMap)
        return 0;

    // read start pose from input port
    _start_pose_samples.read(start_pose);
    _goal_pose_samples.read(stop_pose);

    stop_pose.setTransform(start_pose.getTransform() * stop_pose.getTransform());

    _planning_start.write(start_pose);
    _planning_goal.write(stop_pose);

    executePlanning = true;

    return 1;
}

bool PathPlanner::configureHook()
{
    std::vector<std::string> channels = V3DD::GET_DECLARED_CHANNELS();
    std::vector<std::string> channels_filtered;

    for(const std::string& channel : channels)
    {
        //check if it contains ugv
        if(channel.find("ugv") != std::string::npos)
        {
            channels_filtered.push_back(channel);
        }
    }

    V3DD::CONFIGURE_DEBUG_DRAWINGS_USE_PORT(this, channels_filtered);

    Eigen::Affine3d mls2Ground(Eigen::Translation3d(_gridOffset.rvalue()));

    planner.reset(new Planner(_primConfig.get(), _travConfig.get(), _mobilityConfig.get(), _plannerConfig.get(), mls2Ground));


    planner->setTravMapCallback([&] ()
    {
        //this callback will be called whenever the planner has generated a new travmap.

        SpatioTemporal<TraversabilityBaseMap3d> st(planner->getTraversabilityMap().copyCast<TraversabilityNodeBase*>());
        // planner has static transformation to world by [10, 10, 0]
        st.setFrameID("planner");
        _tr_map.write(st);
    });

    V3DD::FLUSH_DRAWINGS();

    if (! PathPlannerBase::configureHook())
        return false;
    return true;

}
bool PathPlanner::startHook()
{
    if (! PathPlannerBase::startHook())
        return false;

    initalPatchAdded = false;
    executePlanning = false;
    gotMap = false;
    return true;
}

void PathPlanner::updateHook()
{

    envire::core::SpatioTemporal<maps::grid::MLSMapKalman> map;
    auto map_status = _map.readNewest(map, false);

    if(map_status == RTT::NoData)
    {
        setIfNotSet(NO_MAP);
        return;
    } else if(map_status == RTT::NewData)
    {
        gotMap = true;
        map.data.translate(_gridOffset.rvalue());
        planner->updateMap(map.getData());
        setIfNotSet(GOT_MAP);
    }

    if(executePlanning)
    {
        if(!initalPatchAdded)
        {
            planner->setInitialPatch(start_pose.getTransform(), _initialPatchRadius.get());
            initalPatchAdded = true;
        }

        setIfNotSet(PLANNING);
        std::vector<base::Trajectory> trajectory2D;
        std::vector<base::Trajectory> trajectory3D;


        Eigen::Vector3d contact_point;
        const auto translated_end = stop_pose.position + _gridOffset.rvalue();
        map.data.getClosestContactPoint(translated_end, contact_point);
        stop_pose.position.z() = contact_point.z();

        Planner::PLANNING_RESULT res = planner->plan(_maxTime.value(), start_pose, stop_pose, trajectory2D, trajectory3D, _dumpOnError.get(), _dumpOnSuccess.get());
        switch(res)
        {
            case Planner::FOUND_SOLUTION:
                _trajectory2D.write(trajectory2D);
                _trajectory3D.write(trajectory3D);
                setIfNotSet(FOUND_SOLUTION);
                break;
            case Planner::GOAL_INVALID:
                setIfNotSet(ugv_nav4d::PathPlannerBase::GOAL_INVALID);
                break;
            case Planner::START_INVALID:
                setIfNotSet(ugv_nav4d::PathPlannerBase::START_INVALID);
                break;
            case Planner::INTERNAL_ERROR:
                setIfNotSet(ugv_nav4d::PathPlannerBase::INTERNAL_ERROR);
                break;
            case Planner::NO_SOLUTION:
                setIfNotSet(ugv_nav4d::PathPlannerBase::NO_SOLUTION);
                break;
            case Planner::NO_MAP:
                setIfNotSet(ugv_nav4d::PathPlannerBase::NO_MAP);
                break;
        }

        executePlanning = false;
    }

    V3DD::FLUSH_DRAWINGS();
    PathPlannerBase::updateHook();
}
void PathPlanner::errorHook()
{
    PathPlannerBase::errorHook();
}
void PathPlanner::stopHook()
{
    PathPlannerBase::stopHook();
}
void PathPlanner::cleanupHook()
{
    planner.reset();
    PathPlannerBase::cleanupHook();
}
