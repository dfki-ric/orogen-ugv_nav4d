/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PathPlanner.hpp"
#include <ugv_nav4d/Planner.hpp>
#include <envire_core/items/SpatioTemporal.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.h>

using namespace ugv_nav4d;

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

int32_t PathPlanner::triggerPathPlanning(const base::samples::RigidBodyState& start_position, const base::samples::RigidBodyState& goal_position)
{
    if(!gotMap)
        return 0;

    start_pose = start_position;
    stop_pose = goal_position;
    //goal position is in ground frame, transform to body frame
    stop_pose.position += stop_pose.orientation * Eigen::Vector3d(0,0, _travConfig.get().distToGround);
    
    _planning_start.write(start_position);
    _planning_goal.write(goal_position);
    
    executePlanning = true;
    
    return 1;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PathPlanner.hpp for more detailed
// documentation about them.

bool PathPlanner::configureHook()
{
    CONFIGURE_DEBUG_DRAWINGS_USE_PORT_NO_THROW(this);

    planner.reset(new Planner(_primConfig.get(), _travConfig.get(), _mobilityConfig.get()));
    
    planner->setTravMapCallback([&] () {
        envire::core::SpatioTemporal<maps::grid::TraversabilityBaseMap3d> strmap;
        strmap.data = planner->getTraversabilityMap();
        strmap.frame_id = "Traversability";
        
        _tr_map.write(strmap);
        
        FLUSH_DRAWINGS();
    });
    

    
    FLUSH_DRAWINGS();
    
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
    CONFIGURE_DEBUG_DRAWINGS_USE_PORT_NO_THROW(this);
    
    envire::core::SpatioTemporal<maps::grid::MLSMapKalman> map;
    auto map_status = _map.readNewest(map, false);

    if(map_status == RTT::NoData)
    {
        setIfNotSet(NO_MAP);
        return;
    } else if(map_status == RTT::NewData)
    {
        gotMap = true;
        planner->updateMap(map.getData());
    } 
    
    
    if(executePlanning)
    {
        if(!initalPatchAdded)
        {
            planner->setInitialPatch(start_pose.getTransform(), _initialPatchRadius.get());
            initalPatchAdded = true;
        }
        
        setIfNotSet(PLANNING);
        std::vector<base::Trajectory> trajectory;
        
        CLEAR_DRAWING("planner_goal");
        DRAW_AXES("planner_goal", stop_pose.position, stop_pose.orientation);
        
        Planner::PLANNING_RESULT res = planner->plan(_maxTime.value(), start_pose, stop_pose, trajectory);
        switch(res)
        {
            case Planner::FOUND_SOLUTION:
                _trajectory.write(trajectory);
                _motionPrims.write(planner->getMotions());
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
    
    PathPlannerBase::updateHook();
    FLUSH_DRAWINGS();
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
