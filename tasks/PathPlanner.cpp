/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PathPlanner.hpp"
#include <ugv_nav4d/Planner.hpp>
#include <envire_core/items/SpatioTemporal.hpp>

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
    if(state() != newState);
        state(newState);
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PathPlanner.hpp for more detailed
// documentation about them.

bool PathPlanner::configureHook()
{
    if(planner)
        delete planner;

    planner = new Planner(_primConfig.get(), _travConfig.get(), _mobilityConfig.get());
    
    if (! PathPlannerBase::configureHook())
        return false;
    return true;

}
bool PathPlanner::startHook()
{
    if (! PathPlannerBase::startHook())
        return false;
    return true;
}
void PathPlanner::updateHook()
{
    envire::core::SpatioTemporal<maps::grid::MLSMapKalman> map;
    auto map_status = _map.readNewest(map);

    if(map_status == RTT::NoData)
    {
        setIfNotSet(NO_MAP);
        return;
    } else if(map_status == RTT::NewData)
    {
        planner->updateMap(map.getData());
    } 
    
    base::samples::RigidBodyState start_pose;
    base::samples::RigidBodyState stop_pose;
    
    if(_start_position.readNewest(start_pose) == RTT::NoData)
    {
        setIfNotSet(NO_START);
        return;
    } else
    {
        std::cout << "StartPose: " << start_pose.position.x() << " "
            << start_pose.position.y() << " "
            << start_pose.position.z() << " " << std::endl;
    } 

    auto goal_pose_status = _goal_position.readNewest(stop_pose);

    if(goal_pose_status == RTT::NoData)
    {
        setIfNotSet(NO_GOAL);        
        return;
    } else
    {
        std::cout << "GoalPose: " << stop_pose.position.x() << " "
            << stop_pose.position.y() << " "
            << stop_pose.position.z() << " " << std::endl;
                  
    } 

    if (goal_pose_status == RTT::NewData || map_status == RTT::NewData)
    {
        setIfNotSet(PLANNING);
        std::vector<base::Trajectory> trajectory;
        if(planner->plan(_maxTime.value(), start_pose, stop_pose, trajectory))
        {
            _trajectory.write(trajectory);
            
            setIfNotSet(FOUND_SOLUTION);
        } else
        {
            setIfNotSet(NO_SOLUTION);
        }
    }
    
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
    PathPlannerBase::cleanupHook();
}
