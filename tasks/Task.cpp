/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <ugv_nav4d/Planner.hpp>
#include <envire_core/items/SpatioTemporal.hpp>


using namespace ugv_nav4d;

Task::Task(std::string const& name)
    : TaskBase(name), planner(nullptr)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), planner(nullptr)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if(planner)
        delete planner;

    planner = new Planner(_primConfig.get(), _travConfig.get(), _mobilityConfig.get());
    
    if (! TaskBase::configureHook())
        return false;
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}

void Task::setIfNotSet(const TaskBase::States& newState)
{
    if(state() != newState);
        state(newState);
}

void Task::updateHook()
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
        if(planner->plan(_maxTime.value(), start_pose, stop_pose))
        {
            std::vector<base::Trajectory> trajectory;
            planner->getTrajectory(trajectory);
            _trajectory.write(trajectory);
            
            setIfNotSet(FOUND_SOLUTION);
        } else
        {
            setIfNotSet(NO_SOLUTION);
        }
    }
    
    
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
