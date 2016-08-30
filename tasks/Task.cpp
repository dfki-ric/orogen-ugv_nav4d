/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <ugv_nav4d/Planner.hpp>


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
    
    planner = new Planner(_primConfig.get(), _travConfig.get());
    
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
    maps::grid::MLSMapSloped map;
    auto ret = _map.readNewest(map);
    if(ret == RTT::NewData)
    {
        planner->updateMap(map);
    }
    
    if(ret == RTT::NoData)
    {
        setIfNotSet(NO_MAP);
        return;
    }
    
    base::samples::RigidBodyState start;
    base::samples::RigidBodyState stop;
    
    if(_start.readNewest(start) == RTT::NoData)
    {
        setIfNotSet(NO_START);
        return;
    }

    if(_goal.readNewest(stop) == RTT::NoData)
    {
        setIfNotSet(NO_GOAL);
        
        if(planner->plan(base::Time::fromSeconds(10), start, stop))
        {
            std::vector<base::Trajectory> trajectory;
            planner->getTrajectory(trajectory);
            _trajectory.write(trajectory);
        }
        else
        {
        }
        
        return;
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
