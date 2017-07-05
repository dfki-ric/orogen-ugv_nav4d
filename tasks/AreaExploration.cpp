/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AreaExploration.hpp"
#include <ugv_nav4d/AreaExplorer.hpp>
#include <ugv_nav4d/FrontierGenerator.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.h>

using namespace ugv_nav4d;

AreaExploration::AreaExploration(std::string const& name)
    : AreaExplorationBase(name)
{
}

AreaExploration::AreaExploration(std::string const& name, RTT::ExecutionEngine* engine)
    : AreaExplorationBase(name, engine)
{
}

AreaExploration::~AreaExploration()
{
}

void AreaExploration::calculateGoals(::ugv_nav4d::OrientedBoxConfig const & area)
{
    if(!mapValid)
    {
        state(NO_MAP);
        return;
    }

    if(!poseValid)
    {
        state(ugv_nav4d::AreaExplorationBase::NO_POSE);
        return;
    }
    
    generateFrontiers = true;
    this->area = area;
}

void AreaExploration::clearPlannerMap()
{

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See AreaExploration.hpp for more detailed
// documentation about them.

bool AreaExploration::configureHook()
{
    frontGen = std::make_shared<FrontierGenerator>(_travConfig.get(), _costConfig.get());
    
    explorer = std::make_shared<AreaExplorer>(frontGen);
    
    if (! AreaExplorationBase::configureHook())
        return false;
    return true;
}
bool AreaExploration::startHook()
{
    poseValid = false;
    mapValid = false;
    
    generateFrontiers = false;
    
    if (! AreaExplorationBase::startHook())
        return false;
    return true;
}
void AreaExploration::updateHook()
{
    CONFIGURE_DEBUG_DRAWINGS_USE_PORT_NO_THROW(&_debugDrawings);
    
    AreaExplorationBase::updateHook();
    if(_pose_samples.readNewest(curPose, false) == RTT::NewData)
    {
        if(!poseValid)
        {
            frontGen->setInitialPatch(curPose.getTransform(), _distToGround.get(), _initialPatchRadius.get());
        }
        poseValid = true;
    }
    
    envire::core::SpatioTemporal<maps::grid::MLSMapKalman> map;
    if(_map.readNewest(map, false) == RTT::NewData)
    {
        mapValid = true;
        frontGen->updateMap(map.data);
    }
    
    if(generateFrontiers)
    {
        state(PLANNING);
        
        std::vector<base::samples::RigidBodyState> outFrontiers;
        if(explorer->getFrontiers(curPose.position, area, outFrontiers))
        {
            state(GOALS_GENERATED);
            _goals_out.write(outFrontiers);
        }
        else
        {
            state(AREA_EXPLORED);
        }
        
        envire::core::SpatioTemporal<maps::grid::TraversabilityBaseMap3d> foo;
        foo.frame_id = "AreaExploration";
        foo.data = frontGen->getTraversabilityBaseMap();
        
        _tr_map.write(foo);
        
        generateFrontiers = false;
    }
}
void AreaExploration::errorHook()
{
    AreaExplorationBase::errorHook();
}
void AreaExploration::stopHook()
{
    AreaExplorationBase::stopHook();
}
void AreaExploration::cleanupHook()
{
    AreaExplorationBase::cleanupHook();
}
