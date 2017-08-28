/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AreaExploration.hpp"
#include <ugv_nav4d/AreaExplorer.hpp>
#include <ugv_nav4d/FrontierGenerator.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.h>

#include <maps/operations/CoverageMapGeneration.hpp>

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
    CONFIGURE_DEBUG_DRAWINGS_USE_PORT_NO_THROW(this);

    frontGen = std::make_shared<FrontierGenerator>(_travConfig.get(), _costConfig.get());
    explorer = std::make_shared<AreaExplorer>(frontGen);

    if(_coverageRadius.get() > 0)
    {
        coverage = std::make_shared<maps::operations::CoverageTracker>();
    }
    else coverage.reset();

    FLUSH_DRAWINGS();
    
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
    CONFIGURE_DEBUG_DRAWINGS_USE_PORT_NO_THROW(this);
    
    AreaExplorationBase::updateHook();

    if(_map.readNewest(map, false) == RTT::NewData)
    {
        mapValid = true;
        frontGen->updateMap(map.data, coverage ? &coverage->getCoverage() : nullptr);
        if(coverage)
            coverage->setFrame(frontGen->getTraversabilityBaseMap());
    }

    if(_pose_samples.readNewest(curPose, false) == RTT::NewData)
    {
        if(!poseValid)
        {
            explorer->setInitialPatch(curPose.getTransform(), _initialPatchRadius.get());
            previousPose = curPose;
        }
        poseValid = true;
        if(coverage && mapValid && (curPose.position - previousPose.position).norm() > 0.05) // TODO make configurable
        {
            std::cout << "Adding coverage at " << curPose.position.transpose() << '\n';
            // TODO AngleSegment and orientation are ignored at the moment
            coverage->addCoverage(_coverageRadius.get(), base::AngleSegment{}, curPose.getPose());
            _coverage.write(coverage->getCoverage());
            previousPose = curPose;
        }
    }
    
    if(generateFrontiers)
    {
        state(PLANNING);
        
        std::vector<base::samples::RigidBodyState> outFrontiers;
        if(explorer->getFrontiers(curPose.position, area, outFrontiers))
        {
            state(GOALS_GENERATED);
            
            for(auto &f : outFrontiers)
            {
                //convert to ground frame
                f.position +=  f.orientation * Eigen::Vector3d(0,0, -_travConfig.get().distToGround);
            }
            
            _goals_out.write(outFrontiers);
        }
        else
        {
            //FIXME just for testing
            state(GOALS_GENERATED);

            for(auto &f : outFrontiers)
            {
                //convert to ground frame
                f.position +=  f.orientation * Eigen::Vector3d(0,0, -_travConfig.get().distToGround);
            }
            
            _goals_out.write(outFrontiers);
//             state(AREA_EXPLORED);
        }
        
        {
            envire::core::SpatioTemporal<maps::grid::TraversabilityBaseMap3d> trMap;
            trMap.frame_id = "AreaExploration";
            trMap.data = frontGen->getTraversabilityBaseMap();
            _tr_map.write(trMap);
        }
        
        generateFrontiers = false;
        
    FLUSH_DRAWINGS();
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
