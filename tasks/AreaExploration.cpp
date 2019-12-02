/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AreaExploration.hpp"
#include <ugv_nav4d/AreaExplorer.hpp>
#include <ugv_nav4d/FrontierGenerator.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>

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
    generateFrontiers = true;
    this->area = area;
}

void AreaExploration::clearPlannerMap()
{

}

bool AreaExploration::configureHook()
{
    std::vector<std::string> channels = V3DD::GET_DECLARED_CHANNELS();
    std::vector<std::string> channels_filtered;
    for(const std::string& channel : channels)
    {
        //check if it contains ugv
        if(channel.find("area_explore") != std::string::npos)
        {
            channels_filtered.push_back(channel);
        }
    }
    V3DD::CONFIGURE_DEBUG_DRAWINGS_USE_PORT(this, channels_filtered);

    frontGen = std::make_shared<FrontierGenerator>(_travConfig.get(), _costConfig.get());
    explorer = std::make_shared<AreaExplorer>(frontGen);

    if(_coverageRadius.get() > 0)
    {
        coverage = std::make_shared<maps::operations::CoverageTracker>();
    }
    else coverage.reset();

    V3DD::FLUSH_DRAWINGS();
    
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
   
    AreaExplorationBase::updateHook();

    if(_map.readNewest(map, false) == RTT::NewData)
    {
        mapValid = true;
        frontGen->updateMap(map.data, coverage ? &coverage->getCoverage() : nullptr);
        if(coverage)
            coverage->setFrame(frontGen->getTraversabilityMap());
    }

    if(_pose_samples.readNewest(curPose, false) == RTT::NewData)
    {
        if(!poseValid)
        {
            explorer->setInitialPatch(curPose.getTransform(), _initialPatchRadius.get());
            previousPose = curPose;
        }
        poseValid = true;
        
        if(coverage && mapValid && (curPose.position - previousPose.position).norm() > _coverageUpdateDistance.get())
        {
            std::cout << "Adding coverage at " << curPose.position.transpose() << '\n';
            // TODO AngleSegment and orientation are ignored at the moment
            coverage->addCoverage(_coverageRadius.get(), base::AngleSegment{}, curPose.getPose());
            _coverage.write(coverage->getCoverage());
            previousPose = curPose;
        }
    }
    
    if(!poseValid)
    {
        state(ugv_nav4d::AreaExplorationBase::NO_POSE);
    }
    else if(!mapValid)
    {
        state(ugv_nav4d::AreaExplorationBase::NO_MAP);
    }
    else if(generateFrontiers)
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
             state(AREA_EXPLORED);
        }
        
        {
            envire::core::SpatioTemporal<maps::grid::TraversabilityBaseMap3d> trMap;
            trMap.frame_id = "AreaExploration";
            trMap.data = frontGen->getTraversabilityMap().copyCast<maps::grid::TraversabilityNodeBase *>();
//             _tr_map.write(trMap);
        }
        
        generateFrontiers = false;
        
        V3DD::FLUSH_DRAWINGS();
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
