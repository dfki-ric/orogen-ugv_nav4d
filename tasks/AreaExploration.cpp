/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AreaExploration.hpp"
#include "ugv_nav4dTypes.hpp"
#include <ugv_nav4d/AreaExplorer.hpp>
#include <ugv_nav4d/FrontierGenerator.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <envire_core/items/SpatioTemporal.hpp>

#include <orocos_cpp/PkgConfigRegistry.hpp>

#include <maps/operations/CoverageMapGeneration.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

using namespace ugv_nav4d;

using maps::grid::TraversabilityNodeBase;
using maps::grid::TraversabilityBaseMap3d;
using envire::core::SpatioTemporal;

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
    if (!explorationMode) {
        generateFrontiers = true;
        this->area = area;
    }
}

void AreaExploration::startExploring()
{
    base::Vector3d center(0.0,0.0,0.0);
    base::Vector3d dimensions(20.0,20.0,10.0);
    base::Orientation orientation(1.0,0.0,0.0,0.0);
    area = OrientedBoxConfig{center, dimensions, orientation};

    std::cout << "Area set. Starting exploration mode ..." << std::endl;
    explorationMode = true;
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
        if(channel.find("area_explorer") != std::string::npos)
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
    
    //const std::vector<std::string> names{"ugv_nav4d"};
    //orocos_cpp::PkgConfigRegistryPtr pkgConfig = orocos_cpp::PkgConfigRegistry::initialize(names, false);
    //typeRegistry = orocos_cpp::TypeRegistry(pkgConfig);

    if (! AreaExplorationBase::configureHook())
        return false;
    return true;
}
bool AreaExploration::startHook()
{
    poseValid = false;
    mapValid = false;
    
    explorationMode = false;
    generateFrontiers = false;
    
    if (! AreaExplorationBase::startHook())
        return false;
    return true;
}
void AreaExploration::updateHook()
{
    AreaExplorationBase::updateHook();

    Eigen::Affine3d mls2Planner(Eigen::Translation3d(_gridOffset.rvalue()));

    if(_map.readNewest(map, false) == RTT::NewData)
    {
        mapValid = true;
        map.data.translate(_gridOffset.rvalue());
        frontGen->updateMap(map.data, coverage ? &coverage->getCoverage() : nullptr);
        if(coverage)
            coverage->setFrame(frontGen->getTraversabilityMap());
    }

    if(_pose_samples.readNewest(curPose, false) == RTT::NewData)
    {
        // translate curPose
        curPose.setTransform(mls2Planner * curPose.getTransform());

        if(!poseValid)
        {
            std::cout << "Setting initial patch for area explorer" << std::endl;
            explorer->setInitialPatch(curPose.getTransform(), _initialPatchRadius.get());
            previousPose = curPose;
        }
        poseValid = true;
        
        if(coverage && mapValid && (curPose.position - previousPose.position).norm() > 
        _coverageUpdateDistance.get())
        {
            std::cout << "Adding coverage at " << curPose.position.transpose() << '\n';
            // TODO AngleSegment and orientation are ignored at the moment
            coverage->addCoverage(_coverageRadius.get(), base::AngleSegment{}, curPose.getPose());
            _coverage.write(coverage->getCoverage());
            previousPose = curPose;
        }
    }

    boost::int32_t planner_state;
    if (_planner_state.readNewest(planner_state, false == RTT::NewData)) {
        if (planner_state == 0 && state() == EXPLORING) { // BAG_GOAL
            if (currentGoals.size() > 1) { // there is an other goal than the latest.
                std::cout << "Planner says that current best goal is invalid. Writing next best goal on output." << std::endl;
                currentGoals.erase(currentGoals.begin(), currentGoals.begin() + 1); // erase latestBestGoal
                latestBestGoal = currentGoals.front();
                _goal_out_best.write(latestBestGoal);
            } else {
                std::cout << "There are no other goals. Stopping AreaExploration" << std::endl;
                generateFrontiers = false;
                explorationMode = false;
            }
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
    else {
        if (!explorationMode && !generateFrontiers) 
        {
            state(GOT_MAP_AND_POSE);
        }

        if(explorationMode) {           
            if (state() == EXPLORING && (curPose.position - latestBestGoal.position).norm() < _goalReachedThresholdDistance.get())
            {
                std::cout << "Goal was reached. Starting to compute frontiers and select new goal." << std::endl;
                // goal was reached
                generateFrontiers = true;
            } else if (state() == GOT_MAP_AND_POSE) {
                std::cout << "Starting to compute frontierts and select first goal." << std::endl;
                generateFrontiers = true;
            }
        }

        if(generateFrontiers) {
            state(PLANNING);
            
            std::vector<base::samples::RigidBodyState> outFrontiers;
            if(explorer->getFrontiers(curPose.position, area, outFrontiers))
            {
                state(GOALS_GENERATED);
                
                for(auto &f : outFrontiers)
                {
                    //convert to ground frame
                    f.position +=  f.orientation * Eigen::Vector3d(0,0, -_travConfig.get().distToGround);

                    f.setTransform(mls2Planner.inverse() * f.getTransform());
                }
                
                _goals_out.write(outFrontiers);

                if (explorationMode) {
                    currentGoals = outFrontiers;
                    latestBestGoal = currentGoals.front();
                    _goal_out_best.write(latestBestGoal);
                    state(EXPLORING);
                }
            }
            else
            {
                std::cout << "area explored." << std::endl;
                state(AREA_EXPLORED);
                if (explorationMode) {
                    explorationMode = false;
                }
            }
            
            std::cout << "Write updated travMap to output." << std::endl;
            SpatioTemporal<TraversabilityBaseMap3d> st(frontGen->getTraversabilityMap().copyCast<TraversabilityNodeBase*>());
            // planner has static transformation to world by [10, 10, 0]
            st.setFrameID("planner");
            _tr_map.write(st);
                        
            generateFrontiers = false;
            
            V3DD::FLUSH_DRAWINGS();
        }

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
