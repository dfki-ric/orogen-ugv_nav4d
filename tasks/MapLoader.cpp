/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MapLoader.hpp"

using namespace ugv_nav4d;

MapLoader::MapLoader(std::string const& name, TaskCore::TaskState initial_state)
    : MapLoaderBase(name, initial_state)
{
}

MapLoader::~MapLoader()
{
}

void MapLoader::publishMap()
{

}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MapLoader.hpp for more detailed
// documentation about them.

bool MapLoader::configureHook()
{
    if (! MapLoaderBase::configureHook())
        return false;
    return true;
}
bool MapLoader::startHook()
{
    if (! MapLoaderBase::startHook())
        return false;
    return true;
}
void MapLoader::updateHook()
{
    MapLoaderBase::updateHook();
}
void MapLoader::errorHook()
{
    MapLoaderBase::errorHook();
}
void MapLoader::stopHook()
{
    MapLoaderBase::stopHook();
}
void MapLoader::cleanupHook()
{
    MapLoaderBase::cleanupHook();
}
