/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#pragma once

#include "ugv_nav4d/MapLoaderBase.hpp"
#include <maps/grid/MLSMap.hpp>

namespace ugv_nav4d{


    class MapLoader : public MapLoaderBase
    {
    friend class MapLoaderBase;
    
    private:
        envire::core::SpatioTemporal<maps::grid::MLSMapKalman> map;
        bool mapLoaded;
        bool sendMap; //if true map will be published on next cycle
    
    protected:

        /* trigger writing of the map to the port
         */
        virtual void publishMap();
        virtual bool loadMls(const std::string& path);

    public:

        MapLoader(std::string const& name = "ugv_nav4d::MapLoader", TaskCore::TaskState initial_state = Stopped);
        ~MapLoader();

        
        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}
