find_package(LibInit QUIET)
if(LIB_INIT_FOUND)
    init_module(
        SOURCES
            UGVAreaExploration.cpp
            UGVNav4d.cpp    
        HEADERS
            UGVAreaExploration.hpp
            UGVNav4d.hpp
        DEPS_PKGCONFIG
#            envire_maps-init
    )
endif()
