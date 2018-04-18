armarx_component_set_name("DSControllerGroup")

find_package(Eigen3 QUIET)
find_package(Simox QUIET)
armarx_build_if(Eigen3_FOUND "Eigen3 not available")
armarx_build_if(Simox_FOUND "Simox-VirtualRobot not available")

include_directories()

if (Eigen3_FOUND AND Simox_FOUND)
    include_directories(
        ${Eigen3_INCLUDE_DIR}
        ${Simox_INCLUDE_DIRS}
    )
endif()

set(COMPONENT_LIBS
    RobotAPIInterfaces
    RobotAPICore
    ArmarXCoreInterfaces
    ArmarXCore
    ArmarXCoreStatechart
    ArmarXCoreObservers
    ${jsoncpp_LIBRARIES}
    ${Simox_LIBRARIES}
    DSControllerInterfaces
)

set(SOURCES
DSControllerGroupRemoteStateOfferer.cpp
)

set(HEADERS
DSControllerGroupRemoteStateOfferer.h
DSControllerGroup.scgxml
)

# adds all existing state headers and sources to CMake
armarx_generate_statechart_cmake_lists()

armarx_add_component("${SOURCES}" "${HEADERS}")
