set(LIB_NAME       DSRTController)

armarx_component_set_name("${LIB_NAME}")
armarx_set_target("Library: ${LIB_NAME}")

find_package(Eigen3 QUIET)
find_package(Simox ${ArmarX_Simox_VERSION} QUIET)

find_package(MATHLIB QUIET)
armarx_build_if(Eigen3_FOUND "Eigen3 not available")
armarx_build_if(Simox_FOUND "Simox-VirtualRobot not available")

if (Eigen3_FOUND AND Simox_FOUND)
    include_directories(${Simox_INCLUDE_DIRS})
    include_directories(SYSTEM ${Eigen3_INCLUDE_DIR})
endif()

include_directories(${MATHLIB_INCLUDE_DIRS})


message(STATUS "mathlib is ${MATHLIB_LIBS}")
set(LIBS DSControllerInterfaces ArmarXCoreObservers ArmarXCoreStatechart ArmarXCoreEigen3Variants
    VirtualRobot
    Saba
    SimDynamics
    RobotUnit
    RobotAPIUnits
    RobotAPICore
    ${MATHLIB_LIB}
)



set(LIB_FILES
./DSRTController.cpp
./GMRDynamics.cpp
./Gaussians.cpp
#@TEMPLATE_LINE@@COMPONENT_PATH@/@COMPONENT_NAME@.cpp
)
set(LIB_HEADERS
./DSRTController.h
./GMRDynamics.h
./Gaussians.h
#@TEMPLATE_LINE@@COMPONENT_PATH@/@COMPONENT_NAME@.h
)


armarx_add_library("${LIB_NAME}" "${LIB_FILES}" "${LIB_HEADERS}" "${LIBS}")

# add unit tests
add_subdirectory(test)
