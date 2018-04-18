# DSController

cmake_minimum_required(VERSION 2.8)

find_package("ArmarXCore" REQUIRED)
find_package(Eigen3 QUIET)
find_package(Simox QUIET)
if (Simox_FOUND)
        setupSimoxExternalLibraries()
endif()

# Include provides all necessary ArmarX CMake macros
include(${ArmarXCore_USE_FILE})

set(ARMARX_ENABLE_DEPENDENCY_VERSION_CHECK_DEFAULT FALSE)
set(ARMARX_ENABLE_AUTO_CODE_FORMATTING TRUE)

# Name for the project
armarx_project("DSController")

# Specify each ArmarX Package dependency with the following macro
# depends_on_armarx_package(ArmarXGui "OPTIONAL")
depends_on_armarx_package(RobotAPI)
depends_on_armarx_package(Armar6RT)
#depends_on_armarx_package(ArmarXSimulation)
depends_on_armarx_package(ArmarXGui "OPTIONAL")


add_subdirectory(etc)
add_subdirectory(source)

install_project()
