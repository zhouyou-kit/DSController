###
### CMakeLists.txt file for DSController Interfaces
###

# Dependencies on interface libraries to other ArmarX Packages must be specified
# in the following variable separated by whitespaces
set(DSController_INTERFACE_DEPEND ArmarXCore RobotAPI  ArmarXGui)

# List of slice files to include in the interface library
set(SLICE_FILES
    DSControllerBase.ice
)

# generate the interface library
armarx_interfaces_generate_library(DSController "${DSController_INTERFACE_DEPEND}")
