cmake_minimum_required (VERSION 3.12)
project ("MassiveDynamics")

# EXECUTABLES
add_executable (MassiveDynamics "MassiveDynamics.ixx"

# TODO DEBUG GUARD
"Interfaces/Debugging/IMxDebugHelper.ixx"
"Interfaces/Debugging/IMxLogger.ixx"
"Implementations/Debugging/MxDebugHelper.ixx"
"Implementations/Debugging/MxConsoleLogger.ixx"

"Interfaces/IMxPairGenerator.ixx"

"Implementations/MxScene.ixx" 

"MxBuffer.ixx" 
"MxBodyPool.ixx" 
"MxSolver.ixx" 
)
include_directories("Include")
set_property(TARGET MassiveDynamics PROPERTY CXX_STANDARD 20)

# DEPENDENCIES
include_directories("Dependencies")

# TESTS AND INSTALL TARGETS
