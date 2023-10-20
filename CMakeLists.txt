cmake_minimum_required (VERSION 3.8)

# Enable MSVC Hot Reload
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()
if (MSVC)
	set_target_properties(${BUILD_TARGET} PROPERTIES LINK_FLAGS "/PROFILE")
endif()

project ("MassiveDynamics")

# TODO make this configurable, add other platforms and whatnot
link_directories("../SDL/out/build/x64-min")

if(TRUE) # TODO make configurable
	# STATIC LINKING
	link_libraries(
	"version.lib"
	"imm32.lib"
	"winmm.lib"
	"setupapi.lib"
	"SDL3-static.lib"
	)
else()
	# DYNAMIC LINKING
	link_libraries(
	"SDL3.lib"
	)
endif()

add_executable (
	MassiveDynamics 
	"MassiveDynamics.cpp" 

	"Utils/CollisionUtils.cpp"
	"Utils/GeometryUtils.cpp"
	
	"PhysicalShapes/PolygonShape.cpp"

	"Collision/Separation.h"  
	"Collision/CollisionData.h" 

	"ArbiterPools.h"
)

include_directories("./"
# External dependencies
"../glm"
"../SDL/include" # Renderer only
)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET MassiveDynamics PROPERTY CXX_STANDARD 20)
endif()

