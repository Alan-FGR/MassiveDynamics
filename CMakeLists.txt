cmake_minimum_required (VERSION 3.12)

# Enable Hot Reload if supported.
#if (POLICY CMP0141)
#  cmake_policy(SET CMP0141 NEW)
#  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>")
#endif()

project ("MassiveDynamicsRefresh")

# SOURCES
add_executable (MassiveDynamicsRefresh 
"Program.ixx" 
"DebugRenderer.ixx"
"Transform.ixx")

set_property(TARGET MassiveDynamicsRefresh PROPERTY CXX_STANDARD 20)

# INCLUDES
include_directories("Include")
include_directories("Include/Sdl2")

#add_subdirectory("Dependencies/glm")

# LINKING
target_link_libraries(MassiveDynamicsRefresh "${PROJECT_SOURCE_DIR}/Libraries/Sdl2/x64/SDL2.lib")

# COPYING DEPENDENCIES
add_custom_command(TARGET MassiveDynamicsRefresh POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PROJECT_SOURCE_DIR}/Libraries/Sdl2/x64/SDL2.dll"
        $<TARGET_FILE_DIR:MassiveDynamicsRefresh>)