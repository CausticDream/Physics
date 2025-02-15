cmake_minimum_required(VERSION 3.8)

project(physics LANGUAGES CXX)

add_subdirectory(src)

option(PHYSICS_BUILD_SAMPLES "Build the samples" ON)

if (PHYSICS_BUILD_SAMPLES)

	add_subdirectory(extern/glad)
	add_subdirectory(extern/glfw)
	add_subdirectory(extern/imgui)
	add_subdirectory(samples)

	# default startup project for Visual Studio
	if (MSVC)
		set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT samples)
	endif()

endif()
