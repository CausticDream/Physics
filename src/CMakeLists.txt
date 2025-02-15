set(PHYSICS_SOURCE_FILES
	Arbiter.cpp
	Body.cpp
	Collide.cpp
	Joint.cpp
	World.cpp)

set(PHYSICS_HEADER_FILES
	../include/Arbiter.h
	../include/Body.h
	../include/Joint.h
	../include/World.h)

add_library(physics STATIC ${PHYSICS_SOURCE_FILES} ${PHYSICS_HEADER_FILES})
target_include_directories(physics PUBLIC ../include ../extern/glm)