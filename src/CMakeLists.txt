set(BOX2D_SOURCE_FILES
	Arbiter.cpp
	Arbiter3D.cpp
	Body.cpp
	Body3D.cpp
	Collide.cpp
	Joint.cpp
	Joint3D.cpp
	World.cpp
	World3D.cpp)

set(BOX2D_HEADER_FILES
	../include/box2d-lite/Arbiter.h
	../include/box2d-lite/Arbiter3D.h
	../include/box2d-lite/Body.h
	../include/box2d-lite/Body3D.h
	../include/box2d-lite/Joint.h
	../include/box2d-lite/Joint3D.h
	../include/box2d-lite/MathUtils.h
	../include/box2d-lite/World.h
	../include/box2d-lite/World3D.h)

add_library(box2d-lite STATIC ${BOX2D_SOURCE_FILES} ${BOX2D_HEADER_FILES})
target_include_directories(box2d-lite PUBLIC ../include)