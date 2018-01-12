project(sire_raytracer_td1)

cmake_minimum_required(VERSION 2.8)

# This file included below takes care of compiling dependencies
# and setting compiler flags on different platforms
include("CMakeConfig.txt")

add_definitions(-DSIRE_DIR="${PROJECT_SOURCE_DIR}")
add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)

# the list of source files
set(sire_SRCS
    include/common.h
    include/vector.h
    include/block.h
    include/renderingWidget.h
    include/bitmap.h
    include/camera.h
    include/trackball.h
    include/ray.h
    include/object.h
    include/sphere.h
    include/scene.h
    include/mesh.h
    include/shape.h
    include/material.h
    include/light.h
    include/plane.h
    include/glPrimitives.h
    include/sphere.h
    include/color.h
    include/parser.h
    include/proplist.h
    include/transform.h
    include/integrator.h

    src/common.cpp
    src/block.cpp
    src/renderingWidget.cpp
    src/bitmap.cpp
    src/mesh.cpp
    src/camera.cpp
    src/trackball.cpp
    src/main.cpp
    src/object.cpp
    src/shape.cpp
    src/sphere.cpp
    src/scene.cpp
    src/material.cpp
    src/directionalLight.cpp
    src/pointLight.cpp
    src/plane.cpp
    src/phong.cpp
    src/parser.cpp
    src/proplist.cpp
    src/flat.cpp
)

add_executable(sire_raytracer ${sire_SRCS})

add_dependencies(sire_raytracer OpenEXR_p)
add_dependencies(sire_raytracer nanogui_p)
add_dependencies(sire_raytracer lib3ds_p)
add_dependencies(sire_raytracer pugixml)

target_link_libraries(sire_raytracer ${extra_libs})
