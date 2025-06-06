cmake_minimum_required(VERSION 3.30)
project(RigidDynamics)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

add_executable(RigidDynamics main.cc
        render.cc
        render.h
        camera.cc
        camera.h
        bodies/rigid_sphere.cc
        bodies/rigid_sphere.h
        bodies/drawable_rigid_body.cc
        bodies/drawable_rigid_body.h
        bodies/random_color_generator.cc
        bodies/random_color_generator.h
        bodies/body_manager.h
        bodies/body_manager.cc
        bodies/rigid_box.cc
        bodies/rigid_box.h
        bodies/rigid_body.cc
        bodies/rigid_body.h
        bodies/contacts/contact.cc
        bodies/contacts/contact.h
        bodies/contacts/body_interaction.cc
        bodies/contacts/body_interaction.h)

target_include_directories(RigidDynamics PRIVATE ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
target_link_libraries(RigidDynamics ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
