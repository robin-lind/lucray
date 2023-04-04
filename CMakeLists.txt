cmake_minimum_required(VERSION 3.20.0)
project(lucray VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set (EXTERN_SRC_LIST
tinyexr/deps/miniz/miniz.c
glad/src/glad.c
)

set (SRC_LIST
main.cpp
implementation.cpp
scene.cpp
scene_light.cpp
filesystem.cpp
gltf.cpp
${EXTERN_SRC_LIST}
)

set (INCLUDE_LIST
tinyexr/deps/miniz
aixlog/include
glad/include
)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(lucray ${SRC_LIST})
target_include_directories(lucray PUBLIC ${INCLUDE_LIST})
target_link_libraries(lucray glfw OpenGL::GL)
