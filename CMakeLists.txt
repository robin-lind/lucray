cmake_minimum_required(VERSION 3.20.0)
project(lucray VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set (EXTERN_SRC_LIST
tinyexr/deps/miniz/miniz.c
)

set (SRC_LIST
main.cpp
implementation.cpp
filesystem.cpp
scene.cpp
${EXTERN_SRC_LIST}
)

set (INCLUDE_LIST
tinyexr/deps/miniz
aixlog/include
)

add_subdirectory("raylib")

add_executable(lucray ${SRC_LIST})
target_include_directories(lucray PUBLIC ${INCLUDE_LIST})
target_link_libraries(lucray raylib)
