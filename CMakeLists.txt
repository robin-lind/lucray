cmake_minimum_required(VERSION 3.20.0)
project(lucray VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
enable_testing()

set (EXTERN_SRC_LIST
tinyexr/deps/miniz/miniz.c
)

set (SRC_LIST
main.cpp
implementation.cpp
camera.cpp
dacrt.cpp
${EXTERN_SRC_LIST}
)

set (INCLUDE_LIST
tinyexr/deps/miniz
aixlog/include
)

add_executable(lucray ${SRC_LIST})
target_include_directories(lucray PUBLIC ${INCLUDE_LIST})
target_link_libraries(lucray)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)