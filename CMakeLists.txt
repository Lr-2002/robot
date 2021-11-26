cmake_minimum_required(VERSION 3.17)

set(CMAKE_CXX_STANDARD 14)
set(WEBOTS_HOME /home/lr-2002/application/webots)

get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)  # controller directory name

project(${PROJECT})

IF (WIN32)
    link_directories($ENV{WEBOTS_HOME}/msys64/mingw64/bin)
ELSE()
    link_directories(${WEBOTS_HOME}/lib/controller)
ENDIF()

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

if (NOT CPP_SOURCES STREQUAL "")  # Sources contain C++ files
    set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
    include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)
else()  # C
    set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX})
    include_directories(${WEBOTS_HOME}/include/controller/c)
endif()

add_executable(${PROJECT} ${SOURCES})

target_link_libraries(${PROJECT} ${LIBRARIES})

add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
        copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
        )
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

target_link_libraries(${PROJECT} Eigen3::Eigen)