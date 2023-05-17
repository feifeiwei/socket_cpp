cmake_minimum_required(VERSION 2.8)

project(run_inferface_test)

set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++14 -pthread")


find_package(OpenCV REQUIRED)

find_package(Protobuf REQUIRED)

message(STATUS "OpenCV libaray information:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    header file: ${OpenCV_INCLUDE_DIRS}")


include_directories(
	${PROJECT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include
    )


add_executable(run_socket  src/main.cpp
                        src/table_msgs.cpp
                    )

target_link_libraries(run_socket  ${OpenCV_LIBS} )


