cmake_minimum_required(VERSION 3.16)
project(3DPCREG VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)

set(Open3D_DIR /home/cheng/sources/open3ds/install/lib/cmake/Open3D)
# set(Open3D_DIR /home/cheng/sources/open3ds/install/)
find_package(Open3D REQUIRED)

if (Open3D_FOUND)
    message(STATUS "Found Open3D ${Open3D_VERSION}")
    message(STATUS "Open3D installed to: ${Open3D_LIBRARY_DIRS}")

    link_directories(${Open3D_LIBRARY_DIRS})
    include_directories(${Open3D_INCLUDE_DIRS})

    add_executable(3DPCREG main.cpp pc_reg.cpp utils.cpp utils.h)
 
    target_link_libraries(3DPCREG ${Open3D_LIBRARIES})
 
    target_include_directories(3DPCREG PUBLIC ${Open3D_INCLUDE_DIRS})

    # target_link_libraries( ${Open3D_LIBRARIES})

endif()


# target_link_libraries(3DPCREG Open3D::Open3D)

message("WTF${Open3D_INCLUDE_DIRS}")
message("WTF${Open3D_LIBRARY_DIRS}")