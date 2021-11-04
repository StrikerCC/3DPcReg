cmake_minimum_required(VERSION 3.16)
project(3DPCREG VERSION 1.0.0)
#set(CMAKE_CXX_STANDARD 14)

IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
    MESSAGE(STATUS "current platform: Linux ")
    set(platform "linux")
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Windows")
    MESSAGE(STATUS "current platform: Windows")
    set(platform "windows")
ELSEIF (CMAKE_SYSTEM_NAME MATCHES "FreeBSD")
    MESSAGE(STATUS "current platform: FreeBSD")
ELSE ()
    MESSAGE(STATUS "other platform: ${CMAKE_SYSTEM_NAME}")
    FATAL_ERROR("unknow platform")
ENDIF (CMAKE_SYSTEM_NAME MATCHES "Linux")

# current project
include_directories(
        ${PROJECT_SOURCE_DIR}/include/
)

##################################### exe #####################################
add_executable(${PROJECT_NAME} main.cpp
#        src/Registration_mix.cpp
        src/utils.cpp
        #            src/PointCloudReader.cpp
        src/FeatureCompute.cpp
        #            src/SolveCorrespondenceAndRigidTransformation.cpp
        src/Registration.cpp
        #            src/Evaluation.cpp
        #            src/statistics.cpp
        src/CameraToOpen3d.cpp
        )

##################################### camera #####################################
set(RPATH "${PROJECT_SOURCE_DIR}/lib/${platform}")
include_directories(${PROJECT_SOURCE_DIR}/inc)
link_directories(${PROJECT_SOURCE_DIR}/lib/${platform})

##################################### torch lib #####################################
set(Torch_DIR /home/cheng/utils/libtorch-cxx11-abi-shared-with-deps-1.8.2+cu102/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

##################################### open3d #####################################
#set(Open3D_DIR /home/cheng_chen/source/open3d13/install_server/lib/cmake/Open3D)
set(Open3D_DIR /home/cheng/sources/open3d13/install_release/lib/cmake/Open3D)
find_package(Open3D)

##################################### c++ thread #####################################
find_package (Threads)


target_link_libraries(${PROJECT_NAME} optimized 3DCamera debug 3DCamera)
target_link_libraries(${PROJECT_NAME} ${TORCH_LIBRARIES})
if (Open3D_FOUND)
    message(STATUS "Found Open3D ${Open3D_VERSION}")
    message(STATUS "Open3D installed to: ${Open3D_LIBRARY_DIRS}")

    link_directories(${Open3D_LIBRARY_DIRS})
    include_directories(${Open3D_INCLUDE_DIRS})


    target_link_libraries(${PROJECT_NAME} ${Open3D_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PUBLIC ${Open3D_INCLUDE_DIRS})

    # target_link_libraries( ${Open3D_LIBRARIES})
endif()
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

#message("WTF${Open3D_INCLUDE_DIRS}")
#message("WTF${Open3D_LIBRARY_DIRS}")
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)

##################################### output bin #####################################
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
