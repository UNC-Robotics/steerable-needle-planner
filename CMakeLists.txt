cmake_minimum_required(VERSION 3.8)

project(snp)

# Build type.
set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)

    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")

    # On 32bit architectures, use RelWithDebInfo
    if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Type of build" FORCE)
    endif()
endif()

message(STATUS "Building ${CMAKE_BUILD_TYPE}")

# Use C++ 17 for nign and mpt.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 17)
set(USE_C++17 1)

# OpenMp
OPTION (USE_OpenMP "Use OpenMP to enable <omp.h>" ON)

# Find OpenMP
if(APPLE AND USE_OpenMP)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang")
        set(OpenMP_C "${CMAKE_C_COMPILER}")
        set(OpenMP_C_FLAGS "-fopenmp=libomp -Wno-unused-command-line-argument")
        set(OpenMP_C_LIB_NAMES "libomp" "libgomp" "libiomp5")
        set(OpenMP_libomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libgomp_LIBRARY ${OpenMP_C_LIB_NAMES})
        set(OpenMP_libiomp5_LIBRARY ${OpenMP_C_LIB_NAMES})
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
      set(OpenMP_CXX_FLAGS "-fopenmp=libomp -Wno-unused-command-line-argument")
      set(OpenMP_CXX_LIB_NAMES "libomp" "libgomp" "libiomp5")
      set(OpenMP_libomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
      set(OpenMP_libgomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})
      set(OpenMP_libiomp5_LIBRARY ${OpenMP_CXX_LIB_NAMES})
    endif()
endif()

if(USE_OpenMP)
    message("OpenMP is required!")
    find_package(OpenMP REQUIRED)
endif(USE_OpenMP)

if (OPENMP_FOUND)
    message("OpenMP is found!")
    # include_directories("${OPENMP_INCLUDES}")
    # link_directories("${OPENMP_LIBRARIES}")
if(NOT APPLE)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
    # set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)


# Check for eigen3
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
    set(EIGEN3_INCLUDE_DIR "EIGEN3-NOTFOUND" CACHE PATH "Location of EIGEN3 headers")
endif()

if(EIGEN3_INCLUDE_DIR)
    include_directories(${EIGEN3_INCLUDE_DIR})
    message("Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
else(EIGEN3_INCLUDE_DIR)
    message("Eigen3 not found! Please set it manually!")
endif()


# Check for Boost
find_package(Boost 1.56.0 COMPONENTS system program_options timer chrono thread serialization date_time filesystem regex)
include_directories(${Boost_INCLUDE_DIRS})
message("Boost headers: ${Boost_INCLUDE_DIRS}")
message("Boost libraries: ${Boost_LIBRARIES}")


include_directories(BEFORE "include")
include_directories(BEFORE "${CMAKE_CURRENT_BINARY_DIR}/include")

include_directories(BEFORE "external/nigh/src")
include_directories(BEFORE "${CMAKE_CURRENT_BINARY_DIR}/external/nigh/src")

include_directories(BEFORE "external/mpt/src")
include_directories(BEFORE "${CMAKE_CURRENT_BINARY_DIR}/external/mpt/src")

add_subdirectory(app)
