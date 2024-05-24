cmake_minimum_required (VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
project (Raytracing)

#aux_source_directory(${CMAKE_SOURCE_DIR}/src DIR_SRCS) 不会递归地搜索子目录
file(GLOB_RECURSE DIR_SRCS ${CMAKE_SOURCE_DIR}/src/*.cpp)
add_executable(Raytracing ${DIR_SRCS})

# eigen
add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE ${CMAKE_SOURCE_DIR}/cpp_lib/eigen-3.4.0)
target_link_libraries(Raytracing eigen)

# opencv
include_directories(${CMAKE_SOURCE_DIR}/cpp_lib/OpenCV/include)
file(GLOB OpenCV_LIBS ${CMAKE_SOURCE_DIR}/cpp_lib/OpenCV/x64/mingw/bin/*.dll)
target_link_libraries(Raytracing ${OpenCV_LIBS})

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} 
#      "${CMAKE_SOURCE_DIR}/cpp_lib/OpenCV/x64/mingw/lib" 
#      "${CMAKE_SOURCE_DIR}/cpp_lib/eigen-3.4.0/cmake")
# set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/cpp_lib/OpenCV/x64/mingw/lib)
# find_package(OpenCV REQUIRED)
# find_package(Eigen3 3.4.0)
# target_include_directories(Raytracing PUBLIC ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
# target_link_libraries(Raytracing ${OpenCV_LIBS})

#src
include_directories(${CMAKE_SOURCE_DIR}/src)
#Acceleration
include_directories(${CMAKE_SOURCE_DIR}/src/Acceleration)
#Materials
include_directories(${CMAKE_SOURCE_DIR}/src/Materials)
#Primitives
include_directories(${CMAKE_SOURCE_DIR}/src/Primitives)
#Mediums
include_directories(${CMAKE_SOURCE_DIR}/src/Mediums)

#修改可使用的栈大小
MATH(EXPR stack_size "1024 * 1024 * 1024")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--stack,${stack_size}")
