cmake_minimum_required(VERSION 3.4)
set(CMAKE_OSX_ARCHITECTURES "x86_64" CACHE STRING "") #Apple Silicon
project(test)
list(APPEND CMAKE_FRAMEWORK_PATH /Library/Frameworks) #Mac, why not auto?
find_package(Imagine REQUIRED)
find_package( OpenCV REQUIRED )
include_directories(/usr/local/include/opencv4)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(test test.cpp)
target_link_libraries( test ${OpenCV_LIBS} )
ImagineUseModules(test LinAlg Images)