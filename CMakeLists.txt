cmake_minimum_required(VERSION 3.0.0)
project(haste VERSION 1.0.0) # Hypotheses-based Asynchronous Tracker of Events

# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/"
# ${CMAKE_MODULE_PATH})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -O3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(gflags REQUIRED)
find_package(glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} 
  INTERFACE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(tracking_app_file src/tracking_app_file.cpp)
target_link_libraries(tracking_app_file ${OpenCV_LIBS} gflags glog
                      Eigen3::Eigen)

set(GUI
    OFF
    CACHE BOOL "Builds HASTE GUI playground")

if(GUI)
  message("-DGUI=true (Default). GUI will be compiled.")

  add_subdirectory(thirdparty/quickgui)
  add_executable(tracking_app_gui
                 ${CMAKE_CURRENT_LIST_DIR}/src/tracking_app_gui.cpp)
  target_link_libraries(tracking_app_gui ${OpenCV_LIBS} gflags glog QuickGui)

else()
  message("-DGUI=false. No GUI will be compiled.")
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)