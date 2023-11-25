
find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED)
find_library(QPOASES_LIB 
             NAMES  libqpOASES.a 
             PATHS $ENV{QPOASES_LIB_PATH})

if(NOT QPOASES_LIB)
  message(FATAL_ERROR "QPOASES library not found")
endif()

file(GLOB_RECURSE sources "src/*.cpp") 

include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/include/Utilities")
include_directories("../../lcm-types/cpp")
include_directories("include")

add_library(vwbc ${sources})     

target_include_directories(vwbc PUBLIC $ENV{QPOASES_INCLUDE})

target_link_libraries(vwbc ${QPOASES_LIB} pinocchio::pinocchio)

