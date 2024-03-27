
find_package(Boost REQUIRED)
find_library(QPOASES_LIB qpOASES)

if(NOT QPOASES_LIB)
  message(FATAL_ERROR "QPOASES library not found")
endif()

option(USE_PINOCCHIO "use pinocchio for VWBC" OFF)

include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/include/Utilities")
include_directories("../../lcm-types/cpp")
include_directories("include")

file(GLOB_RECURSE sources src/casadi_interface.cpp
                          src/HandC.cpp
                          src/MC_kinematics_casadi.cpp
                          src/VWBC.cpp) 
if(USE_PINOCCHIO)
  list(APPEND sources src/PinocchioInterface.cpp)
endif(USE_PINOCCHIO)

add_library(vwbc ${sources})     

target_link_libraries(vwbc ${QPOASES_LIB})

if(USE_PINOCCHIO)
  find_package(pinocchio REQUIRED)
  target_link_libraries(vwbc pinocchio::pinocchio)
  target_compile_definitions(vwbc USE_PINOCCHIO)
endif(USE_PINOCCHIO)


