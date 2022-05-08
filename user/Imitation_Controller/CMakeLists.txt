
include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/include/Utilities")
include_directories("../../lcm-types/cpp")
include_directories("HKD-TrajOpt")
#set(CMAKE_CXX_FLAGS "-O0 -ggdb")
add_subdirectory(HKD-TrajOpt)

add_executable(imitation_ctrl Imitation_Controller.cpp main.cpp Imitation_Reference.cpp)
target_link_libraries(imitation_ctrl hkd-trajopt robot biomimetics dynacore_param_handler)

add_executable(mpc_solve mpc_main.cpp MPCSolver.cpp Imitation_Reference.cpp)
target_link_libraries(mpc_solve hkd-trajopt robot biomimetics dynacore_param_handler)