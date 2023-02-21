
include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/include/Utilities")
include_directories("../../common/include/Dynamics")
include_directories("../../lcm-types/cpp")
#set(CMAKE_CXX_FLAGS "-O0 -ggdb")

add_executable(imitation_ctrl Imitation_Controller.cpp main.cpp )
target_link_libraries(imitation_ctrl robot biomimetics dynacore_param_handler)

