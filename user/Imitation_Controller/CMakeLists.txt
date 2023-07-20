
include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/include/Utilities")
include_directories("../../common/include/Dynamics")
include_directories("../../lcm-types/cpp")
include_directories(SYSTEM "../../third-party/qpOASES/include")
#set(CMAKE_CXX_FLAGS "-O0 -ggdb")

add_executable(imitation_ctrl Imitation_Controller.cpp main.cpp )
add_executable(imitation_ctrl_with_qp Imitation_Controller_With_QP.cpp main.cpp )
target_link_libraries(imitation_ctrl robot biomimetics dynacore_param_handler)
target_link_libraries(imitation_ctrl_with_qp robot biomimetics dynacore_param_handler qpOASES)

