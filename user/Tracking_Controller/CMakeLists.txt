
include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/include/Utilities")
include_directories("../../lcm_types/cpp")
include_directories("../")

add_executable(tc_ctrl TrackingController.cpp main.cpp )
target_link_libraries(tc_ctrl robot biomimetics dynacore_param_handler vwbc)

