
include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/include/Utilities")
include_directories("../../lcm_types/cpp")
include_directories("../")

add_executable(mhpc_llctrl MHPC_LLController.cpp main.cpp )
target_link_libraries(mhpc_llctrl robot biomimetics dynacore_param_handler vwbc)

