cmake_minimum_required(VERSION 3.8)
project(object_position_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectPosition.msg"
)

ament_package()
