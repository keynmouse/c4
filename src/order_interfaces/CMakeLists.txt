cmake_minimum_required(VERSION 3.8)
project(order_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)  # 추가됨

# 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FoodOrder.msg"
  "msg/NewOrder.msg"
  "msg/CancelOrder.msg"
  "srv/OrderService.srv"
  DEPENDENCIES builtin_interfaces  # 의존성 추가
)

ament_package()