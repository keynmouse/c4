cmake_minimum_required(VERSION 3.8)
project(test3)

# 컴파일러 옵션 추가 (C++14 이상 권장)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

# 메시지와 서비스 생성 설정
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Order.srv"
)

# Python 패키지 설치
ament_python_install_package(${PROJECT_NAME})

# 실행 파일 설치 (필요한 경우)
install(PROGRAMS
  scripts/client
  scripts/server
  DESTINATION lib/${PROJECT_NAME}
)

# 의존성 추가
ament_export_dependencies(rosidl_default_runtime)

# 패키지 설정
ament_package()