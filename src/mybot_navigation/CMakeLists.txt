cmake_minimum_required(VERSION 3.5)
project(mybot_navigation)

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Указываем, что пакет использует ament_cmake
ament_package()

###########
## Build ##
###########

## Указываем дополнительные каталоги для заголовочных файлов
# include_directories(include)

## Объявляем C++ библиотеку (если нужно)
# add_library(mybot_navigation
#   src/${PROJECT_NAME}/mybot_navigation.cpp
# )

## Указываем зависимости для библиотеки
# ament_target_dependencies(mybot_navigation
#   rclcpp
#   nav2_common
#   nav2_msgs
#   nav2_util
#   nav2_costmap_2d
#   nav2_controller
#   nav2_planner
#   nav2_recoveries
#   tf2_ros
#   geometry_msgs
#   sensor_msgs
# )

## Объявляем C++ исполняемый файл (если нужно)
# add_executable(mybot_navigation_node src/mybot_navigation_node.cpp)

## Указываем зависимости для исполняемого файла
# ament_target_dependencies(mybot_navigation_node
#   rclcpp
#   nav2_common
#   nav2_msgs
#   nav2_util
#   nav2_costmap_2d
#   nav2_controller
#   nav2_planner
#   nav2_recoveries
#   tf2_ros
#   geometry_msgs
#   sensor_msgs
# )

## Указываем библиотеки для линковки
# target_link_libraries(mybot_navigation_node
#   ${rclcpp_LIBRARIES}
#   ${nav2_common_LIBRARIES}
#   ${nav2_msgs_LIBRARIES}
#   ${nav2_util_LIBRARIES}
#   ${nav2_costmap_2d_LIBRARIES}
#   ${nav2_controller_LIBRARIES}
#   ${nav2_planner_LIBRARIES}
#   ${nav2_recoveries_LIBRARIES}
#   ${tf2_ros_LIBRARIES}
#   ${geometry_msgs_LIBRARIES}
#   ${sensor_msgs_LIBRARIES}
# )

#############
## Install ##
#############

## Устанавливаем исполняемые файлы (Python и C++)
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION lib/${PROJECT_NAME}
# )

## Устанавливаем C++ исполняемые файлы и библиотеки
# install(TARGETS
#   mybot_navigation
#   mybot_navigation_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

## Устанавливаем заголовочные файлы
# install(DIRECTORY include/
#   DESTINATION include
# )

## Устанавливаем launch-файлы и другие ресурсы
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

## Добавляем тесты (если нужно)
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()
