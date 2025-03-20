cmake_minimum_required(VERSION 3.5)
project(mybot_description)

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
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
# add_library(mybot_description
#   src/${PROJECT_NAME}/mybot_description.cpp
# )

## Указываем зависимости для библиотеки
# ament_target_dependencies(mybot_description
#   rclcpp
#   urdf
#   xacro
#   tf2_ros
#   geometry_msgs
#   sensor_msgs
# )

## Объявляем C++ исполняемый файл (если нужно)
# add_executable(mybot_description_node src/mybot_description_node.cpp)

## Указываем зависимости для исполняемого файла
# ament_target_dependencies(mybot_description_node
#   rclcpp
#   urdf
#   xacro
#   tf2_ros
#   geometry_msgs
#   sensor_msgs
# )

## Указываем библиотеки для линковки
# target_link_libraries(mybot_description_node
#   ${rclcpp_LIBRARIES}
#   ${urdf_LIBRARIES}
#   ${xacro_LIBRARIES}
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
#   mybot_description
#   mybot_description_node
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
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

## Добавляем тесты (если нужно)
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test
