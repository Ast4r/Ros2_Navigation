cmake_minimum_required(VERSION 3.8)
project(mybot_gazebo)

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(turtlebot3_description REQUIRED)  # Если вы используете TurtleBot3

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Указываем, что пакет использует ament_cmake
ament_package()

###########
## Build ##
###########

## Указываем дополнительные каталоги для заголовочных файлов
# include_directories(include)

## Объявляем C++ библиотеку (если нужно)
# add_library(mybot_gazebo
#   src/${PROJECT_NAME}/mybot_gazebo.cpp
# )

## Указываем зависимости для библиотеки
# ament_target_dependencies(mybot_gazebo
#   rclcpp
#   gazebo_ros
# )

## Объявляем C++ исполняемый файл (если нужно)
# add_executable(mybot_gazebo_node src/mybot_gazebo_node.cpp)

## Указываем зависимости для исполняемого файла
# ament_target_dependencies(mybot_gazebo_node
#   rclcpp
#   gazebo_ros
# )

## Указываем библиотеки для линковки
# target_link_libraries(mybot_gazebo_node
#   ${rclcpp_LIBRARIES}
#   ${gazebo_ros_LIBRARIES}
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
#   mybot_gazebo
#   mybot_gazebo_node
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
  worlds
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
