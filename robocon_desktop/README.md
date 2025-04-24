# Robocon Desktop

## Creating a Project with Qt Creator

1. Open Qt Creator.
2. Select **New Project** -> **Qt Widgets Application**.
3. Reorganize the project as a ROS 2 package by moving the code to the `src` folder.
4. Move `CMakeLists.txt` to the root of the package folder.
5. Update `CMakeLists.txt` to include ROS 2 dependencies:
    ```cmake
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)

    install(TARGETS
      robocon_desktop
      DESTINATION lib/${PROJECT_NAME})
    ament_package()
    ```
