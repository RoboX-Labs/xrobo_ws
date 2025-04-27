# Robocon Desktop

## Creating a Project with Qt Creator

1. Open terminal and run:

   ```bash
   source /opt/ros/humble/setup.bash
   qtcreator
   ```

    This will source the ROS 2 environment and open Qt Creator.

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


## Problems and Solutions
1. [rviz_embed_test](https://github.com/mjeronimo/rviz_embed_test/blob/master/CMakeLists.txt)
2. [qt_gui_ros2](https://github.com/bandasaikrishna/qt_gui_ros2/blob/main/qt_gui_ros2/CMakeLists.txt)