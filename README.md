# MIRROR_OBSTACLE_DETECTION

- [CAD データ](https://workbench.grabcad.com/workbench/projects/gcxSH3HJgjFLzhEdpCU-hRgii7pXtuxlk3E0U46gWIwDdF#/space/gcip-oxmde-EwvW47NdzGdO0WVoQtpcRMA1btDJiVZNaXg)

- [STL モデル](https://github.com/maHidaka/mirror_obstacle_detection/blob/b723508b7b0683b292ef337a1da8b8ce57669b37/models/%E3%83%9F%E3%83%A9%E3%83%BC%E6%90%AD%E8%BC%89%E5%9E%8BURG.stl)

## 環境構築

```
sudo apt-get install ros-${ROS_DISTRO}-urg-node

sudo chmod a+rw /dev/ttyACM0

rosrun urg_node urg_node _serial_port:="/dev/ttyACM0"
```
