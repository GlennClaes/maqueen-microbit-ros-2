# Robot stoppen (of druk op knop A op de micro:bit)
docker exec maqueen bash -c "source /opt/ros/jazzy/setup.bash && ros2 param set /maqueen_slam mode stop"
