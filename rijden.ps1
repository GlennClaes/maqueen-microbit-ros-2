# Zelf besturen met het toetsenbord: i vooruit, , achteruit, j/l draaien, k stop
docker exec -it maqueen bash -c "source /opt/ros/jazzy/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=2.0"
