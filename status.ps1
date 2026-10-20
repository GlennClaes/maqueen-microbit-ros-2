# Wat doet de robot nu? (Ctrl+C om te stoppen)
docker exec -it maqueen bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /maqueen/status"
