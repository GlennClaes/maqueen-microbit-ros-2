# Robot terug naar het startpunt sturen (of druk A+B op de micro:bit)
docker exec maqueen bash -c "source /opt/ros/jazzy/setup.bash && ros2 param set /maqueen_slam mode naar_huis"
