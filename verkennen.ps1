# Robot laat zelf de ruimte verkennen (komt daarna vanzelf terug naar huis)
docker exec maqueen bash -c "source /opt/ros/jazzy/setup.bash && ros2 param set /maqueen_slam mode verkennen"
