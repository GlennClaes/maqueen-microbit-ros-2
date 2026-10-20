# Schatting van de kamer: afmetingen, vrije oppervlakte, aantal objecten
docker exec maqueen bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo --once /maqueen/kamer"