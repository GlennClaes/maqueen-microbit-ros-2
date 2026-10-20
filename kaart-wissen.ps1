# Kaart leegmaken (robot stopt). Nieuw startpunt nodig? Gebruik: docker compose restart robot
docker exec maqueen bash -c "source /opt/ros/jazzy/setup.bash && ros2 param set /maqueen_slam mode reset_kaart"
