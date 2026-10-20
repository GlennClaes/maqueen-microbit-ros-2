# Kaart opslaan in de map kaarten (kamer.pgm + kamer.yaml)
docker exec maqueen bash -c "source /opt/ros/jazzy/setup.bash && ros2 run nav2_map_server map_saver_cli -f /kaarten/kamer"
