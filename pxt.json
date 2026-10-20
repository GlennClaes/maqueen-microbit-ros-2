FROM ros:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-serial \
        python3-numpy \
        python3-scipy \
        ros-jazzy-tf2-ros \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-nav2-map-server \
    && rm -rf /var/lib/apt/lists/*

COPY ros/ /maqueen/
RUN sed -i 's/\r$//' /maqueen/start.sh && chmod +x /maqueen/start.sh \
    && echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

ENV PORT=socket://host.docker.internal:7777

CMD ["/maqueen/start.sh"]
