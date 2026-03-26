ARG BASE_IMAGE=roborregos/home2:l4t_base
FROM ${BASE_IMAGE}

COPY roudi_config.toml /etc/iceoryx/roudi_config.toml

CMD ["/opt/ros/humble/bin/iox-roudi", "-c", "/etc/iceoryx/roudi_config.toml"]
