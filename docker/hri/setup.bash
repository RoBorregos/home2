# Create pulse audio socket to share with docker container
# If container doesn't start, delete /tmp/pulseaudio.socket and /tmp/pulseaudio.client.conf and run script again

# Ref: https://github.com/mviereck/x11docker/wiki/Container-sound:-ALSA-or-Pulseaudio

# Remove files if they exist
sudo rm -rf /tmp/pulseaudio.socket
sudo rm -rf /tmp/pulseaudio.client.conf

# Create pulseaudio socket.
pactl load-module module-native-protocol-unix socket=/tmp/pulseaudio.socket

# Create pulseaudio clients config.
echo 'default-server = unix:/tmp/pulseaudio.socket
# Prevent a server running in the container
autospawn = yes
daemon-binary = /bin/true
# Prevent the use of shared memory
enable-shm = false' > /tmp/pulseaudio.client.conf

sudo usermod -aG audio $USER # Make sure current user has access to audio resources.
sudo chmod 777 /dev/snd/* # Allow access to audio devices.

echo "Finished hri setup configuration for docker."
echo "Note: some of the docker compose files depend on the following environment variables. You may set them on the terminal running the docker compose command, in an .env file in the docker compose directory, or in your ~/.bashrc file:

- export LOCAL_USER_ID=\$(id -u)
- export LOCAL_GROUP_ID=\$(id -g)"
