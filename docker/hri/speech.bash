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
