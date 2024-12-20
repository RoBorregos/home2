# Home Navigation

Makefile contains the following commands:

nav.build: Builds the Docker image
nav.create: Creates a container based on the image previously built, attaches the ws folder, and sets up hostnames and IPs necessary for multimaster
nav.up: Starts the container
nav.down: Stops the container
nav.shell: Starts a shell session inside the container
nav.remove: Removes the container