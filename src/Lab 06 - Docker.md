# Docker

## Installation

*May not be required in the laboratory.*

1. Remove older versions of Docker:

    ```bash
    sudo apt remove docker docker-engine docker.io containerd runc
    ```

2. Update and install required packages:

    ```bash
    sudo apt update

    sudo apt install ca-certificates curl gnupg lsb-release
    ```

3. Add Docker’s official GPG key:

    ```bash
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc
    ```

4. Set up the stable Docker repository:

    ```bash
    echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
    $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update
    ```

5. Install Docker Engine, CLI, and Containerd:

    ```bash
    sudo apt update
    sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ```

6. Verify that Docker Engine is installed correctly by running the `hello-world` image:

    ```bash
        sudo docker run hello-world
    ```

7. (Optional) To run Docker commands without `sudo`, create the `docker` group and add your user to it:

    ```bash
    sudo groupadd docker
    sudo usermod -aG docker $USER
    newgrp docker   
    ```

    You can test if the above step was successful by running:

    ```bash
    docker run hello-world
    ```

    ^ Notice that you don't need `sudo` in front of the command.

The above steps and further documentation are also available [here](https://docs.docker.com/engine/install/ubuntu/).

## Docker Commands

### List Docker images

```bash
docker images
```

### List all Docker containers

```bash
docker ps -a
```

### Start a container

```bash
docker start <container_name>
```

### Run an interactive `bash` shell inside a container

```bash
docker exec -it <container_name> bash
```

### Create a new container and run an interactive `bash` shell inside it

```bash
docker run -it <image_name> bash
```

### Copy a file from the host to a container

```bash
docker cp path/to/file/on/host <container_name>:path/to/file/in/container
```

### Copy a file from a container to the host

```bash
docker cp <container_name>:path/to/file/in/container path/to/file/on/host
```

### List volumes

```bash
docker volume ls
```

### Create a volume

```bash
docker volume create my_volume
```

### Remove a volume

```bash
docker volume rm my_volume
```

---

### 🛠🔥 Task 1 🛠🔥

Read [Docker's get started](https://docs.docker.com/get-started/workshop/) (10 parts in total).

### 🛠🔥 Task 2 🛠🔥

Containerize the application from the previous class. The application should use [bind mounts](https://docs.docker.com/engine/storage/bind-mounts/) to write to a file located in the host’s filesystem.

- Hint 1: You can use the [Python image](https://hub.docker.com/_/python) as a base for your application image.
- Hint 2: You can install required packages (in this case `click`) globally with `pip` during the image build process.

Run the application and check the logs. Try enabling autostart of your container with system startup.

### 🛠🔥 Task 3 🛠🔥

Instead of mounting a host directory, you can use a [Docker volume](https://docs.docker.com/engine/storage/volumes/). Perform the following steps:

- Create a volume named `my_volume`.
- Create and run a container with the volume attached using:  
  `docker run -it --name my_container -v my_volume:/data ubuntu`
- Create and save a file inside the container (e.g., `echo "Hello from Docker!" > /data/hello.txt`).
- Exit the container with the `exit` command.
- Remove the container with:  
  `docker rm my_container`.

Think about whether it is still possible to read the text file you created after removing the container.
<details>
<summary><b>After thinking, check in the console</b></summary>
The following command will automatically remove the container after running and reading the text file:

`docker run --rm -v my_volume:/data ubuntu cat /data/hello.txt`

</details>