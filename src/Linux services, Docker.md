# Linux services, Docker

## Introduction to Linux services management

A variety of services run continuously on a Linux background, such as network and system services. Services running on Linux are also known as daemons, which refers to a group of processes working on the back-end.

The `systemd` is a software suite of tools adopted by most of the modern Linux distributions. All system tasks can be controlled through `systemd`. The process can be started or ended using this tool, and all enabled and disabled services information can also be listed with `systemd`.

### List services using `systemctl`

The easiest way to list services on Linux, when you are on a `systemd` system, is to use the `systemctl` command followed by `list-units`. You can specify the `--type=service` option in order to restrict the results to services only.

```bash
systemctl list-units --type=service
```

In order to list all services, both active and inactive, you have to use the “systemctl list-units” command followed by the “–all” option:

```bash
systemctl list-units --type=service --all
```

### Start a service

```bash
systemctl start [service_name]
```

### Stop a service

```bash
systemctl stop [service_name]
```

### Enable starting the service when the system boots

```bash
systemctl enable [service_name]
```

### Disable starting the service when the system boots

```bash
systemctl disable [service_name]
```

### Check service status

```bash
systemctl status [service_name]
```

### View logs produced by services

The logs produced by various services are saved by `systemd` via `journald` daemon. To see the logs that the `journald` daemon has collected, use the `journalctl` command.

When used alone, every journal entry that is in the system will be displayed within a pager (usually less) for you to browse. The oldest entries will be up top:

```bash
journalctl
```

You will likely have pages and pages of data to scroll through, which can be tens or hundreds of thousands of lines long if systemd has been on your system for a long while. This demonstrates how much data is available in the journal database.

### View logs from the current boot

Adding `-b` flag to the `journalctl` will filter the logs, showing only the data logged since last system reboot:

```bash
journalctl -b
```

### View logs produced by a specific service

```bash
journalctl -u [service_name]
```

## Creating own services in Linux

At times you create a script and then you want to have the scripts controlled by `systemd` or in some cases you wish to have the scripts getting restarted by itself when it is killed due to some reason. In such cases `systemd` in Linux helps to configure services which can be managed.

1. Create a file named `[service_name].service` in `/etc/systemd/system` using the following template:

    ```systemd
    [Unit]
    Description=<description about this service>

    [Service]
    User=<user e.g. root>
    WorkingDirectory=<directory_of_script e.g. /root>
    ExecStart=<script which needs to be executed>
    Restart=always

    [Install]
    WantedBy=multi-user.target
    ```

2. For Python specific projects which include virtual environment:

    ```systemd
    [Unit]
    Description=<project description>

    [Service]
    User=<user e.g. root>
    WorkingDirectory=<path to your project directory containing your python script>
    ExecStart=/home/user/.virtualenv/bin/python main.py
    Restart=always
    # replace /home/user/.virtualenv/bin/python with your virtualenv and main.py with your script

    [Install]
    WantedBy=multi-user.target
    ```

3. Reload the service files to include the new service:

    ```bash
    sudo systemctl daemon-reload
    ```

4. Start your service:

    ```bash
    sudo systemctl start your-service.service
    ```

***

#### 🛠🔥 Task 1 🛠🔥

Write a Python application that uses [click](https://click.palletsprojects.com/en/8.1.x/) library. The CLI (command line interface) of the application should accept a path to some file and time in seconds (let's call it `n` here). The application should write a random number, along with the current time to the given file every `n` seconds.

Write a service file that will run your application. Try adding couple of `print` statements in your code so you can check if the logs are collected properly. Enable and start your new service. Check the status and the logs. Try rebooting your system. The service should start automatically.

You may want to create a [virtual environment](https://docs.python.org/3.10/library/venv.html) (`venv`) and install the `click` library in it.

**Finish by stopping and disabling the service. Remove the service file from `/etc/systemd/system`**

***

## Docker

### Installation

*It may not be required in the lab.*

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
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
    ```

4. Set up stable Docker repository:

    ```bash
    echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
        $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    ```

The above steps and further documentation is also available [here](https://docs.docker.com/engine/install/ubuntu/).

### Docker commands

#### List Docker images

```bash
docker images
```

#### List all Docker containers

```bash
docker ps -a
```

#### Start Docker container

```bash
docker start <CONTAINER NAME>
```

#### Execute an interactive bash shell on the container

```bash
docker exec -it <CONTAINER NAME> bash
```

#### Copy file from host to container

```bash
docker cp path/to/file/on/host <CONTAINER NAME>:path/to/file/in/container
```

#### Copy file from container to host

```bash
docker cp <CONTAINER NAME>:path/to/file/in/container path/to/file/on/host
```

#### 🛠🔥 Task 2 🛠🔥

Read through [Docker's get started](https://docs.docker.com/get-started/) (10 parts in total). Contenerize the application from the previous task. The application should use bind mounts to write to a file located in the host file system.

- HINT 1: You can use [Python image](https://hub.docker.com/_/python) as a base for your app image.
- HINT 2: You can install required packages (`click` in this case) globally, using `pip` during the image building process.

Run the application and check the logs. Try enabling autostart with system boot for your container.

## Sources and resources

- <https://devconnected.com/how-to-list-services-on-linux/>
- <https://devconnected.com/how-to-list-services-on-linux/>
- <https://www.digitalocean.com/community/tutorials/how-to-use-journalctl-to-view-and-manipulate-systemd-logs>
- <https://www.shubhamdipt.com/blog/how-to-create-a-systemd-service-in-linux/>
