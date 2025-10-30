# Linux Services

## Introduction to Managing Services in Linux

Many services run continuously in the background on a Linux system, including network and system services. Services running on Linux are also known as daemons.

`systemd` is a suite of tools used by most modern Linux distributions. All system tasks can be controlled through `systemd`. You can start or stop processes with this tool and display all enabled and disabled services along with their details.

### Listing services with `systemctl`

The simplest way to list services on a system that uses `systemd` is to run the `systemctl` command followed by `list-units`. You can add the `--type=service` option to restrict the results to services only.

```bash
systemctl list-units --type=service
```

To display all services, both active and inactive, use the `--all` option:

```bash
systemctl list-units --type=service --all
```

### Starting a service

```bash
systemctl start [service_name]
```

### Stopping a service

```bash
systemctl stop [service_name]
```

### Enabling a service at boot

```bash
systemctl enable [service_name]
```

### Disabling a service at boot

```bash
systemctl disable [service_name]
```

### Checking service status

```bash
systemctl status [service_name]
```

### Viewing logs produced by services

The logs produced by various services are stored by the `journald` daemon. To view the logs that `journald` has collected, use the `journalctl` command.

When the command is used on its own, every journal entry is displayed inside a pager (usually `less`). The oldest entries appear first:

```bash
journalctl
```

The journal may contain a significant number of messages—tens or even hundreds of thousands of lines—which shows how much information is stored in the database.

### Showing logs from the current boot

Adding the `-b` flag to `journalctl` filters the logs so that only entries recorded since the last system reboot are shown:

```bash
journalctl -b
```

### Viewing logs produced by a specific service

```bash
journalctl -u [service_name]
```

## Creating Your Own Services in Linux

Sometimes you want to let `systemd` manage your application. In some cases it is useful to restart a script automatically if it stops for any reason. `systemd` is a good solution for these scenarios.

1. Create a file named `[service_name].service` in `/etc/systemd/system` using the following template:

    ```systemd
    [Unit]
    Description=Service description

    [Service]
    User=username (for example, root)
    ExecStart=/absolute/path/to/script (for example, /home/student/Desktop/script.sh)
    Restart=always

    [Install]
    WantedBy=multi-user.target
    ```

2. For Python projects that use a virtual environment, you can use:

    ```systemd
    [Unit]
    Description=Service description

    [Service]
    User=username (for example, root)
    ExecStart=/home/user/virtualenv/bin/python main.py
    Restart=always
    # replace /home/user/virtualenv/bin/python with your virtualenv interpreter and main.py with your script

    [Install]
    WantedBy=multi-user.target
    ```

3. Make your script executable:

    ```bash
    chmod +x script_name
    ```

4. Reload the service files to include the new service:

    ```bash
    sudo systemctl daemon-reload
    ```

5. Start your service:

    ```bash
    sudo systemctl start your-service.service
    ```

***

Example service and script:

`my_service.service`:

```systemd
[Unit]
Description=Service description

[Service]
User=jakub
ExecStart=/usr/bin/python3 /home/jakub/Desktop/print_messages.py
Restart=always

[Install]
WantedBy=multi-user.target
```

`print_messages.py`:

```python
#!/usr/bin/env python3
import time

if __name__ == "__main__":
    while True:
        print("Hello world")
        time.sleep(1)
```

or for Bash:

```bash
#!/bin/bash
while $(sleep 1);
do
    echo "hello world"
done
```

When building your own script, make sure to include a shebang line (the first line in the examples above).

***

#### 🛠🔥 Task 1 🛠🔥
Create a Python script named `print_messages.py`. The file contents were shown earlier in this guide. Run the script from two different locations in the terminal:

- the directory where the script resides
- the `/etc/systemd/system` directory

#### 🛠🔥 Task 2 🛠🔥
Create a `my_service.service` unit file. The file contents were shown earlier in this guide. Start the service.

#### 🛠🔥 Task 3 🛠🔥
Write a Python application that uses the [click](https://click.palletsprojects.com/en/8.1.x/) library. The application's CLI should accept a path to a file and an interval in seconds (call it `n`). The application should append a random number along with the current time to the file every `n` seconds.

Create a service file that runs your application. Add a few `print` statements to your code so that you can verify that the logs are collected correctly. Enable and start your new service. Check its status and logs. Restart your system; the service should start automatically.

You may want to create a [virtual environment](https://docs.python.org/3.10/library/venv.html) (`venv`) and install the `click` library inside it.

**Finish the task by stopping and disabling the service. Remove the service file from `/etc/systemd/system`. Save your code - you will need it in future labs.**

***

## Sources

- https://devconnected.com/how-to-list-services-on-linux/
- https://www.digitalocean.com/community/tutorials/how-to-use-journalctl-to-view-and-manipulate-systemd-logs
- https://www.shubhamdipt.com/blog/how-to-create-a-systemd-service-in-linux/
