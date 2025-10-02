# An extended introduction to Linux

## A reminder of basic work conveniences

As part of working with the terminal, it is useful to recall some useful "tricks":

* When typing commands, access paths, etc., pressing the `Tab` key, the terminal will complete the next part of the command if there is only one possible completion. If there are several possibilities, pressing `Tab` twice will display the possible complements.
* By pressing the up and down arrow keys while working with the command line, we can easily move between the commands issued in this session.
* Wider access to the command history can be obtained by pressing `Ctrl+R` and searching for the relevant command.
* The `clear` command clears the terminal screen.
* Typing `!!` automatically replaces the last command issued. For example, if you forget to execute a complicated command with the privileges of user *root*, just type `sudo !!` in the next step.
* To issue several commands on one line, we can separate successive commands with `&&` or `;` characters. In the first case, subsequent commands will execute only if the previous ones do not end in an error. Separation by semicolon causes sequential execution of commands without paying attention to the result of the operation.

 ***

### System help

In Linux, system help is available for each command, describing various aspects and uses of the tool and available switches. One mechanism for displaying help files is to call a command with the `--help` switch, such as:

```bash
ls --help
```

causes a description of the operation of all switches, references to external documentation and other useful information to be displayed.

More extensive command documentation can be obtained by calling:

```bash
man command_name
```

This launches an interactive help file browser for the specified program or service.
The system help is displayed using the *more* viewer, which is operated with the following keyboard commands:

* `Space` - move to the next page;
* `Ctrl+B` - move to the previous page;
* `q` - closing and exiting the browser;
* `/` - forward text search, after the / sign enter the text to be searched;
* `?` - search backwards;
* `n`, `N` - move to the next (n) or previous (N) occurrences of the search term.

### Network information

The `ip` command allows you to obtain information about the current status of network interfaces (including the computer's IP address).

***

#### 🛠🔥 Task 1 🛠🔥

Familiarize yourself with the help for the `ip` command. Using especially the "EXAMPLES" section, display the current addresses of the computer's network interfaces.

***

## File handling

Basic file handling operations can be performed using the following commands:

* `cp [switches] file_name new_name_or_directory` - copy the file specified by the first argument to the name or to the directory specified by the second argument, for example:
* `cp abc.txt xyz.txt` - copies the file `abc.txt` under the new name `xyz.txt` in the current directory;
* `cp /tmp/abc.txt ~` - copies the `abc.txt` file from the `/tmp` directory to the user's home directory;
* `cp abc.txt ~/xyz.txt` - copies the `abc.txt` file from the current directory under the new name `xyz.txt` in the user's home directory.
  
A useful switch for the `cp` command is the `-r` switch, which is used to copy entire directory structures.

* `rm [switches] list_files` - deletes files given as arguments to the call, e.g.:
* `rm abc.txt xyz.txt` - removes files `abc.txt` and `xyz.txt` in the current directory.
* `rm /tmp/abc.txt` - removes the `abc.txt` file from the `/tmp` directory.

A useful switch for the `rm` command is the -r switch, which is used to delete entire directory structures.

* `mv [switches] file_name new_name` - rename the file specified by the first argument of the call to the name specified by the second argument of the call. If the second argument of the call is a directory, then the file will be moved to that directory, for example:
* `mv abc.txt xyz.txt` - rename the file `abc.txt` to the name `xyz.txt` in the current directory;
* `mv /tmp/abc.txt ~` - move file `abc.txt` from directory `/tmp` to user's home directory.
  
* `touch [switches] file_name` - modifies information on file modification and reading times, but also allows to create a file, for example:
  * `touch abc.txt` - creates an (empty) `abc.txt` file in the current directory.
  
Commands for files (and directories) can also be issued using so-called generalization patterns, which are created using the following operators:

`*` - replaces any string of characters (including empty),

`?` - replaces exactly one arbitrary character,

`[<characters>]` - replaces exactly one character from the specified range, e.g.: `[xyz]`.

`[^<characters>]` - the ^ character at the beginning denotes the complement of the set, so for example, `[^xyz]`, denotes any character that is not `x, y` and `z`.

Here are sample commands using generalization patterns:

`cp ./*.txt ~` - copies all files with the extension .txt from the current directory to the user's home directory,

`rm ~/[0-9]*` - deleting all files from the home directory whose name begins with a digit.

## Text editors `nano`, `vim`.

When working with systems that do not have a graphical environment or through a remote terminal, it is often necessary to edit text files from the terminal. Efficient editing of text files is possible with console editors such as *Emacs*, *Vim* or *Nano*.

People who use such editors on a daily basis know plenty of shortcuts and tricks that make working with such a program more efficient than with a graphical editor.

### Nano

For occasional editing of files, such as configuration files, the *Nano* program is the easiest to use. It offers a quasi-graphical interface with keyboard shortcut prompts.

Example usage (if the file does not exist, it will be created):

```bash
nano file.txt
```

Key shortcuts:

* **Ctrl-o** - save.
* **Ctrl-x** - exit
* **Ctrl-k** - cut text (current line by default).
* **Ctrl-u** - paste text.

### Vim/Neovim

More efficient (but requiring some preliminary knowledge) file editing is possible in the *Vim* editor. There is also an alternative version of it: *Neovim*.

Example usage (if the file does not exist, it will be created):

```bash
vim file.txt
```

or for the *Neovim* version:

```bash
nvim file.txt
```

The most important keyboard shortcuts:

* `i` - enters editing mode, allows you to enter text.
* `a` - enters edit mode after the cursor, allows text entry.
* `Esc` - exits edit mode or interrupts command entry.

Commands (approved with enter):

* `:w` - save
* `:q` - exit
* `:q!` - exit without saving
* `:x` - save and exit

***

#### 🛠🔥 Task 2 🛠🔥.

Using *Vim* edit any text file.

#### 🛠🔥 Task 3 🛠🔥

Measure the editing time of any file using the `time` command:

```bash
time vim file.txt
```

***

## Shutting down, restarting and pausing the computer

On systems without a graphical user interface (so-called headless systems), it will be useful to know the commands to safely manage the operation of the machine.

The vast majority of Linux distributions use the system daemon *systemd*. In this regard, it is useful to know the commands:

* `systemctl poweroff` - shuts down the device,
* `systemctl reboot` - restarts the device,
* `systemctl suspend` - suspends (puts to sleep) the device.

Putting the device to sleep is supported on personal computers, but may not be implemented on single-board computers (for example, RaspberryPi).

There are shortcut versions (so-called aliases) for the mentioned main commands:

* `poweroff`.
* `reboot`.

***

#### 🛠🔥 Task 4 🛠🔥.

Test the operation of the `systemctl suspend` command.

***

## Installing packages

In Linux distributions, software is installed from central repositories. Avoid "cluttering" the system with external installers downloaded from websites (for example, in the form of *.run* files).

In the Ubuntu distribution, the `apt` utility is used to install packages. Some programs (usually windows-based) are also available as `snap` packages. Ubuntu has a graphical application "store" called *Software*.

***

#### 🛠🔥 Task 5 🛠🔥.

Run the `apt --help` command. Familiarize yourself with the available options. Try to search for any package (for example, *neovim*).

***

## Remote access to devices

Linux is equipped with tools to remotely access another device's console and transfer files.

### `ssh` command.

The `ssh` command is used to remotely access another device. The syntax of the command is as follows:

```bash
ssh <user>@<address>
```

For example:

```bash
ssh student@127.0.0.1
```

will connect us to our own computer.

If the username on the remote machine is identical to our username we can skip it in the command:

```bash
ssh 127.0.0.1
```

***

#### 🛠🔥 Task 6 🛠🔥

Ask your neighboring colleagues for their IP address. Connect to their computer and turn it off.

**NOTE 1**: account password is ***lrm***.

**NOTE 2**: with remote access, you may need to run the command to shut down the computer as an administrator - you can precede it with the *sudo* command.

***

### Transfer files between devices

The basic command for transferring files between devices is `scp`. The syntax is a combination of cp and ssh:

```bash
scp <local_file> <user>@<address>:<target_directory>
scp -r <local_directory> <user>@<address>:<target_directory>

scp <user>@<address>:<remote_file> <local_directory>
scp -r <user>@<address>:<remote_directory> <local_directory>
```

***

#### 🛠🔥 Task 7 🛠🔥

Download any file and directory from another computer. Change its contents and upload it back.

***

### `rsync` tool

The `rsync` tool is used for incremental (allows resuming) transfer of files and/or directories. It is very well suited for transferring large amounts of data. There are very many options for its configuration, but in most cases the following syntax will work well for transfer:

```bash
rsync -PHAXphax <local_directory> <user>@<address>:<target_directory>
rsync -PHAXphax <user>@<address>:<remote_file> <local_directory>
```

***

#### 🛠🔥 Task 8 🛠🔥

Check what the above flags of the `rsync` tool mean. Use the command to transfer the selected directory.

***
