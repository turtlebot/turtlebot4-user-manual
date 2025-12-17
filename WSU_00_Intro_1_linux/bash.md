---
sort: 1
---
## Bash in Linux

**Bash** (Bourne Again SHell) is the default command-line interface (CLI) for many Linux distributions. It's a Unix shell and command language, and is an enhanced replacement for the original Bourne Shell (`sh`).

### Features

1. **Command Line Editing**: Bash provides facilities to edit the command line, with shortcuts similar to Emacs or vi.
2. **Job Control**: Run multiple processes from a single bash session.
3. **History**: Keeps a list of previously executed commands which can be recalled and executed again.
4. **Scripting**: Bash can be used to write scripts, a series of commands to be executed.
5. **Aliases**: Create shortcuts for commands or command sequences.
6. **Autocompletion**: Press `Tab` to auto-complete commands, filenames, etc.

### Basic Commands

- `ls`: List files and directories.
- `cd`: Change the current directory.
- `pwd`: Print the current directory.
- `echo`: Display a message or output.
- `man`: Display the manual page for a command.
- `mkdir`: Create a directory.
- `rmdir`: Remove a directory.
- `cat`: Display the content of a file.
- `rm`: Remove files or directories.
- `cp`: Copy files or directories.
- `mv`: Move or rename files or directories.

### Scripting

Bash scripts usually have the extension `.sh` and start with the shebang line:

\```bash
#!/bin/bash
\```

This line indicates the interpreter to be used for executing the script, which is bash in this case.

## Common Linux Terminal Commands

Here are some common terminal commands for Linux systems.

### Navigation Commands

- `ls`: List files and directories in the current directory.
- `cd [dir]`: Change the current directory to the directory specified. For example, `cd /home/user` would change the current directory to `/home/user`.
- `pwd`: Print the path of the current working directory.

### System Administrations Commands

- `sudo`: `sudo [command]`: The `sudo` command stands for "Super User DO", and it is used to perform tasks that require administrative or root permissions. It is used in front of other commands to execute them with superuser privileges.

 - `sudo apt update`: This command updates the list of available packages and their versions, but it does not install or upgrade any packages. Apt (Advanced Package Tool) is a command-line package manager and requires superuser permissions for many operations.

- `sudo apt upgrade`: This command actually installs newer versions of the packages you have. After updating the lists, the package manager knows about available updates for the software you have installed.

### File and Directory Operations

- `cp [source] [destination]`: Copy a file or directory from the source to the destination.
- `mv [source] [destination]`: Move a file or directory from the source to the destination.
- `rm [file]`: Remove a file. Be careful with this command, as it permanently deletes files.
- `mkdir [dir]`: Create a new directory.

### System Information

- `uname -a`: Display all system information.
- `df -h`: Display disk space usage in human-readable format.
- `free -h`: Display free and used memory in the system in human-readable format.

### Process and Job Control

- `ps`: List the currently running processes.
- `top`: Display the system's running processes in real-time.
- `kill [PID]`: Kill a process with a specific PID (Process ID).

### Network Commands

- `ping [host]`: Send a network request to a specific host to check if it is up and running.
- `ifconfig` or `ip addr`: Display or configure a network interface.
- `netstat`: Network Statistics.

### File Permissions

- `chmod [permissions] [file]`: Change the file system permissions of files and directories.
- `chown [user]:[group] [file]`: Change the user and group ownership of files and directories.

### Searching and Sorting

- `grep [pattern] [file]`: Search for a specific pattern within a file.
- `sort`: Sort lines in text files.
- `find [dir] -name [file]`: Find files inside a directory.

### Compression and Archiving

- `tar -cvf [file.tar] [file or dir]`: Archive files or directories.
- `tar -xvf [file.tar]`: Extract files or directories from an archive.
- `gzip [file]`: Compress a file to a .gz archive.
- `gunzip [file.gz]`: Decompress a .gz archive.

Please note that you should replace `[...]` with your specific arguments


## Further Details on common commands

### ls

### Navigation Commands

- `ls`: Lists files and directories in the current directory. The `ls` command has many options that can help you format the output, display additional details, and more. Here are some common options:

    - `ls -l`: Long listing. Instead of just the filenames, display a detailed directory listing that includes the file permissions, number of links, owner, group, size, and time of the last modification.

    - `ls -a`: Show all files and directories, including hidden ones. In Linux, files and directories that start with a dot (`.`) are hidden.

    - `ls -h`: Human-readable sizes. Display file size in a more human-readable format (e.g., KB, MB) in the long listing view.

    - `ls -t`: Sort by modification time. The newest files will be displayed first.

    - `ls -r`: Reverse order. Reverse the order of how files are displayed (useful with -t, for example, to show oldest files first).

    - `ls -R`: Recursive. List the files in subdirectories as well.

    - `ls -S`: Sort by file size.

    - `ls -1`: List one file per line.

For example, if you wanted a long listing of all files, including hidden ones, in human-readable format, you would use `ls -laH`.

These options can be combined in most cases. For example, `ls -lS` would give a long listing sorted by file size. Keep in mind that not all combinations of options make sense together.

As always, you can use `man ls` or `ls --help` to get more information about the `ls` command and its options.


## cd

### Navigation Commands

- `cd [dir]`: Change the current directory to the directory specified. This command is used to navigate the directory structure of a file system. The `[dir]` is a placeholder for the directory you want to navigate to. Here are some common usages:

    - `cd /`: Changes the directory to the root directory.

    - `cd ..`: Moves the directory one level up.

    - `cd -`: Changes to the previous directory you were in.

    - `cd ~` or `cd`: Changes the directory to the home directory.

    - `cd /path/to/directory`: Changes the directory to a specific directory path. The path can be relative to the current directory or an absolute path.

If the directory name contains spaces, you should enclose the path in quotes. For example: `cd "Directory Name"` or `cd 'Directory Name'`.

As always, you can use `help cd` to get more information about the `cd` command.


### System Administration Commands

- `sudo [command]`: The `sudo` command stands for "Super User DO", and it is used to perform tasks that require administrative or root permissions. It is used in front of other commands to execute them with superuser privileges. Here are some examples:

    - `sudo apt update`: This command updates the list of available packages and their versions, but it does not install or upgrade any packages. Apt (Advanced Package Tool) is a command-line package manager and requires superuser permissions for many operations.

    - `sudo apt upgrade`: This command actually installs newer versions of the packages you have. After updating the lists, the package manager knows about available updates for the software you have installed.

    - `sudo reboot`: This command will reboot the system immediately. Normal users do not have this privilege, so it requires superuser permissions.

    `sudo` can be a dangerous command if misused. It gives you root-level (the highest level) access, so it should be used with caution.

    Also, in some Linux distributions, you might need to configure `sudo` before using it. This is done by adding users to the sudoers file, which is typically located at `/etc/sudoers`. This should be done with caution, and preferably using the `visudo` command which includes some syntax verification.

    In other distributions like Ubuntu, the primary user is already configured with `sudo` access upon system installation.

    As always, you can use `man sudo` or `sudo --help` to get more information about the `sudo` command and its options.



## chmod 

### File Permissions using chmod

- `chmod [permissions] [file]`: Change the file system permissions of files and directories. 

Permissions are defined as a three-digit number where each digit is an octal number that represents the permissions for the owner, group, and others respectively. Each octal number is the sum of read (4), write (2), and execute (1) permissions. For example, `chmod 755 [file]` would give the owner read, write, and execute permissions (7), and the group and others only read and execute permissions (5). 

Here is what each of these means:

- Read (4): The file can be opened, and its content viewed.
- Write (2): The file can be edited, modified, and deleted.
- Execute (1): If the file is a script or a program, it can be run (executed).

For directories, the permissions have slightly different meanings:

- Read (4): The directory listing can be obtained.
- Write (2): Items within the directory can be created, deleted, and renamed if execute permission is also set.
- Execute (1): The directory can be entered (i.e., `cd [dir]`).

To apply these permissions:

- For the user who owns the file/directory (`u`):
- For other users in the file's group (`g`):
- For other users not in the file's group (`o`):
- For all users (`a`):

You can use the format `ugoa` (where `u` = user, `g` = group, `o` = others, `a` = all) combined with `+-=` (where `+` = add a permission, `-` = remove a permission, `=` = set exactly these permissions) and `rwx` (where `r` = read, `w` = write, `x` = execute). 

For example:

- `chmod u+x [file]`: This would add (`+`) execute (`x`) permission for the user (`u`) who owns the file or directory.
- `chmod go-rw [file]`: This would remove (`-`) both read (`r`) and write (`w`) permissions for other users in the file's group (`g`) and for other users not in the file's group (`o`).

Be careful while using this command and make sure you know exactly what permissions you are setting.

## VS Code

### Open Folder in vs code

Navigate the the directory that you would like to open in vscode

```shell
code .
```
