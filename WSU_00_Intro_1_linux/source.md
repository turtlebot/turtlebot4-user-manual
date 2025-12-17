---
sort: 3
---

## `source` Command in Linux

The `source` command in Linux is a shell built-in command used to read and execute commands from a file within the current shell session. By sourcing a script, you can set environment variables, define functions, and run commands without spawning a new shell process.

### Syntax

The syntax for the `source` command is:

\```
source filename [arguments]
\```

`filename` is the path to the file you want to source, and `arguments` are any arguments you want to pass to the script.

### Usage

1. **Environment Variables**: If you have a file, `env.sh`, that sets some environment variables, you can use the `source` command to load these variables into your current shell.

   \```bash
   source env.sh
   \```

2. **Functions**: If you define functions in a file and want to use them in your current shell, you can source the file.

   \```bash
   source functions.sh
   \```

3. **Aliases**: For loading aliases defined in a file into your current shell.

   \```bash
   source aliases.sh
   \```

### Alternatives

The `.` (dot) command is equivalent to `source` in most shells:

\```bash
. filename [arguments]
\```

### Tips

- The `source` command affects the current shell environment, so be cautious when sourcing files.
- It's commonly used in shell startup files like `.bashrc` or `.bash_profile` to load configurations.
- The file being sourced does not need to have execute permissions, as it's not being executed as a separate process.

## Sourcing ROS Environment with `source`

When working with the Robot Operating System (ROS), it's essential to configure your shell to load the necessary environment variables for the ROS distribution you're using. This ensures that the shell knows where to find ROS packages and tools.

### Command

The command to set up the environment for ROS (assuming the distribution name is "humble") is:

\```
source /opt/ros/humble/setup.bash
\```

### Usage

1. **Initialize ROS Environment**: This command configures the shell to use the tools and packages of the specified ROS distribution.

2. **ROS Nodes**: After sourcing the setup file, you can start ROS nodes, and they will be aware of other nodes, packages, and tools in the ROS ecosystem.

3. **ROS Tools**: Tools like `roscd`, `roslaunch`, and `rosrun` will work correctly, pointing to the right directories and packages.

### Why `source`?

Using `source` (or the equivalent `.`) ensures that the commands and environment variables in the `setup.bash` script are executed in the current shell, rather than a sub-shell. This means the configurations apply to your current session and any subsequent commands you run.

### Tips

- Always source the `setup.bash` file of the ROS distribution you're working with before running any ROS-related commands in a new shell session.
- If you're using multiple ROS distributions or workspaces, ensure you're sourcing the correct `setup.bash` file for the environment you intend to work in.
- Consider adding the `source` command to your shell's startup file (like `.bashrc`) if you're consistently working with a particular ROS distribution.



