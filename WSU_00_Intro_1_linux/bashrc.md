---
sort: 2
---
## `.bashrc` File in Linux

The `.bashrc` file is a script that runs every time Bash is started in interactive mode. It is used to configure the shell environment for the user. This can include setting environment variables, defining aliases, and customizing the shell prompt.

### Location

The `.bashrc` file is typically located in the user's home directory:

\```
~/.bashrc
\```

### Common Uses

1. **Setting Environment Variables**: Define or modify environment variables like `PATH`, `EDITOR`, and others.
   
   \```bash
   export PATH="$PATH:/path/to/directory"
   \```

2. **Aliases**: Aliases are shortcuts for longer commands. They allow users to define their own command abbreviations for frequently used commands or command sequences.

   - Basic alias creation:
   
     \```bash
     alias ll="ls -lah"
     \```

   - Removing an alias:
   
     \```bash
     unalias ll
     \```

   - Listing all defined aliases:
   
     \```bash
     alias
     \```
   \```

3. **Customizing PS1**: Modify the shell prompt to display custom information.

   \```bash
   PS1="\u@\h:\w$ "
   \```

### Reloading `.bashrc`

Changes made to `.bashrc` won't take effect until the next time the shell starts. However, you can manually source the `.bashrc` file to apply changes immediately:

\```bash
source ~/.bashrc
\```

or simply:

\```bash
. ~/.bashrc
\```

### Tips

- Before making significant changes, it's a good practice to back up your current `.bashrc`.
- For system-wide configurations, the `/etc/bash.bashrc` file can be modified, but do so with caution.
- Some distributions may also utilize a `.bash_profile` or `.profile` file for login shells. Make sure to understand the differences and which file to edit based on your needs.
