# Turtlebot4 User Manual

This repository contains the source code for building the Turtlebot4 User Manual. To view the manual, click [here](https://turtlebot.github.io/turtlebot4-user-manual/).

## Build

To build the Turtlebot4's User Manual locally, first [install Ruby version 3.2](https://gorails.com/setup/ubuntu/24.04)

You can check your ruby the version using:
```bash
ruby -v
```
Ensure that it is showing the correct version.

<p>
<details>
    <summary><b>Configuring Ubuntu 24.04</b></summary>
    By default, Ubuntu 24.04 uses a debian-packaged version of Ruby, which installs gems to a root-owned path. This makes installing additional gems problematic.
    To work around this issue, run the following commands:
    ```shell
    mkdir $HOME/.ruby
    echo 'export GEM_HOME=$HOME/.ruby/' >> $HOME/.bashrc
    echo 'export PATH="$PATH:$HOME/.ruby/bin"' >> $HOME/.bashrc
    source $HOME/.bashrc
    ```
    This will make the `gem` command install Ruby gems to your local user's `.ruby` directory.
</details>
</p>

Ensure your Ruby gems are up to date:
```shell
gem update --system
```

Clone this repository:

```bash
git clone https://github.com/turtlebot/turtlebot4-user-manual.git
```

Build and start a local server:

```bash
cd turtlebot4-user-manual
make
make update
make server
```

The webpage will now be available at `http://127.0.0.1:4000/turtlebot4-user-manual`
