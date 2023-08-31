# Turtlebot4 User Manual

This repository contains the source code for building the Turtlebot4 User Manual. To view the manual, click [here](https://turtlebot.github.io/turtlebot4-user-manual/).

## Build

To build the Turtlebot4's User Manual locally, first [install Ruby version 2.7.7](https://gorails.com/setup/ubuntu/22.04)

You can check your ruby the version using:
```bash
ruby -v
```
Ensure that it is showing the correct version.

<p>
<details>
    <summary><b>Using Ruby 3+</b></summary>
    Ruby 3+ isn't recommended due to [this issue](https://talk.jekyllrb.com/t/error-no-implicit-conversion-of-hash-into-integer/5890/2). Unfortunately, Ruby 3+ is required for `sass-converter` to be installed; a dependency of jekyll. If you don't want to (or can't) juggle multiple Ruby versions, you can run this patch to fix Ruby 3+ such that it works with Github Pages.

```shell
sudo sed -i.bak 's/, kwd/, **kwd/' $(gem which pathutil)
```
</details>
</p>

If you get the warning "No version is set for command ruby" then you need to set the active ruby version in asdf:
```bash
asdf global ruby 2.7.7
```
Ensure your Ruby gems are up to date:
```bash
gem update --system
```

Next [install Jekyll and the rest of the prerequisites](https://jekyllrb.com/docs/). Be careful to not overwrite the Ruby version. 

Clone this repository:

```bash
git clone https://github.com/turtlebot/turtlebot4-user-manual.git
```

Build and start a local server:

```bash
cd turtlebot4-user-manual
make
make server
```

The webpage will now be available at `127.0.0.1:4000`
