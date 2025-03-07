---
sort: 1
---

# WSU Roboracer: NoMachine

## Install NoMachine on RoboRacer

```bash
wget https://www.nomachine.com/free/arm/v8/deb -O nomachine.deb
```

### dpkg

```bash 
sudo dpkg -i nomachine.deb
```

[documentation]('https://kb.nomachine.com/AR02R01074')

## Install NoMachine on Host Machine -- Ubuntu


[download this]('https://downloads.nomachine.com/download/?id=1')

### dpkg

```bash 
cd ~/Downloads
sudo dpkg -i nomachine_8.16.1_1_amd64.deb
```

## Connect to Roboracer

On Host machine open NoMachine

Click on the Add button
![add_computer](media/add_comp.png)

Add name of bot 
Add ip address of bot 
Check Always accept the hosts verification
Then click Add
![add_bot](media/add_name.png)

Find the bot in the list of computers and select
![find_bot](media/load.png)

Enter in the credentials for the roboracer
![credentials](media/credentials.png)

Connected to Desktop of roboracer
![roboracer](media/connected.png)