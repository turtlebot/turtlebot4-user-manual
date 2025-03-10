---
sort: 2
---

# VS CODE /Chromium on RoboRacer

## VS Code
```bash
sudo apt update && sudo apt upgrade -y
```

```bash
sudo apt install wget gpg -y
```

```bash
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /usr/share/keyrings/packages.microsoft.gpg > /dev/null
```

```bash
echo "deb [signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
```

```bash
sudo apt update
sudo apt install code -y
```

## ARM

```bash
sudo apt install chromium-browser -y
```
