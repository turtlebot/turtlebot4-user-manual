---
sort: 2
---

# Lidar Setup

### **Step 1: Create a New Network Connection**

Run the following command to create a connection named `Hokuyo` for the `enp8p1s0` interface:

```bash
sudo nmcli connection add type ethernet con-name Hokuyo ifname enP8p1s0 ipv4.addresses 192.168.0.15/24 ipv4.gateway 192.168.0.10 ipv4.method manual
```

This sets:

- **IP address**: `192.168.0.15`
- **Subnet mask**: `/24` (equivalent to `255.255.255.0`)
- **Gateway**: `192.168.0.10`
- **Manual IP configuration** (not DHCP)

---

### **Step 2: Verify the Connection**

List all available connections to confirm the `Hokuyo` connection was created:

```bash
nmcli connection show
```

You should see an entry for `Hokuyo`.

---

### **Step 3: Activate the Connection**

Activate the newly created `Hokuyo` connection:

```bash
sudo nmcli connection up Hokuyo
```

This enables the configuration for the `enp8p1s0` interface.

---

### **Step 4: Verify the Network Configuration**

Check that the `enp8p1s0` interface has the correct settings:

```bash
ip addr show enP8p1s0
```

You should see the IP address `192.168.0.15` assigned to the interface.

---

### **Step 5: Test the Connection**

Ping the Hokuyo LiDAR at `192.168.0.10` to confirm connectivity:
`ping 192.168.0.10`

---
create alias for quicker lidar ping in .bashrc add

```bash
alias lidar="ping 192.168.0.10"
```