---
sort: 8
---

# Creating a backup of your SD card

You may find it helpful to take an image of your Turtlebot's microSD card. This makes it easy to
restore a backup, or deploy the same image to multiple robots by flashing the same image to
multiple cards.

## Clean up the image

Before creating the image, you may find it helpful to clean up some automatically-generated files
from the Raspberry Pi. This can reduce the size of the image.

SSH into the Turtlebot and run any or all of the following commands:
```bash
# Empty journalctl logs
sudo journalctl --flush --rotate
sudo journalctl --vacuum-time=1s

# Clean cached apt packages
sudo apt clean

# Empty Bash History
rm .bash_history
history -c
```

## Remove the microSD card

Once you have cleaned the system, power off the Turtlebot and remove the microSD card from the
Raspberry Pi. Plug the card into your laptop (you may need an adaptor if your computer does not
have a built-in microSD card reader).

Open a terminal and perform the following steps.

## Determine the card's device file

Determine the microSD card's device handle on your laptop. Run `lsblk -d -e7` in a terminal to
list connected storage devices. The output should look something like this:
```
NAME    MAJ:MIN RM   SIZE RO TYPE MOUNTPOINTS
mmcblk0 179:0    0  29.7G  0 disk
nvme0n1 259:0    0 476.9G  0 disk
```

Find the drive whose size is closest to your microSD card and note its name. In the example above
a 32GB card is used, so the 29.7G drive, named `mmcblk0` is the card. Note this name for subsequent
steps. Anytime in the following instructions, wherever you see `mmcblk0` replace it with your SD
card's file if it is different.

> **WARNING**
>
> The following steps will make modifications to your SD card. To make a simple backup that will not
> modify the SD card, run
>
> ```bash
> sudo dd if=/dev/mmcblk0 of=my_turtlebot_backup.img status=progress
> ```
>
> This will produce a much larger image on-disk, but will not modify the SD card you are copying.

## Shrink the filesystem

> **WARNING**
>
> This step will modify the SD card.

Typically the file system on the microSD card will not take up the entire drive. By shrinking the
filesystem we can reduce the size of the image we will create.  Run the following commands,
replacing `mmcblk0` with the name of your card if necessary.

Unmount the partitions if they are mounted:
```bash
for part in $(ls /dev/mmcblk0p*); do
  sudo umount $part
done
```

Determine the size of the main storage partition. Using the standard Turtlebot SD card images, this
is partition `p2`. If you have created your own custom image the partition may be different.
```bash
sudo e2fsck -f /dev/mmcblk0p2
```
This will output something like this:
```
e2fsck 1.47.0 (5-Feb-2023)
Pass 1: Checking inodes, blocks, and sizes
Pass 2: Checking directory structure
Pass 3: Checking directory connectivity
Pass 4: Checking reference counts
Pass 5: Checking group summary information
writable: 194593/1868256 files (0.5% non-contiguous), 1770531/7660411 blocks
```
Note the final line where it specifies the number of blocks used, `1770531` in the example
above.

Determine the block size by running
```bash
sudo tune2fs -l /dev/mmcblk0p2 |grep -i "block size"
```
This will produce output like this, indicating the block size in bytes:
```
Block size:               4096
```

Multiply the number of used blocks (`1770531`) by the block size (`4096`) to determine the number
of bytes used by the filesystem, `1770531 * 4096 = 7252094976` in this example. Round this value
up slightly to get the number of gigabytes to resize the filesystem to. It is advisable to add a
little extra room, e.g. 0.25 to 0.5GB to make sure there's extra room in the filesystem the first
time it boots. In this example we have approximately 7.25GB used, so we will round up to 7.5GB.

Resize the filesystem by running
```bash
sudo resize2fs /dev/mmcblk0p2 7.5G
```

After resizing, re-run
```bash
sudo e2fsck -f /dev/mmcblk0p2
```
to make sure nothing was corrupted.

## Shrink the partition

> **WARNING**
>
> This step will modify the SD card.

After shrinking the filesystem we can resize the actual partition. The instructions below assume
you are using the standard Turtlebot 4 SD card image, where the user data is stored on the second
partition. If you have a custom image you may need to resize a different partition.

Run the following command:
```bash
sudo parted /dev/mmcblk0
```
`parted` is a command-line tool with many options.

Run the `print` command to print the size of the partition in GiB. One GiB is `2^30 = 1073741824`
bytes, as opposed to one GB which is `10^9 = 1000000000` bytes, so the numbers printed will not
necessarily be the same as in previous steps. The output of the `print` command will look
something like this:
```
Model: SD SP32G (sd/mmc)
Disk /dev/mmcblk0: 31.9GB
Sector size (logical/physical): 512B/512B
Partition Table: msdos
Disk Flags:

Number  Start   End     Size    Type     File system  Flags
 1      1049kB  538MB   537MB   primary  fat32        boot, lba
 2      538MB   31.9GB  31.4GB  primary  ext4
```

Note that the sizes are in MiB and GiB, despite using "MB" and "GB" units. (One MiB is
`2^20 = 1048576` bytes.)

Starting with the size of the filesystem from the previous step (7.5GB for this example), increase
the size by 0.2. Multiply this result by `2^30`: `(7.5 + 0.2) * 2^30 = 7.7 * 1073741824 = 8267812044.8`.

Add this value to the start position of partition `2` in bytes (`538 * 2**20 = 564133888`):
`8267812044.8 + 564133888 = 8831945932.8`. Divide this value by the block size of `512`:
`8831945932.8 / 512 = 17249894.4`. Round this value up to the next `1000` to get the new end sector
for partition `2`: `17249894.4 -> 17250000`. Note this number for the next step.

In `parted` run the following command to resize the partition:
```
resizepart 2 17250000s
```
Note that the `s` after the number is required to specify the end sector.

When the operation completes re-run the `print` command to make sure the change was applied.
Quit `parted` by typing `quit`.

In the `bash` terminal, check the number of sectors in the disk:
```bash
sudo fdisk -l /dev/mmcblk0
```

## Make the image

Finally we can create the SD card image using the `dd` command:
```bash
sudo dd if=/dev/mmcblk0 of=turtlebot4_custom_sd_image.img bs=512 count=17250000 status=progress
```
with the following substitutions:
- `if` is set to the SD card's device file,
- `of` is the name you wish to give the image, and
- `count` is the number of the end sector determined in the previous step.

## Flashing the image

You can flash your new Turtlebot4 image by following the
[same steps](../setup/basic.md#install-latest-raspberry-pi-image) you would use to install the
latest official SD card image for your robot:
```bash
wget https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/jazzy/scripts/sd_flash.sh
bash sd_flash.sh turtlebot4_custom_sd_image.img
```
