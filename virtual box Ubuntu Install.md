## Install Virtualbox on Windows

Step 1: <br>
Download Ubuntu from https://ubuntu.com/download 

Step 2: <br>
Download Virtualbox from https://www.virtualbox.org/

Step 3: 
Follow https://trendoceans.com/install-ubuntu-on-virtualbox/ to install ubuntu in virtualbox

Step 4: Install Virtualbox Guest Eddition Resource: (https://linuxconfig.org/install-virtualbox-guest-additions-on-linux-guest)
1. Install prerequisite packages
```
sudo apt update
sudo apt install build-essential dkms linux-headers-$(uname -r)
```
2.  On the virtual machine window, click on Devices > Insert Guest Additions CD Image.
3.  Instead of running the software on the disc image when the prompt pops up, we will make a new directory to which we can mount the Guest Additions CD Image. Make the directory and then execute the mount command to mount it.
```
sudo mkdir -p /mnt/cdrom
sudo mount /dev/cdrom /mnt/cdrom
```
4. 
```
cd /mnt/cdrom
sudo ./VBoxLinuxAdditions.run
reboot
```
5. Virtualbox > Devices > Drag and Drop > Biodirectional
