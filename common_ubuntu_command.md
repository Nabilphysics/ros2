## Ubuntu Update, Upgrade and dist upgrade
The update command only updates the package list with the latest available versions, however, it does not install or upgrade the package
```
sudo apt-get update
```
The upgrade command actually upgrades and installs the latest versions of packages that are already installed. 
```
sudo apt-get upgrade
```
To upgrade a specific package, command is as follows:
```
sudo apt-get upgrade <package_name>
```
Similar to apt-get upgrade command, the apt-get dist-upgrade also upgrades the packages. In addition to this, it also handles changing dependencies with the latest versions of the package. It intelligently resolves the conflict among package dependencies and tries to upgrade the most significant packages at the expense of less significant ones, if required. Unlike apt-get upgrade command, the apt-get dist-upgrade is proactive and it installs new packages or removes existing ones on its own in order to complete the upgrade.
```
sudo apt-get dist-upgrade
```
```
sudo apt-get dist-upgrade <package_name>
```
## Install GUI Tool Stacer
```
sudo apt install stacer
```
![alt text](http://i0.wp.com/www.linuxlinks.com/wp-content/uploads/2018/06/Stacer.jpg?resize=650,400)
## apt list installed packages
```
apt list
```
## List all installed packages only
```
apt list --installed
```
## List App Package:
```
sudo apt list
```
## Remove package:
```
sudo apt purge PACKAGENAME
```
## More Package Related Command
https://www.cyberciti.biz/faq/apt-get-list-packages-are-installed-on-ubuntu-linux/
## Auto remove Unnecessary Package

This option removes libs and packages that were installed automatically to satisfy the dependencies of an installed package. If that package is removed, these automatically installed packages are useless in the system.
It also removes old Linux kernels that were installed automatically in the system upgrade.
It’s a no-brainer command that you can run from time to time to make some free space on your Ubuntu system:

```
sudo apt-get autoremove
```
You can remove a program in Ubuntu from the software centre or use the command below with the particular app names:
```
sudo apt-get remove package-name1 package-name2
```
## See size of the cache
Ubuntu uses APT (Advanced Package Tool) for installing, removing and managing software on the system, and in doing so it keeps a cache of previously downloaded and installed packages even after they’ve been uninstalled.
The APT package management system keeps a cache of DEB packages in /var/cache/apt/archives. Over time, this cache can grow quite large and hold a lot of packages you don’t need.
You can see the size of this cache with the du command below:
```
sudo du -sh /var/cache/apt
```
## Clean APT cache
You have two ways to clean the APT cache.
Either remove only the outdated packages, like those superseded by a recent update, making them completely unnecessary.
```
sudo apt-get autoclean
```
Or delete apt cache in its entirety (frees more disk space):
```
sudo apt-get clean
```
## Clear systemd journal logs 
Every Linux distribution has a logging mechanism that helps you investigate what’s going on in your system. You’ll have kernel logging data, system log messages, standard output and errors for various services in Ubuntu.
The problem is that over time, these logs take a considerable amount of disk space. You can check the log size with this command:
```
journalctl --disk-usage
```
Now, there are ways to clean systemd journal logs. The easiest for you is to clear the logs that are older than certain days.
```
sudo journalctl --vacuum-time=3d
```
## Virual Box Copy-paste problem solution.
Install Virtual Box Guest Edition
Follow This link: https://linuxconfig.org/install-virtualbox-guest-additions-on-linux-guest

