After creating ubuntu virtual box you might want to increase disk size. First incease the size from virtual box using this link https://itsfoss.com/increase-disk-size-virtualbox/ . Then turn on the ubuntu virtual machine. 

Video: https://www.youtube.com/watch?v=a-MeH95ei1g

Step 1: 
```sudo fdisk /dev/sda```

Step 2: 
Type p in ```Command (m for help): p```

Step 3:
Type d then type the Partition number. In my case it was 3

Step 4:
Type n

Step 5:
Press enter to select First and Last Sector

Step 6: 
Type N in the prompt "Do you want to remove the signature? [Y]es/[N]o: N"

Step 7:
Type w 
You should see Syncing Disks.

Step 8:
Using ```lsblk``` you can see whether everying is ok or not. 


