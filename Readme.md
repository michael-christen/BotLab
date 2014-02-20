# BotLab Repo
* To add remote type this command 
```
git remote add bruce maebot@192.168.3.106:ebolson/eecs467/src/BotLab.git
//to update type
git push bruce master
//While in bruce, pull while in ebolson/eecs467/src/BotLab
git pull
//to update
```

* To get the bot working do this in the bot
```
/ebolson/eecs467/src/maebot/maebot_driver & 
/ebolson/eecs467/src/apps/maebot_video_teleop &
```
and do this on you machine
```
/ebolson/eecs467/bin/vx_remote_viewer -a 192.168.3.106
```

* To generate lcm data do this
```
cd eecs467/lcmtypes
lcm-gen -c *.lcm
```
