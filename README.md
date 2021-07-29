# xarm-commander
A C++ CLI Tool for controlling the xArm7 Robot using the [xArm-C++-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK) and [CLI11](https://github.com/CLIUtils/CLI11).
For demonstration purposes, only two simple commands have been implemented so far.

## Installation: Linux
To use the xarm-commander, first install the xArm-C++-SDK library:
```
sudo make lib-install
```
Then compile and build the sourcecode for the xarm-commander:
```
make commander
```
To use the xarm-commander globally install it in `/usr/bin`:
```
sudo make install
```
> :bulb: These steps can be automatically run by using: `sudo make all`  

## Uninstall/Clean: Linux
Uninstall the xArm-C++-SDK library:
```
sudo make lib-uninstall
```
Clean the build folder of the xarm library:
```
make lib-clean
```
Uninstall the xarm-commander:
```
sudo make uninstall
```
Delete the build folder of the xarm-commander:
```
make clean
```
> :bulb: These steps can be automatically run by using: `sudo make clean-all`  
## Help
For further help on the xarm-commander usage, use -h or --help:
```
xarm-commander -h
```
> :bulb: Can be also used for subcommands: `xarm-commander get_version -h`  

