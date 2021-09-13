# xarm-commander
A C++ CLI Tool for controlling the xArm7 Robot using the [xArm-CPLUS-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK) and [CLI11](https://github.com/CLIUtils/CLI11).
For demonstration purposes, only two simple commands have been implemented so far.

The `xArm-CPLUS-SDK` is added as a subomodule, so when cloning this repository, do
```
~ % git clone git@github.com:Interactions-HSG/xarm-commander.git --recursive
```

## Preparation: xArm-CPLUS-SDK
To use the xarm-commander, first install the xArm-PLUS-SDK library.
```console
# install
~/xarm-commander % sudo make -C libs/xArm-CPLUS-SDK xarm install

# uninstall
~/xarm-commander % sudo make -C libs/xArm-CPLUS-SDK uninstall

# clean the build folder of the xarm library
~/xarm-commander % sudo make -C libs/xArm-CPLUS-SDK clean
```

## Install
```console
# compile and build the sourcecode for the xarm-commander:
~/xarm-commander % make

# to use the xarm-commander globally install it in `/usr/local/bin`
~/xarm-commander % sudo make install

# uninstall the xarm-commander
~/xarm-commander % sudo make uninstall

# delete the build folder of the xarm-commander
~/xarm-commander % make clean
```

## Examples
The `examples` directory contains examples on how to integrate the xarm-commander into different languages.  
Currently, the following examples are provided:

### `routine.sh`: A simple collection of commands called from a bash script.
```console
~/xarm-commander % sh examples/routine.sh <IP address of xArm7 control box>
```

### `node_async.js`: Integration into Javascript using `async` functions.
```console
~/xarm-commander %
```

### `node_promise.js`: Integration into Javascript using `promises`.
```console
~/xarm-commander %
```

## Synopsis
For further description about usage of the xarm-commander, use -h or --help:
```console
~/xarm-commander % bin/xarm-commander -h
```
> :bulb: Can be also used for subcommands: `xarm-commander get_version -h`

## Maintainers

- Jonas Brütsch ([@jo-bru](https://github.com/jo-bru))
- Iori Mizutani ([@iomz](https://github.com/iomz))

## License
See `LICENSE`.

