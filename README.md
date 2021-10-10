# xarm-commander
A C++ CLI Tool for controlling [UFACTORY xArm series](https://www.ufactory.cc/pages/xarm) using the [xArm-CPLUS-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK).

The dependencies are added as subomodules, so when cloning this repository, do:
```console
$ git clone git@github.com:Interactions-HSG/xarm-commander.git --recursive
```

This project consists of two artifacts:
- `xarm-commander`: The main CLI program to run commands on xArms implemented as a gRPC client.
- `xarm-daemon`: A daemon responsible for the Modbus/TCP connection manager and packet generation (i.e., `xArmAPI` object) implemented as gRPC server providing the `xArmAPI` as a service.

The proto file is defined in `proto/xapi.proto` based on [`xarm_api.h`](https://github.com/xArm-Developer/xArm-CPLUS-SDK/blob/master/include/xarm/wrapper/xarm_api.h).

`xarm-commander` implements a few subcommands specific to the connection management:
- `initialize`: instantiate a `xArmAPI` object and connect the daemon to a xArm specified by the `-x` option.
- `disconnect`: disconnect the daemon from the xArm.

## Preparation

### xArm-CPLUS-SDK
To use the xarm-commander, first install the xArm-PLUS-SDK library:
```console
$ sudo make -C libs/xArm-CPLUS-SDK xarm install
```

uninstall:
```console
$ sudo make -C libs/xArm-CPLUS-SDK uninstall
```

clean the build folder of the xarm library
```console
$ sudo make -C libs/xArm-CPLUS-SDK clean
```

### cmake, gRPC and Protocol Buffers
Follow the [gRPC's Quick start guide](https://grpc.io/docs/languages/cpp/quickstart/) to install cmake, gRPC, and Protocol Buffers.

The rest of this document assumes you have installed gRPC and Protocol Buffers in `~/.local`.

## Build `xarm-daemon` and `xarm-commander`

```console
$ mkdir -p cmake/build
$ cmake -DCMAKE_PREFIX_PATH=~/.local ../..
$ make -
```

## Synopsis
For further description about usage of the xarm-commander, use -h or --help:

xarm-daemon needs to be running in order to run any command from xarm-commander.

Start the daemon by:
```console
$ xarm-daemon start
```

then:
```console
$ xarm-commander initialize -x <ip address of your xarm>
Initialized
$ xarm-commander get_version
Verion: h2,v1.6.5
```
> :bulb: Can be also used for subcommands: `xarm-commander get_version -h`

## Examples
The `examples` directory contains examples on how to integrate the xarm-commander into different languages.  
Currently, the following examples are provided:

### `routine.sh`: A simple collection of commands called from a bash script.
```console
$ sh examples/routine.sh <IP address of xArm7 control box>
```

### `node_async.js`: Integration into Javascript using `async` functions.
```console
$ node examples/node_async.js <IP address of xArm7 control box>
```

### `node_promise.js`: Integration into Javascript using `promises`.
```console
$ node examples/node_promise.js <IP address of xArm7 control box>
```
> :bulb: NodeJS version 10+ (e.g., 10.13.0 LTS) is required to run the `node_*` examples.

## Dependencies
- [CLI11](https://github.com/CLIUtils/CLI11)
- [CMake](https://github.com/Kitware/CMake)
- [Easylogging++](https://github.com/amrayn/easyloggingpp/)
- [{fmt}](https://github.com/fmtlib/fmt)
- [gRPC](https://github.com/grpc/grpc)
- [xArm-CPLUS-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK)

## Maintainers

- Jonas Brütsch ([@jo-bru](https://github.com/jo-bru))
- Iori Mizutani ([@iomz](https://github.com/iomz))

## License
See `LICENSE`.

