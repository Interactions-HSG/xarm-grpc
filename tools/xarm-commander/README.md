# xarm-commander
A C++ CLI Tool which implements a gRPC client for the `xarm-grpc-service`.  
This tool can be used for controlling [UFACTORY xArm series](https://www.ufactory.cc/pages/xarm) using the [xArm-CPLUS-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK).


## Synopsis
Before using the `xarm-commander` make sure that the `xarm-grpc-service` is already running using a correct port and address.  
For description on how to start this service, see [main README](../../README.md).

Example usage:
```console
$ xarm-commander initialize -x <ip address of your xarm>
Initialized
$ xarm-commander get_version
Verion: h2,v1.6.5
```

For further description about usage of the xarm-commander, use -h or --help.
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


