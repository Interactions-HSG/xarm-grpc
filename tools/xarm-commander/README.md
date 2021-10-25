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

