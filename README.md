# xarm-grpc
A gRPC service implemenation for controlling [UFACTORY xArm series](https://www.ufactory.cc/pages/xarm) using the [xArm-CPLUS-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK).

The dependencies are added as subomodules, so when cloning this repository, do:
```console
$ git clone git@github.com:Interactions-HSG/xarm-grpc.git --recursive
```

This project implements the following:
- `xarm-grpc-service`: The gRPC service responsible for the Modbus/TCP connection management and packet generation (i.e., `xArmAPI` object) providing the `xArmAPI` as a service.
- `xarm-commander`: A gRPC client implemented as a C++ CLI Tool to run commands on xArms. For further information about the `xarm-commander` go to section Tools.


The proto file is defined in `proto/xapi.proto` based on [`xarm_api.h`](https://github.com/xArm-Developer/xArm-CPLUS-SDK/blob/master/include/xarm/wrapper/xarm_api.h).

The following services are specific to the connection management:
- `Initialize`: instantiate a `xArmAPI` object and connect the server to a xArm specified by the `-x` option.
- `Disconnect`: close the connection to the xArm.

## Preparation

### xArm-CPLUS-SDK
To use the `xarm-grpc-service` first install the xArm-PLUS-SDK library:
```console
$ make -C libs/xArm-CPLUS-SDK xarm
```

### cmake, gRPC and Protocol Buffers
Follow the [gRPC's Quick start guide](https://grpc.io/docs/languages/cpp/quickstart/) to install cmake, gRPC, and Protocol Buffers.

The rest of this document assumes you have installed gRPC and Protocol Buffers in `~/.local`.

## Build `xarm-grpc-service` and `xarm-commander`

```console
$ mkdir -p cmake/build && cd cmake/build
$ cmake -DCMAKE_PREFIX_PATH=~/.local ../..
$ make -j
```
For global installation then run:
```console
$ cp xarm-grpc-service ~/.local/bin
$ cp xarm-commander ~/.local/bin
```

## Synopsis
The gRPC service can be started using following command:
```console
$ xarm-grpc-service start
```
Then use a gRPC client (e.g., `xarm-commander`) to send commands to the xArm.

To close the gRPC service run:
```console
$ xarm-grpc-service stop
```
For further description about usage of the `xarm-grpc-service`, use `-h` or `--help`.

## Tools
The Tools directory includes available clients for the xarm-grpc-service.
Currently, the following clients are implemented:
- `xarm-commander`: A gRPC client implemented as a C++ CLI Tool to run commands on xArms. For further instructions on how to use the `xarm-commander` follow [this link](tools/xarm-commander).

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

