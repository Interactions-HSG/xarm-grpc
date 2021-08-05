#!/bin/bash

make -C lib/xArm-CPLUS-SDK install
clang-tidy "$@"
