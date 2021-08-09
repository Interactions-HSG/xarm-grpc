#!/bin/bash

make -C libs/xArm-CPLUS-SDK install
clang-tidy "$@"
