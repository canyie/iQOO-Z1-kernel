#!/bin/bash
PATH=`pwd`/aarch64-linux-android-4.9/bin:$PATH
mkdir -p out
make -j32 O=out ARCH=arm64 CROSS_COMPILE=aarch64-linux-android- k6889v1_64_defconfig
make -j32 O=out ARCH=arm64 CROSS_COMPILE=aarch64-linux-android-
