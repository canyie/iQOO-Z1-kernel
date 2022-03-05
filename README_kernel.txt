################################################################################

1. How to Build
- get Toolchain
From android git server, codesourcery and etc ..
- aarch64-linux-android-4.9
- (git clone https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9 -b brillo-m9-release)

- put the gcc in the right path.
put the aarch64-linux-android-4.9 folder in the $(kernel directory)/ path

$ ./build_kernel.sh

2. Output files
- Kernel : $(kernel directory)/out/arch/arm64/boot/Image.gz-dtb

3. How to Clean
$ rm -rf $(kernel directory)/out
################################################################################
