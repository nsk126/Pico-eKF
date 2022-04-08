# PicoEKF using 2 Cores and RTOS threads

You will need to include the FreeRTOS-Kernel in your directory to build the program.

`git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git`

1. Add `FreeRTOSConfig.h` file in `include/` folder.
2. include `pico_sdk_import.cmake` in main directory and include in `CMakeLists.txt`.
3. Or set the path of `pico_sdk_import.cmake` to an ENV variable and call it in `CMakeLists.txt`.

## Dependencies

1. Download Eigen-3.4.0 to your project directory.
    - https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
