# Motion Capture Spray

C code using the ESP-IDF framework for the motion capture device. Includes classic Bluetooth and an MPU6050 using a GY521 breakout board following I2C protocol. 

In order to use the PlatformIO packages on MacOSX, run a virtual environment based on an Intel x86-64 architecture since there isn't enough support for ARM-based chips. 

    >>> source ~/platformio-intel/bin/activate
   
 In order to create and Intel based virtual environment, the following command can be run a venv has already been created (this step isn't necessary, it is just here for documentation).

    >>> arch -x86_64 /usr/bin/python3 venv ~/platformio-intel

To make/run the file, do these three commands in order to clean, compile, and upload the code using the PlatformIO CLI.

    >>> pio run -t clean
    >>> pio run
    >>> pio run -t upload

Within the `src` folder, there are multiple files, for each of the different things that the ESP32 is keeping track of, except for a dedicated Bluetooth file. Each file is there to debug the different components that the ESP32 is connected to. In order to switch file that is being run, change the following line in `src/CMakeLists.txt` to the filename you want in the parameter `[FILENAME]`.

    idf_component_register(SRCS "[FILENAME]" INCLUDE_DIRS ".")

## Bluetooth


