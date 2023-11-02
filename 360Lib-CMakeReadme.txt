NextSoftware360 build howto:

The software uses cmake to create the needed build files. 
Download cmake: http://www.cmake.org/ and install it.

Usage:

Open a command prompt on your system and change into the root directory
of this project (location of README.txt).

Create a build directory in the root directory:
mkdir build 

After that use one of the following cmake commands. Feel free to change the 
commands to satisfy your needs.

Windows sample for Visual Studio 2019 64 Bit:
cd build
cmake .. -DEXTENSION_360_VIDEO=1 -G "Visual Studio 16 2019"

Windows sample for Visual Studio 2015 64 Bit:
cd build
cmake .. -DEXTENSION_360_VIDEO=1 -G "Visual Studio 14 2015 Win64"

Linux Release Makefile sample:
cd build
cmake .. -DEXTENSION_360_VIDEO=1 -DCMAKE_BUILD_TYPE=Release

Linux Debug Makefile sample:
cd build
cmake .. -DEXTENSION_360_VIDEO=1 -DCMAKE_BUILD_TYPE=Debug

MACOSX Xcode sample:
cd build
cmake .. -DEXTENSION_360_VIDEO=1 -G "Xcode"
