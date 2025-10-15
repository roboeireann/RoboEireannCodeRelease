# Version

This is tag v3.7.1 from the the Catch2 repo https://github.com/catchorg/Catch2/tree/v3.7.1

# Prepare contents of rematch/Util/Catch2

    git clone git@github.com:catchorg/Catch2.git
    git checkout v3.7.1

Change into the base Catch2 directory and create the build directory

    cd Catch2
    cmake -S . -B build

Create an install directory (still from the base Catch2 directory)

    mkdir install
    cmake --install build --prefix install

Copy the relevant files to Util

- copy LICENSE.txt and README.md to rematch/Util/Catch2
- copy Catch2/install/include to rematch/Util/Catch2
- copy Catch2/install/lib to rematch/Util/Catch2
- move rematch/Util/Catch2/lib/*.a to new directory rematch/Util/Catch2/lib/Linux

