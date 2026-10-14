# RoboEireann Integration: Catch2 v3.11.0

This folder contains a flattened version of the Catch2 unit testing framework.

## Directory Structure
- `include/`       # Catch2 headers (Common for all platforms)
- `lib/Linux/`     # Ubuntu Desktop (x86_64, created with Clang 17)
- `lib/V6/`        # Nao Robot (Silvermont, created with Clang 17, Cross-compiled)
- `lib/macOS/`    # [PLACEHOLDER] for macOS
- `lib/Windows/`     # [PLACEHOLDER] for Windows/WSL

## How to Recreate

Clone Catch2 from https://github.com/catchorg/Catch2 into a suitable directory
**outside** the reboost codebase, e.g. ~/third_party/Catch2. We'll refer to this
as the external Catch2 project.

### Preparing the reboost project

1. Create a directory for the minimal Catch2 framework libraries etc in the reboost project
   (replacing <REBOOST_PROJECT_DIR> with the appropriate path)

   ```bash
   mkdir <REBOOST_PROJECT_DIR>/Util/Catch2
   ```

2. Change directory into the external Catch2 project directory (where you cloned the Catch2)
   source and copy the `README.md` and `LICENSE.txt` files to 
   `<REBOOST_PROJECT_DIR>/Util/Catch2` (replacing as usual)

   ```bash
   cp README.md LICENSE.txt <REBOOST_PROJECT_DIR>/Util/Catch2
   ```

### Adding Linux (desktop) support

1. Checkout the relevant release tag

   ```bash
   cd Catch2
   git checkout v3.11.0
   ```

2. Make a `build_desktop` directory, replace <YOUR_COMPILER> below to make 
   CMake point at the correct compiler version (must be clang 16 or higher, 
   e.g. use `clang++` if your default version is high enough, otherwise specify 
   another compiler on your system, e.g. `clang++-17`), then initialise the
   `build_desktop` directory 

   ```bash
   mkdir build_desktop && cd build_desktop
   cmake .. -DCMAKE_CXX_COMPILER=<YOUR_COMPILER> \
     -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=./install_temp
   ```

3. Build the libraries, prepare the installation setup, copy the files into the
   reboost project (replacing <REBOOST_PROJECT_DIR> with the appropriate path)

   ```bash
   make -j$(nproc)
   make install
   cp -r install_temp/include/* <REBOOST_PROJECT_DIR>/Util/Catch2/include/
   cp install_temp/lib/*.a <REBOOST_PROJECT_DIR>/Util/Catch2/lib/Linux/
   ```

### Adding Nao support

1. Starting again from the `Catch2` directory (not the `build_desktop` directory),
   make a `build_nao` sub-directory and initialise it (replacing <REBOOST_PROJECT_DIR>
   with the appropriate path)

   ```bash
   mkdir build_nao && cd build_nao
   cmake .. \
     -DCMAKE_TOOLCHAIN_FILE=<REBOOST_PROJECT_DIR>/Make/CMake/NaoToolchain.cmake \
     -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=./install_temp
   ```

2. Build the libraries and prepare the installation setup. Copy just the library 
   files (the includes were already dealt with under desktop support) into the
   reboost project (replacing <REBOOST_PROJECT_DIR> with the appropriate path)

   ```bash
   make -j$(nproc)
   make install
   cp install_temp/lib/*.a <REBOOST_PROJECT_DIR>/Util/Catch2/lib/V6/
   ```

### Adding Booster support

1. Starting again from the `Catch2` directory (not the `build_desktop` directory),
   make a `build_booster` sub-directory and initialise it (replacing <REBOOST_PROJECT_DIR>
   with the appropriate path)

   ```bash
   mkdir build_booster && cd build_booster
   cmake .. \
     -DCMAKE_TOOLCHAIN_FILE=<REBOOST_PROJECT_DIR>/Make/CMake/BoosterToolchain.cmake \
     -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX=./install_temp
   ```

2. Build the libraries and prepare the installation setup. Copy just the library 
   files (the includes were already dealt with under desktop support) into the
   reboost project (replacing <REBOOST_PROJECT_DIR> with the appropriate path)

   ```bash
   make -j$(nproc)
   make install
   cp install_temp/lib/*.a <REBOOST_PROJECT_DIR>/Util/Catch2/lib/LinuxArm64/
   ```
