# RoboEireann Integration

This folder contains a pre-compiled version of the `{fmt}` formatting library.
(Version 12.1 as of 8 Jan 2026.)

## Directory Structure

* `include/`        # fmt headers (Common for all platforms)
* `lib/Linux/`      # Ubuntu Desktop (x86_64, Clang 17) or WSL
* `lib/V6/`         # Nao Robot (Silvermont, Clang 17, Cross-compiled)
* `lib/LinuxArm64/` # Booster Robot (ARM, Cross-compiled)
* `lib/macOS/`      # macOS (Universal or x86_64)
* `lib/macOSArm64/` # macOS Apple silicon I guess
* `lib/Windows/`    # Windows

## How to Recreate

Download/Clone the fmt source from [https://github.com/fmtlib/fmt](https://github.com/fmtlib/fmt) 
into a directory **outside** the codebase, e.g. as a subdirectory of 
the `third_party` directory.

### 1. Preparing the reboost project

Assume that <PROJECT_ROOT_DIR> refers to the root directory of the project under
which `Src`, `Util` etc exists.

Create the directory structure:

```bash
mkdir -p <PROJECT_ROOT_DIR>/Util/fmt/include

mkdir -p <PROJECT_ROOT_DIR>/Util/fmt/lib/Linux
mkdir -p <PROJECT_ROOT_DIR>/Util/fmt/lib/V6
mkdir -p <PROJECT_ROOT_DIR>/Util/fmt/lib/LinuxArm64
# TODO other directories for macOS and Windows

```

Change directory into the the external `fmt` directory. Then copy the headers 
from the external `fmt` source to reboost:

```bash
cp -r include/fmt <PROJECT_ROOT_DIR>/Util/fmt/include/

```

### 2. Adding Linux (desktop) support

In the external `fmt` base directory:

```bash
mkdir build_desktop && cd build_desktop
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DFMT_TEST=OFF -DFMT_FUZZ=OFF -DFMT_DOC=OFF -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_COMPILER=<YOUR_COMPILER>
make -j$(nproc)
cp libfmt.a <PROJECT_ROOT_DIR>/Util/fmt/lib/Linux/

```

Note: replace <YOUR_COMPILER> in the line above to make 
   CMake point at the correct compiler version (must be clang 16 or higher, 
   e.g. use `clang++` if your default version is high enough, otherwise specify 
   another compiler on your system, e.g. `clang++-17`)


### 3. Adding Nao support (V6)

Start again from the external `fmt` base directory:

```bash
mkdir build_nao && cd build_nao
cmake .. \
  -DCMAKE_TOOLCHAIN_FILE=<PROJECT_ROOT_DIR>/Make/CMake/NaoToolchain.cmake \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -fomit-frame-pointer" -DFMT_TEST=OFF -DFMT_DOC=OFF \
  -DCMAKE_CXX_COMPILER=<YOUR_COMPILER>
make -j$(nproc)
cp libfmt.a <PROJECT_ROOT_DIR>/Util/fmt/lib/V6/

```

### 4. Adding Booster support (ARM)

Start again from the external `fmt` base directory:

```bash
mkdir build_booster && cd build_booster
cmake .. \
  -DCMAKE_TOOLCHAIN_FILE=<PROJECT_ROOT_DIR>/Make/CMake/BoosterToolchain.cmake \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -fomit-frame-pointer" -DFMT_TEST=OFF -DFMT_DOC=OFF
make -j$(nproc)
cp libfmt.a <PROJECT_ROOT_DIR>/Util/fmt/lib/ARM/

```

---

## CMake Integration Logic

To use this library in the reboost project, add the following to the main `CMakeLists.txt` (following the FFTW pattern) and update the library links (assuming this has not already been done).

### Main `CMakeLists.txt`

Add this in the 3rd-party library section:

```cmake
# --- fmtlib Integration ---
if(BUILD_NAO)
  add_library(Nao::fmt::fmt STATIC IMPORTED GLOBAL)
  set_target_properties(Nao::fmt::fmt PROPERTIES
    IMPORTED_LOCATION "${PROJECT_ROOT_PREFIX}/Util/fmt/lib/V6/libfmt.a"
    INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_ROOT_PREFIX}/Util/fmt/include")
endif()

if(BUILD_BOOSTER)
  add_library(Booster::fmt::fmt STATIC IMPORTED GLOBAL)
  set_target_properties(Booster::fmt::fmt PROPERTIES
    IMPORTED_LOCATION "${PROJECT_ROOT_PREFIX}/Util/fmt/lib/LinuxArm64/libfmt.a"
    INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_ROOT_PREFIX}/Util/fmt/include")
endif()

if(BUILD_DESKTOP)
  if (LINUX OR MACOS)
    add_library(fmt::fmt STATIC IMPORTED GLOBAL)
    set_target_properties(fmt::fmt PROPERTIES
      IMPORTED_LOCATION "${PROJECT_ROOT_PREFIX}/Util/fmt/lib/${PLATFORM}${ARCH}/libfmt.a"
      INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_ROOT_PREFIX}/Util/fmt/include")
  else() # for WINDOWS
    message(FATAL_ERROR "Windows support for fmt lib must be added")
  endif()
endif()

```

### `TextLogging.cmake`

Update the target link to use the alias:

```cmake
if(BUILD_NAO)
  target_link_libraries(TextLogging_Objs${TARGET_SUFFIX} PRIVATE Nao::fmt::fmt)
elseif(BUILD_BOOSTER)
  target_link_libraries(TextLogging_Objs${TARGET_SUFFIX} PRIVATE Booster::fmt::fmt)
else()
  target_link_libraries(TextLogging_Objs${TARGET_SUFFIX} PRIVATE fmt::fmt)
endif()

```

### `Platform.cmake`

Change the dependency to `PUBLIC` so headers propagate to the final apps:

```cmake
target_link_libraries(Platform${TARGET_SUFFIX} PUBLIC TextLogging_Objs${TARGET_SUFFIX})

```
