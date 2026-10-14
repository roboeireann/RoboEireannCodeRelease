# RoboEireann code release 2026

This is the RoboEireann code release 2026, based on the [B-Human Code Release 2025](https://github.com/bhuman/BHumanCodeRelease/tree/coderelease2025) 
with RoboEireann specific modules and features added. The code release is a very lightly cleaned up version of code used at RoboCup 2026.

This release is a complete replacement for our earlier releases, which were based 
on the B-Human Code Release 2021. The code base has changed substantially, 
so there is no straightforward upgrade path from those versions.

## Libraries to copy from the robot (before building)

The CUDA and TensorRT include files and libraries for ARM64 are not part of this
repository. After cloning, and before building, you must copy them from a robot:

1. Create two subdirectories of `Util`: `Util/cuda-arm64` and `Util/tensorrt-arm64`.
2. Copy the CUDA include files from the robot into `Util/cuda-arm64/include`
   and the CUDA libraries into `Util/cuda-arm64/lib`.
3. Copy the TensorRT include files from the robot into `Util/tensorrt-arm64/include`
   and the TensorRT libraries into `Util/tensorrt-arm64/lib/LinuxArm64`.

The build will fail if these directories are missing or empty.

## Building and running

After completing the step above, to install, build, and run the software, the 
instructions at the [B-Human wiki](https://docs.b-human.de/coderelease2025/) 
for Linux generally apply.

- **Supported platforms:** Ubuntu 22.04 and Ubuntu 24.04 only.
- **macOS and Windows:** the build options for these platforms have not been removed, but building on them is not supported. They have not been updated to match this release and will not work without modification.
- **IDEs:** we normally use command line tools to build and run the software, so we do not guarantee that the integration of build or run related utilities works with any given IDE.

## Licence and acknowledgments

See [Licence](Licence.md) for the terms that apply to RoboEireann's contributions. Files derived from B-Human or Nao Devils software remain subject to their licences (`LicenseBHuman.md`, `LicenseNaoDevils.txt`). Parts of this distribution from other third parties keep their own licences.

This product includes software developed by B-Human (https://www.b-human.de).

This product includes software developed by Nao Devils (https://naodevils.de).

This product includes software developed by RoboÉireann (http://www.roboeireann.ie).