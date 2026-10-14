set(BOOSTER_ROOT_DIR "${PROJECT_ROOT_PREFIX}/Src/Apps/Booster")
set(BOOSTER_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/Linux/Booster/$<CONFIG>")

set(BOOSTER_SOURCES
    "${BOOSTER_ROOT_DIR}/Main.cpp")

if(BUILD_BOOSTER)
  if(NOT MACOS AND NOT (LINUX AND ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang))
    message(ERROR "The target Booster has to be built for Linux and with clang.")
  endif()

  add_executable(Booster ${BOOSTER_SOURCES})
  set_property(TARGET Booster PROPERTY RUNTIME_OUTPUT_DIRECTORY "${BOOSTER_OUTPUT_DIR}")
  set_property(TARGET Booster PROPERTY RUNTIME_OUTPUT_NAME roboeireann)
  target_link_libraries(Booster PRIVATE RobotCore${TARGET_SUFFIX})
  target_link_libraries(Booster PRIVATE Platform${TARGET_SUFFIX})
  target_link_libraries(Booster PRIVATE Math${TARGET_SUFFIX})
  target_link_libraries(Booster PRIVATE Streaming${TARGET_SUFFIX})
  target_link_libraries(Booster PRIVATE RobotParts${TARGET_SUFFIX})
  target_link_libraries(Booster PRIVATE Framework${TARGET_SUFFIX})
  target_link_libraries(Booster PRIVATE Booster::Booster::SDK)
  target_link_libraries(Booster PRIVATE Booster::FastCDR::FastCDR)
  target_link_libraries(Booster PRIVATE Booster::FastRTPS::FastRTPS)
  target_link_libraries(Booster PRIVATE Booster::FooNathan::Memory)
  target_link_libraries(Booster PRIVATE Booster::WakeupAPI::WakeupAPI)
  target_link_libraries(Booster PRIVATE Booster::TensorRT::NvInfer)
  target_link_libraries(Booster PRIVATE Booster::CUDA::Runtime)
  target_include_directories(Booster SYSTEM PRIVATE
      "${PROJECT_ROOT_PREFIX}/Util/cuda-arm64/include")

  target_compile_options(Booster PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
  if(MACOS)
    set_property(TARGET Booster PROPERTY XCODE_ATTRIBUTE_LDPLUSPLUS_arm64 "${CMAKE_CURRENT_SOURCE_DIR}/../../Util/Buildchain/macOS/bin/link")
    set_property(TARGET Booster PROPERTY FOLDER Apps)
    source_group(TREE "${BOOSTER_ROOT_DIR}" FILES ${BOOSTER_SOURCES})
  endif()

  target_link_libraries(Booster PRIVATE Flags::Default)
  if(MACOS)
    add_dependencies(Booster DeployDialog)
  endif()
elseif(NOT BUILD_NAO)
  if(WINDOWS)
    set(CMAKE_MSVCIDE_RUN_PATH "%systemroot%\\Sysnative")
    add_custom_target(Booster ALL bash -c "cd Linux/CMake/$<CONFIG>/Booster-prefix; stdbuf -e0 -oL cmake --build . --target Booster | stdbuf -e0 -oL sed 's@^/mnt/\\([a-z]\\)@\\U\\1:@'"
        WORKING_DIRECTORY "${OUTPUT_PREFIX}/Build"
        SOURCES ${BOOSTER_SOURCES})
    add_custom_command(TARGET Booster PRE_BUILD COMMAND bash -c "rm -f Linux/Booster/$<CONFIG>/success" WORKING_DIRECTORY "${OUTPUT_PREFIX}/Build")
    add_custom_command(TARGET Booster POST_BUILD COMMAND bash -c "touch Linux/Booster/$<CONFIG>/success" WORKING_DIRECTORY "${OUTPUT_PREFIX}/Build")
    set_property(TARGET Booster PROPERTY FOLDER Apps)

    source_group(TREE "${BOOSTER_ROOT_DIR}" FILES ${BOOSTER_SOURCES})
  else()
    include(ExternalProject)
    ExternalProject_Add(Booster
        SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
        CMAKE_ARGS -DCMAKE_BUILD_TYPE=$<CONFIG>
                   -DBUILD_BOOSTER=ON
                   -DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/../CMake/BoosterToolchain.cmake
        BUILD_ALWAYS 1
        USES_TERMINAL_BUILD ON
        INSTALL_COMMAND "")
    set_property(TARGET Booster PROPERTY FOLDER "")
    add_dependencies(Booster DeployDialog)
  endif()
endif()
