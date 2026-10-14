# --- CatchTests.cmake ---

message(STATUS "--- Entering CatchTests.cmake ---")
message(STATUS "  BUILD_DESKTOP: ${BUILD_DESKTOP}")
message(STATUS "  BUILD_NAO: ${BUILD_NAO}")
message(STATUS "  TARGET_SUFFIX: ${TARGET_SUFFIX}")
message(STATUS "  Platform: ${PLATFORM} Arch: ${ARCH}")

if(TARGET CatchTestsNao)
   message(FATAL_ERROR "Target CatchTestsNao already exists. Type: ${CMAKE_TARGET_TYPE}")
endif()


# --- CatchTests.cmake ---

# source files for our test code
set(CATCH_TESTS_ROOT "${PROJECT_ROOT_PREFIX}/Src/Apps/CatchTests")
file(GLOB_RECURSE CATCH_SOURCES
  "${CATCH_TESTS_ROOT}/*.cpp"
  "${CATCH_TESTS_ROOT}/*.h"
)

# Prevent any file/folder beginning with "IGNORE_" from being included in the build
list(FILTER CATCH_SOURCES EXCLUDE REGEX ".*/IGNORE_.*")


# 1. Internal logic to ensure unique executable naming
set(CATCH_EXE_SUFFIX "")

set(SHOULD_BUILD_CATCH_TESTS FALSE)
set(CATCH_EXE_BASE_NAME "CatchTests")

if(BUILD_DESKTOP)
  message(STATUS "  desktop pass")
  set(SHOULD_BUILD_CATCH_TESTS TRUE)
  set(CATCH_NAMESPACE "Catch2")
  set(CATCH_LIB_DIR "${PLATFORM}${ARCH}")
  set(CATCH_EXE_NAME ${CATCH_EXE_BASE_NAME})
elseif(BUILD_NAO)
  message(STATUS "  Nao robot pass")
  set(SHOULD_BUILD_CATCH_TESTS TRUE)
  set(CATCH_NAMESPACE "Nao::Catch2")
  set(CATCH_LIB_DIR "V6")
  set(CATCH_EXE_NAME "${CATCH_EXE_BASE_NAME}Nao")
elseif(BUILD_BOOSTER)
  message(STATUS "  Booster robot pass")
  set(SHOULD_BUILD_CATCH_TESTS TRUE)
  set(CATCH_NAMESPACE "Booster::Catch2")
  set(CATCH_LIB_DIR "LinuxArm64")
  set(CATCH_EXE_NAME "${CATCH_EXE_BASE_NAME}Booster")
endif()

# 2. Add the executable
if(SHOULD_BUILD_CATCH_TESTS AND CATCH_SOURCES)
  if(NOT TARGET ${CATCH_NAMESPACE}::Catch2)
    add_library(${CATCH_NAMESPACE}::Catch2 STATIC IMPORTED)
    set_target_properties(${CATCH_NAMESPACE}::Catch2 PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_ROOT_PREFIX}/Util/Catch2/include"
      IMPORTED_LOCATION "${PROJECT_ROOT_PREFIX}/Util/Catch2/lib/${CATCH_LIB_DIR}/libCatch2.a")

    # The Runner (libCatch2Main.a)
    # create a 'WithMain' target that depends on the core library.
    add_library(${CATCH_NAMESPACE}::Catch2WithMain STATIC IMPORTED)
    set_target_properties(${CATCH_NAMESPACE}::Catch2WithMain PROPERTIES
      IMPORTED_LOCATION "${PROJECT_ROOT_PREFIX}/Util/Catch2/lib/${CATCH_LIB_DIR}/libCatch2Main.a"
      INTERFACE_LINK_LIBRARIES "${CATCH_NAMESPACE}::Catch2")
  endif()

  message(STATUS "  adding executable: ${CATCH_EXE_NAME}")

  if (TARGET ${CATCH_EXE_NAME})
    message(FATAL_ERROR "target ${CATCH_EXE_NAME} is already defined")
  endif()

  add_executable(${CATCH_EXE_NAME} EXCLUDE_FROM_ALL ${CATCH_SOURCES})

  # Isolation via Output Directory
  set(CATCH_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/${PLATFORM}/${CATCH_EXE_NAME}/$<CONFIG>")
  set_target_properties(${CATCH_EXE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CATCH_OUTPUT_DIR}")
  set_property(TARGET ${CATCH_EXE_NAME} PROPERTY FOLDER Apps)

  # 3. Linking Dependencies
  # Here we use TARGET_SUFFIX to remain consistent with Nao.cmake logic.
  # On Linux: TARGET_SUFFIX is empty, so we link RobotCore.
  # On macOS: TARGET_SUFFIX is "Nao", so we link RobotCoreNao.
  target_link_libraries(${CATCH_EXE_NAME} PRIVATE
    RobotCore${TARGET_SUFFIX}
    Framework${TARGET_SUFFIX}
    Math${TARGET_SUFFIX}
    Platform${TARGET_SUFFIX}
    Streaming${TARGET_SUFFIX}
    ${CATCH_NAMESPACE}::Catch2WithMain
  )

  if(BUILD_DESKTOP)
    target_link_libraries(${CATCH_EXE_NAME} PRIVATE Flags::DebugInDevelop)
  else()
    target_compile_definitions(${CATCH_EXE_NAME} PRIVATE TARGET_ROBOT)
    target_compile_options(${CATCH_EXE_NAME} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
    target_link_libraries(${CATCH_EXE_NAME} PRIVATE Flags::Default)
  endif()

  # 4. Proxy Targets (defined during desktop Pass Only)
  if(BUILD_DESKTOP AND NOT APPLE)
    foreach(ROBOT Nao Booster)
      set(CATCH_PROXY_NAME CatchTests${ROBOT})
      set(CATCH_ROBOT_BUILD_DIR "Linux/CMake/$<CONFIG>/${ROBOT}-prefix/src/${ROBOT}-build")

      message(STATUS "  adding proxy: ${CATCH_PROXY_NAME}")

      add_custom_target(${CATCH_PROXY_NAME}
        COMMAND ${CMAKE_COMMAND} -E chdir "${CATCH_ROBOT_BUILD_DIR}" ${CMAKE_COMMAND} --build . --target ${CATCH_PROXY_NAME}
        WORKING_DIRECTORY "${OUTPUT_PREFIX}/Build"
        SOURCES ${CATCH_SOURCES}
        VERBATIM
        USES_TERMINAL
      )      

      if(TARGET ${ROBOT})
        add_dependencies(CatchTests${ROBOT} ${ROBOT})
      endif()
    endforeach()
  endif()
endif()

message(STATUS "--- Exiting CatchTests.cmake ---")
