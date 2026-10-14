# --- SmallApps.cmake ---

# The purpose of this file is to allow small multi-platform apps such as 
# prototypes, benchmarks, simple tools, etc to be easily added to the make
# structure. For each app you can decide which platforms it supports by 
# adding the selected values from DESKTOP NAO BOOSTER. E.g. if you specify
#     add_small_app(myapp MY_APP_SOURCES DESKTOP BOOSTER)
# then the corresponding myapp will only be exist for desktop or booster targets

function(add_small_app APP_BASE_NAME SOURCES)
  # Define the supported platforms for this specific app
  set(options DESKTOP NAO BOOSTER)
  cmake_parse_arguments(APP "${options}" "" "" ${ARGN})

  set(SHOULD_BUILD_APP FALSE)
  set(APP_EXE_NAME ${APP_BASE_NAME})

  # 1. Platform Check Logic
  if(BUILD_DESKTOP AND APP_DESKTOP)
    set(SHOULD_BUILD_APP TRUE)
  elseif(BUILD_NAO AND APP_NAO)
    set(SHOULD_BUILD_APP TRUE)
    set(APP_EXE_NAME "${APP_BASE_NAME}Nao")
  elseif(BUILD_BOOSTER AND APP_BOOSTER)
    set(SHOULD_BUILD_APP TRUE)
    set(APP_EXE_NAME "${APP_BASE_NAME}Booster")
  endif()

  # 2. Add Executable and Isolate Directory
  if(SHOULD_BUILD_APP)
    add_executable(${APP_EXE_NAME} EXCLUDE_FROM_ALL ${${SOURCES}})

    # Matches the structure required by your updated deploy script
    set(APP_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/${PLATFORM}/${APP_EXE_NAME}/$<CONFIG>")
    set_target_properties(${APP_EXE_NAME} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${APP_OUTPUT_DIR}"
      FOLDER Apps
    )

    # 3. Create Desktop Proxies (Only if the app supports the target robot)
    if(BUILD_DESKTOP AND NOT APPLE)
      foreach(ROBOT Nao Booster)
        string(TOUPPER ${ROBOT} ROBOT_UPPER)

        # Only create the proxy if the app actually supports that robot
        if(APP_${ROBOT_UPPER})
          set(APP_PROXY_NAME "${APP_BASE_NAME}${ROBOT}")
          set(ROBOT_BUILD_DIR "Linux/CMake/$<CONFIG>/${ROBOT}-prefix/src/${ROBOT}-build")

          if(NOT TARGET ${APP_PROXY_NAME})
            add_custom_target(${APP_PROXY_NAME}
              COMMAND ${CMAKE_COMMAND} -E chdir "${ROBOT_BUILD_DIR}" ${CMAKE_COMMAND} --build . --target ${APP_PROXY_NAME}
              WORKING_DIRECTORY "${OUTPUT_PREFIX}/Build"
              SOURCES ${${SOURCES}}
              VERBATIM
              USES_TERMINAL
            )

            if(TARGET ${ROBOT})
              add_dependencies(${APP_PROXY_NAME} ${ROBOT})
            endif()
          endif()
        endif()
      endforeach()
    endif()

    # Export the EXE name to the parent scope so we can link libraries outside the function
    set(CURRENT_APP_EXE ${APP_EXE_NAME} PARENT_SCOPE)
  else()
    unset(CURRENT_APP_EXE PARENT_SCOPE)
  endif()
endfunction()

# ==============================================================================
# APP DEFINITIONS
# ==============================================================================

# --- BenchmarkTextLogging (All Platforms) ---
set(BENCH_TEXTLOG_ROOT "${PROJECT_ROOT_PREFIX}/Src/Apps/SmallApps/BenchmarkTextLogging")
file(GLOB BENCH_TEXTLOG_SOURCES "${BENCH_TEXTLOG_ROOT}/*.cpp" "${BENCH_TEXTLOG_ROOT}/*.h")

add_small_app(BenchmarkTextLogging BENCH_TEXTLOG_SOURCES DESKTOP NAO BOOSTER) # all platforms supported

if(CURRENT_APP_EXE)
  target_link_libraries(${CURRENT_APP_EXE} PRIVATE
    RobotCore${TARGET_SUFFIX}
    Platform${TARGET_SUFFIX}
    Streaming${TARGET_SUFFIX}
  )

  if(BUILD_DESKTOP)
    target_link_libraries(${CURRENT_APP_EXE} PRIVATE Flags::DebugInDevelop)
  else()
    target_compile_definitions(${CURRENT_APP_EXE} PRIVATE TARGET_ROBOT)
    target_compile_options(${CURRENT_APP_EXE} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
    target_link_libraries(${CURRENT_APP_EXE} PRIVATE Flags::Default)
  endif()

endif()
