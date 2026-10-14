set(RE_HYBRID_BEHAVIOUR_ROOT_DIR "${PROJECT_ROOT_PREFIX}/Src/Modules/REHybridBehaviour")

file(GLOB_RECURSE RE_HYBRID_BEHAVIOUR_SOURCES CONFIGURE_DEPENDS
    "${RE_HYBRID_BEHAVIOUR_ROOT_DIR}/CommonControl/*.cpp" "${RE_HYBRID_BEHAVIOUR_ROOT_DIR}/CommonControl/*.h"
    "${RE_HYBRID_BEHAVIOUR_ROOT_DIR}/Control/*.cpp" "${RE_HYBRID_BEHAVIOUR_ROOT_DIR}/Control/*.h"
    "${RE_HYBRID_BEHAVIOUR_ROOT_DIR}/Skills/*.cpp" "${RE_HYBRID_BEHAVIOUR_ROOT_DIR}/Skills/*.h")

set(RE_HYBRID_BEHAVIOUR_TREE "${RE_HYBRID_BEHAVIOUR_SOURCES}")

# Prevent any file/folder beginning with "IGNORE_" from being included in the build
list(FILTER RE_HYBRID_BEHAVIOUR_SOURCES EXCLUDE REGEX ".*/IGNORE_.*")


set(RE_HYBRID_BEHAVIOUR_PCHS
    [["REHybridBehaviourModule.h"]])

# while debugging the macros
set_source_files_properties(
    "${RE_HYBRID_BEHAVIOUR_ROOT_DIR}/CommonControl/HandlePlayerStateTask.cpp"
    PROPERTIES SKIP_PRECOMPILE_HEADERS ON
             COMPILE_OPTIONS "-save-temps=obj"
)

add_library(REHybridBehaviourTasks${TARGET_SUFFIX} OBJECT ${RE_HYBRID_BEHAVIOUR_SOURCES})
set_property(TARGET REHybridBehaviourTasks${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
set_property(TARGET REHybridBehaviourTasks${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(REHybridBehaviourTasks${TARGET_SUFFIX} PRIVATE "${PROJECT_ROOT_PREFIX}/Src")
target_include_directories(REHybridBehaviourTasks${TARGET_SUFFIX} PRIVATE "${RE_HYBRID_BEHAVIOUR_ROOT_DIR}")
target_link_libraries(REHybridBehaviourTasks${TARGET_SUFFIX} PRIVATE Platform${TARGET_SUFFIX})
target_link_libraries(REHybridBehaviourTasks${TARGET_SUFFIX} PRIVATE MathBase${TARGET_SUFFIX})
target_link_libraries(REHybridBehaviourTasks${TARGET_SUFFIX} PRIVATE Streaming${TARGET_SUFFIX})
target_link_libraries(REHybridBehaviourTasks${TARGET_SUFFIX} PRIVATE Math${TARGET_SUFFIX})
target_link_libraries(REHybridBehaviourTasks${TARGET_SUFFIX} PRIVATE RobotParts${TARGET_SUFFIX})
target_link_libraries(REHybridBehaviourTasks${TARGET_SUFFIX} PRIVATE Debugging${TARGET_SUFFIX})
target_link_libraries(REHybridBehaviourTasks${TARGET_SUFFIX} PRIVATE Framework${TARGET_SUFFIX})

if(BUILD_DESKTOP)
  set_property(TARGET REHybridBehaviourTasks${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(REHybridBehaviourTasks${TARGET_SUFFIX} PRIVATE Flags::DebugInDevelop)
else()
  target_compile_definitions(REHybridBehaviourTasks${TARGET_SUFFIX} PUBLIC TARGET_ROBOT)
  target_compile_options(REHybridBehaviourTasks${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG> $<$<CONFIG:Release>:-Wno-unused>)
  target_link_libraries(REHybridBehaviourTasks${TARGET_SUFFIX} PRIVATE Flags::Default)
endif()

target_precompile_headers(REHybridBehaviourTasks${TARGET_SUFFIX} PRIVATE ${RE_HYBRID_BEHAVIOUR_PCHS})

if(MSVC)
  target_compile_options(REHybridBehaviourTasks${TARGET_SUFFIX} PUBLIC /Zm200)
endif()

source_group(TREE "${RE_HYBRID_BEHAVIOUR_ROOT_DIR}" FILES ${RE_HYBRID_BEHAVIOUR_TREE})
