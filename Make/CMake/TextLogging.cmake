set(TEXT_LOGGING_ROOT_DIR "${PROJECT_ROOT_PREFIX}/Src/Libs/TextLogging")

set(TEXT_LOGGING_SOURCES
    "${TEXT_LOGGING_ROOT_DIR}/TextLogging.h"
    "${TEXT_LOGGING_ROOT_DIR}/TextLoggingStream.h"
    "${TEXT_LOGGING_ROOT_DIR}/TextLogging.cpp"
)

add_library(TextLogging_Objs${TARGET_SUFFIX} OBJECT ${TEXT_LOGGING_SOURCES})
set_property(TARGET TextLogging_Objs${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")

if(BUILD_DESKTOP)
  set_property(TARGET TextLogging_Objs${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(TextLogging_Objs${TARGET_SUFFIX} PRIVATE Flags::DebugInDevelop)
else()
  target_compile_definitions(TextLogging_Objs${TARGET_SUFFIX} PRIVATE TARGET_ROBOT)
  target_compile_options(TextLogging_Objs${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG>)
  target_link_libraries(TextLogging_Objs${TARGET_SUFFIX} PRIVATE Flags::Default)
endif()

# for fmtlib style formatting of text logging
if(BUILD_NAO)
  target_link_libraries(TextLogging_Objs${TARGET_SUFFIX} PRIVATE Nao::fmt::fmt)
elseif(BUILD_BOOSTER)
  target_link_libraries(TextLogging_Objs${TARGET_SUFFIX} PRIVATE Booster::fmt::fmt)
else()
  target_link_libraries(TextLogging_Objs${TARGET_SUFFIX} PRIVATE fmt::fmt)
endif()


# for assert and async thread support
# Note - we cannot just link to platform directly or cmake complains of a circular dependency
#target_link_libraries(TextLogging_Objs${TARGET_SUFFIX} PRIVATE Platform${TARGET_SUFFIX})

# Use PUBLIC so that when these objects are added to Platform, 
# the paths propagate to Platform's users.
target_include_directories(TextLogging_Objs${TARGET_SUFFIX} PUBLIC "${TEXT_LOGGING_ROOT_DIR}/..")

# IDE Source Grouping
source_group(TREE "${TEXT_LOGGING_ROOT_DIR}" FILES ${TEXT_LOGGING_SOURCES})
