# WARNING: no apple support has been added/implemented for catch2 tests 

set(TESTS_ROOT_DIR "${PROJECTROOT_PREFIX}/Src")
set(TESTS_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/${OS}/Tests/$<CONFIG>")

file(GLOB TESTS_COMMON_SOURCES
    "${TESTS_ROOT_DIR}/Platform/${OS}/*.cpp" "${TESTS_ROOT_DIR}/Platform/${OS}/*.h" "${TESTS_ROOT_DIR}/Platform/${OS}/*.mm"
    "${TESTS_ROOT_DIR}/Platform/*.cpp" "${TESTS_ROOT_DIR}/Platform/*.h"
    "${TESTS_ROOT_DIR}/Tools/*.cpp" "${TESTS_ROOT_DIR}/Tools/*.h"
    "${TESTS_ROOT_DIR}/Tools/Debugging/TimingManager.cpp" "${TESTS_ROOT_DIR}/Tools/Debugging/TimingManager.h"
    "${TESTS_ROOT_DIR}/Tools/Math/Random.cpp" "${TESTS_ROOT_DIR}/Tools/Math/Random.h"
    "${TESTS_ROOT_DIR}/Tools/Math/RotationMatrix.cpp" "${TESTS_ROOT_DIR}/Tools/Math/RotationMatrix.h"
    "${TESTS_ROOT_DIR}/Tools/Logging/LoggingTools.cpp" "${TESTS_ROOT_DIR}/Tools/Logging/LoggingTools.h"
    "${TESTS_ROOT_DIR}/Tools/MessageQueue/*.cpp" "${TESTS_ROOT_DIR}/Tools/MessageQueue/*.h"
    "${TESTS_ROOT_DIR}/Tools/Module/*.cpp" "${TESTS_ROOT_DIR}/Tools/Module/*.h"
    "${TESTS_ROOT_DIR}/Tools/Streams/*.cpp" "${TESTS_ROOT_DIR}/Tools/Streams/*.h")

set(TESTS_TREE ${TESTS_COMMON_SOURCES})

################################################################################
# GTest setup
# add GTest test files
file(GLOB_RECURSE GTEST_TEST_SOURCES
    "${TESTS_ROOT_DIR}/Utils/Tests/GTests/*.cpp" "${TESTS_ROOT_DIR}/Utils/Tests/GTests/*.h")

if(APPLE)
  list(REMOVE_ITEM GTEST_TEST_SOURCES "${TESTS_ROOT_DIR}/Utils/Tests/GTests/Test.cpp")
  list(APPEND GTEST_TEST_SOURCES "${TESTS_ROOT_DIR}/Utils/Tests/GTests/Test.mm")
endif()

add_executable(GTests MACOSX_BUNDLE ${TESTS_COMMON_SOURCES} ${GTEST_TEST_SOURCES})

set_property(TARGET GTests PROPERTY RUNTIME_OUTPUT_DIRECTORY "${TESTS_OUTPUT_DIR}")
set_property(TARGET GTests PROPERTY FOLDER Utils)
set_property(TARGET GTests PROPERTY MACOSX_BUNDLE_INFO_PLIST "${TESTS_ROOT_DIR}/Utils/Tests/GTests/Info.plist")
set_property(TARGET GTests PROPERTY XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "")
set_property(TARGET GTests PROPERTY XCODE_GENERATE_SCHEME ON)
set_property(TARGET GTests PROPERTY XCODE_PRODUCT_TYPE "com.apple.product-type.bundle.unit-test")


target_include_directories(GTests PRIVATE "${TESTS_ROOT_DIR}")

# multiplatform

if(APPLE)
  target_include_directories(GTests SYSTEM PRIVATE ${CORE_SERVICES_FRAMEWORK} ${CORE_SERVICES_FRAMEWORK}/Headers)
  target_link_libraries(GTests PRIVATE ${CORE_SERVICES_FRAMEWORK})
  target_link_libraries(GTests PRIVATE ${APP_KIT_FRAMEWORK})
endif()

target_link_libraries(GTests PRIVATE $<$<PLATFORM_ID:Linux>:flite::flite_cmu_us_slt> $<$<PLATFORM_ID:Linux>:flite::flite_usenglish>
    $<$<PLATFORM_ID:Linux>:flite::flite_cmulex> $<$<PLATFORM_ID:Linux>:flite::flite>)
target_link_libraries(GTests PRIVATE $<$<PLATFORM_ID:Linux>:ALSA::ALSA>)
target_link_libraries(GTests PRIVATE $<$<PLATFORM_ID:Linux>:-lpthread>)

target_link_libraries(GTests PRIVATE Eigen::Eigen)
target_link_libraries(GTests PRIVATE GameController::GameController)
target_link_libraries(GTests PRIVATE GTest::GTest)

target_compile_definitions(GTests PRIVATE TARGET_TOOL GTEST_DONT_DEFINE_FAIL GTEST_DONT_DEFINE_TEST GTEST_HAS_TR1_TUPLE=0)

target_link_libraries(GTests PRIVATE Flags::ForDevelop)

# visual studio
source_group(TREE "${TESTS_ROOT_DIR}" FILES ${TESTS_TREE})
source_group(TREE "${TESTS_ROOT_DIR}/Utils/Tests" FILES ${GTEST_TEST_SOURCES})


################################################################################
# Catch2 setup
# catch2 tutorial - FIXME - probably remove later
file(GLOB_RECURSE CATCH2_TUTORIAL_SOURCES
    "${TESTS_ROOT_DIR}/Utils/Tests/Catch2Tutorial/*.cpp" "${TESTS_ROOT_DIR}/Utils/Tests/Catch2Tutorial/*.h")

# catch2 tests
file(GLOB_RECURSE CATCH2_TEST_SOURCES
    "${TESTS_ROOT_DIR}/Utils/Tests/Catch2Tests*.cpp" "${TESTS_ROOT_DIR}/Utils/Tests/Catch2Tests/*.h")


add_executable(Catch2Tutorial ${CATCH2_TUTORIAL_SOURCES})
add_executable(Catch2Tests ${TESTS_TESTS_COMMON_SOURCES} ${CATCH2_TEST_SOURCES})

set_target_properties(Catch2Tutorial PROPERTIES LINKER_LANGUAGE CXX RUNTIME_OUTPUT_DIRECTORY "${TESTS_OUTPUT_DIR}")
set_target_properties(Catch2Tests PROPERTIES LINKER_LANGUAGE CXX RUNTIME_OUTPUT_DIRECTORY "${TESTS_OUTPUT_DIR}")


target_include_directories(Catch2Tutorial PRIVATE "${TESTS_ROOT_DIR}")
target_include_directories(Catch2Tests PRIVATE "${TESTS_ROOT_DIR}")

# Catch2Tests - Linux only

target_link_libraries(Catch2Tutorial PRIVATE Catch2::Catch2WithMain)


target_link_libraries(Catch2Tests PRIVATE $<$<PLATFORM_ID:Linux>:flite::flite_cmu_us_slt> $<$<PLATFORM_ID:Linux>:flite::flite_usenglish>
   $<$<PLATFORM_ID:Linux>:flite::flite_cmulex> $<$<PLATFORM_ID:Linux>:flite::flite>)
target_link_libraries(Catch2Tests PRIVATE $<$<PLATFORM_ID:Linux>:ALSA::ALSA>)
target_link_libraries(Catch2Tests PRIVATE $<$<PLATFORM_ID:Linux>:-lpthread>)

target_link_libraries(Catch2Tests PRIVATE Eigen::Eigen)
target_link_libraries(Catch2Tests PRIVATE GameController::GameController)
target_link_libraries(Catch2Tests PRIVATE Catch2::Catch2WithMain)
