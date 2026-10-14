set(ROBOT_CORE_ROOT_DIR "${PROJECT_ROOT_PREFIX}/Src")

file(GLOB_RECURSE ROBOT_CORE_SOURCES CONFIGURE_DEPENDS
    "${ROBOT_CORE_ROOT_DIR}/Modules/*.cpp" "${ROBOT_CORE_ROOT_DIR}/Modules/*.h"
    "${ROBOT_CORE_ROOT_DIR}/Representations/*.cpp" "${ROBOT_CORE_ROOT_DIR}/Representations/*.h"
    "${ROBOT_CORE_ROOT_DIR}/Threads/*.cpp" "${ROBOT_CORE_ROOT_DIR}/Threads/*.h"
    "${ROBOT_CORE_ROOT_DIR}/Tools/*.cpp" "${ROBOT_CORE_ROOT_DIR}/Tools/*.h")

# Behaviour Options, Skills, and Tasks built by their own makefile so remove them from here
list(REMOVE_ITEM ROBOT_CORE_SOURCES ${OPTIONS_AND_SKILLS_SOURCES})
list(REMOVE_ITEM ROBOT_CORE_SOURCES ${RE_HYBRID_BEHAVIOUR_SOURCES})


# Prevent any file/folder beginning with "IGNORE_" from being included in the build
list(FILTER ROBOT_CORE_SOURCES EXCLUDE REGEX ".*/IGNORE_.*")

file(GLOB ROBOT_CORE_OPTIMIZED_SOURCES CONFIGURE_DEPENDS
    "${ROBOT_CORE_ROOT_DIR}/Tools/Math/*.cpp" "${ROBOT_CORE_ROOT_DIR}/Tools/Math/*.h"
    "${ROBOT_CORE_ROOT_DIR}/Tools/Motion/*.cpp" "${ROBOT_CORE_ROOT_DIR}/Tools/Motion/*.h")
list(APPEND ROBOT_CORE_OPTIMIZED_SOURCES
    "${ROBOT_CORE_ROOT_DIR}/Modules/BehaviorControl/FieldRatingProvider/FieldRatingProvider.h" "${ROBOT_CORE_ROOT_DIR}/Modules/BehaviorControl/FieldRatingProvider/FieldRatingProvider.cpp"
    "${ROBOT_CORE_ROOT_DIR}/Representations/Sensing/RobotModel.cpp" "${ROBOT_CORE_ROOT_DIR}/Representations/Sensing/RobotModel.h")
if(NOT WINDOWS)
  list(APPEND ROBOT_CORE_OPTIMIZED_SOURCES
      "${ROBOT_CORE_ROOT_DIR}/Modules/MotionControl/HeadMotionEngine/HeadMotionEngine.cpp" "${ROBOT_CORE_ROOT_DIR}/Modules/MotionControl/HeadMotionEngine/HeadMotionEngine.h"
      "${ROBOT_CORE_ROOT_DIR}/Modules/MotionControl/WalkingEngine/WalkingEngine.cpp" "${ROBOT_CORE_ROOT_DIR}/Modules/MotionControl/WalkingEngine/WalkingEngine.h"
      "${ROBOT_CORE_ROOT_DIR}/Modules/Perception/FieldPerceptors/LinePerceptor.cpp" "${ROBOT_CORE_ROOT_DIR}/Modules/Perception/FieldPerceptors/LinePerceptor.h"
      "${ROBOT_CORE_ROOT_DIR}/Modules/Perception/ImagePreprocessors/ECImageProvider.cpp" "${ROBOT_CORE_ROOT_DIR}/Modules/Perception/ImagePreprocessors/ECImageProvider.h"
      "${ROBOT_CORE_ROOT_DIR}/Modules/Sensing/FallDownStateDetector/FallDownStateProvider.cpp" "${ROBOT_CORE_ROOT_DIR}/Modules/Sensing/FallDownStateDetector/FallDownStateProvider.h"
      "${ROBOT_CORE_ROOT_DIR}/Modules/Sensing/FallDownStateDetector/BoosterFallDownStateProvider.cpp" "${ROBOT_CORE_ROOT_DIR}/Modules/Sensing/FallDownStateDetector/BoosterFallDownStateProvider.h"
      "${ROBOT_CORE_ROOT_DIR}/Modules/Sensing/InertialDataProvider/InertialDataProvider.cpp" "${ROBOT_CORE_ROOT_DIR}/Modules/Sensing/InertialDataProvider/InertialDataProvider.h"
      "${ROBOT_CORE_ROOT_DIR}/Tools/Modeling/UKFPose2D.cpp" "${ROBOT_CORE_ROOT_DIR}/Tools/Modeling/UKFPose2D.h")
endif()

# Prevent any file/folder beginning with "IGNORE_" from being included in the build
list(FILTER ROBOT_CORE_OPTIMIZED_SOURCES EXCLUDE REGEX ".*/IGNORE_.*")


set(ROBOT_CORE_PCHS
    [["Debugging/DebugDrawings.h"]]
    [["Framework/Module.h"]]
    [["MathBase/RingBufferWithSum.h"]]
    [["Math/Eigen.h"]]
    [["Math/Boundary.h"]]
    [["Math/Geometry.h"]]
    [["Math/Pose3f.h"]]
    [["Streaming/MessageQueue.h"]]
    [["Tools/Math/Transformation.h"]]
    <algorithm>
    <list>
    <sstream>)

if(MACOS AND BUILD_DESKTOP)
  add_library(RobotCore-Optimized${TARGET_SUFFIX} OBJECT ${ROBOT_CORE_OPTIMIZED_SOURCES})
  set_property(TARGET RobotCore-Optimized${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
  set_property(TARGET RobotCore-Optimized${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_include_directories(RobotCore-Optimized${TARGET_SUFFIX} PRIVATE "${ROBOT_CORE_ROOT_DIR}")
  target_link_libraries(RobotCore-Optimized${TARGET_SUFFIX} PRIVATE Platform${TARGET_SUFFIX})
  target_link_libraries(RobotCore-Optimized${TARGET_SUFFIX} PRIVATE MathBase${TARGET_SUFFIX})
  target_link_libraries(RobotCore-Optimized${TARGET_SUFFIX} PRIVATE Streaming${TARGET_SUFFIX})
  target_link_libraries(RobotCore-Optimized${TARGET_SUFFIX} PRIVATE Math${TARGET_SUFFIX})
  target_link_libraries(RobotCore-Optimized${TARGET_SUFFIX} PRIVATE ImageProcessing${TARGET_SUFFIX})
  target_link_libraries(RobotCore-Optimized${TARGET_SUFFIX} PRIVATE Network${TARGET_SUFFIX})
  target_link_libraries(RobotCore-Optimized${TARGET_SUFFIX} PRIVATE RobotParts${TARGET_SUFFIX})
  target_link_libraries(RobotCore-Optimized${TARGET_SUFFIX} PRIVATE Debugging${TARGET_SUFFIX})
  target_link_libraries(RobotCore-Optimized${TARGET_SUFFIX} PRIVATE Framework${TARGET_SUFFIX})
  target_link_libraries(RobotCore-Optimized${TARGET_SUFFIX} PRIVATE CompiledNN::ONNX)
  target_link_libraries(RobotCore-Optimized${TARGET_SUFFIX} PUBLIC asmjit${TARGET_SUFFIX})

  target_link_libraries(RobotCore-Optimized${TARGET_SUFFIX} PRIVATE Flags::Default)
  target_precompile_headers(RobotCore-Optimized${TARGET_SUFFIX} PRIVATE ${ROBOT_CORE_PCHS})

  list(REMOVE_ITEM ROBOT_CORE_SOURCES ${ROBOT_CORE_OPTIMIZED_SOURCES})
endif()

add_library(RobotCore${TARGET_SUFFIX} OBJECT ${ROBOT_CORE_SOURCES})
if(BUILD_DESKTOP)
  if(MACOS)
    target_sources(RobotCore${TARGET_SUFFIX} INTERFACE $<TARGET_OBJECTS:RobotCore-Optimized${TARGET_SUFFIX}>)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC RobotCore-Optimized${TARGET_SUFFIX})
  else()
    set_property(SOURCE ${ROBOT_CORE_OPTIMIZED_SOURCES} APPEND PROPERTY COMPILE_OPTIONS
        $<${_is_msvc}:$<$<CONFIG:Develop>:/Ox>>
        $<${_is_clang}:$<$<CONFIG:Develop>:-O3>>)
    set_property(SOURCE ${ROBOT_CORE_OPTIMIZED_SOURCES} PROPERTY SKIP_PRECOMPILE_HEADERS ON)
  endif()
endif()

target_sources(RobotCore${TARGET_SUFFIX} INTERFACE 
  $<TARGET_OBJECTS:OptionsAndSkills${TARGET_SUFFIX}> 
  $<TARGET_OBJECTS:REHybridBehaviourTasks${TARGET_SUFFIX}> 
  $<TARGET_OBJECTS:Platform${TARGET_SUFFIX}> 
  $<TARGET_OBJECTS:MathBase${TARGET_SUFFIX}> 
  $<TARGET_OBJECTS:Streaming${TARGET_SUFFIX}> 
  $<TARGET_OBJECTS:Math${TARGET_SUFFIX}> 
  $<TARGET_OBJECTS:ImageProcessing${TARGET_SUFFIX}> 
  $<TARGET_OBJECTS:Network${TARGET_SUFFIX}> 
  $<TARGET_OBJECTS:RobotParts${TARGET_SUFFIX}> 
  $<TARGET_OBJECTS:Debugging${TARGET_SUFFIX}> 
  $<TARGET_OBJECTS:Framework${TARGET_SUFFIX}>)

set_property(TARGET RobotCore${TARGET_SUFFIX} PROPERTY FOLDER "Libs/${TARGET_SUFFIX}")
target_include_directories(RobotCore${TARGET_SUFFIX} PUBLIC "${ROBOT_CORE_ROOT_DIR}")
target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC OptionsAndSkills${TARGET_SUFFIX})
target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC REHybridBehaviourTasks${TARGET_SUFFIX})
target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC Platform${TARGET_SUFFIX})
target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC MathBase${TARGET_SUFFIX})
target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC Streaming${TARGET_SUFFIX})
target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC Math${TARGET_SUFFIX})
target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC ImageProcessing${TARGET_SUFFIX})
target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC Network${TARGET_SUFFIX})
target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC RobotParts${TARGET_SUFFIX})
target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC Debugging${TARGET_SUFFIX})
target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC Framework${TARGET_SUFFIX})
if(BUILD_DESKTOP)
  set_property(TARGET RobotCore${TARGET_SUFFIX} PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE FFTW::FFTW FFTW::FFTWF)
  target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE libjpeg::libjpeg)
  if(OPENSSL_FOUND)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE OpenSSL::SSL)
    target_compile_definitions(RobotCore${TARGET_SUFFIX} PRIVATE HAVE_OPENSSL)
  endif()
  if(X86)
    if(NOT MACOS)
      target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE asmjit${TARGET_SUFFIX})
    endif()
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE CompiledNN${TARGET_SUFFIX})
  endif()
  target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE CompiledNN::ONNX)
  target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Flags::DebugInDevelop)
else() # not desktop
  if(X86)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE CompiledNN${TARGET_SUFFIX})
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Nao::FFTW::FFTW Nao::FFTW::FFTWF)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Nao::libjpeg::libjpeg)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Nao::ALSA::ALSA)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Nao::CompiledNN::ONNX)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Nao::OpenSSL::SSL)
    target_compile_definitions(RobotCore${TARGET_SUFFIX} PRIVATE HAVE_OPENSSL)
  else()
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Booster::FFTW::FFTW Booster::FFTW::FFTWF)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Booster::libjpeg::libjpeg)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Booster::CompiledNN::ONNX)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Booster::realsense2::realsense2)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Booster::Booster::SDK)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Booster::FastCDR::FastCDR)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Booster::FastRTPS::FastRTPS)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Booster::FooNathan::Memory)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Booster::WakeupAPI::WakeupAPI)

    # address sanitizer - comment out when done - doesn't work currently
    # target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC Flags::SanitizerFlags)  
    # set_property(SOURCE ${ROBOT_CORE_OPTIMIZED_SOURCES} APPEND PROPERTY COMPILE_OPTIONS -fno-sanitize=address) 
    # set_property(SOURCE ${ROBOT_CORE_OPTIMIZED_SOURCES} APPEND PROPERTY SKIP_PRECOMPILE_HEADERS ON) 
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Booster::TensorRT::NvInfer)
    target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Booster::CUDA::Runtime)
    target_compile_definitions(RobotCore${TARGET_SUFFIX} PRIVATE BUILD_BOOSTER)
  endif()
  target_include_directories(RobotCore${TARGET_SUFFIX} SYSTEM PRIVATE "${PROJECT_ROOT_PREFIX}/Util/MD5")
  target_compile_definitions(RobotCore${TARGET_SUFFIX} PUBLIC TARGET_ROBOT)
  target_compile_options(RobotCore${TARGET_SUFFIX} PRIVATE $<$<CONFIG:Develop>:-UNDEBUG> $<$<CONFIG:Release>:-Wno-unused>)
  # target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Flags::Default)
  target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE Flags::OptimizedDebugInDevelop)
endif()
target_link_libraries(RobotCore${TARGET_SUFFIX} PRIVATE voronoi::voronoi)
target_link_libraries(RobotCore${TARGET_SUFFIX} PUBLIC GameController::GameController)
target_compile_definitions(RobotCore${TARGET_SUFFIX} PRIVATE CONFIGURATION=$<CONFIG>) # for RobotHealthProvider

target_precompile_headers(RobotCore${TARGET_SUFFIX} PUBLIC ${ROBOT_CORE_PCHS})
if(MSVC)
  target_compile_options(RobotCore${TARGET_SUFFIX} PUBLIC /Zm200)
endif()

source_group(TREE "${ROBOT_CORE_ROOT_DIR}" FILES ${ROBOT_CORE_SOURCES})
