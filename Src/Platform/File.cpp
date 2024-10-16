/**
 * @file Platform/File.cpp
 */

#include "File.h"
#include "BHAssert.h"

#ifndef TARGET_TOOL
#include "Tools/Global.h"
#include "Tools/Settings.h"
#endif

#include "Tools/TextLogging.h"

#include <unordered_set>

#include <cstdarg>
#include <cstdio>

#ifdef WINDOWS
#define ftell _ftelli64
#define fseek _fseeki64
#endif

DECL_TLOGGER(tlogger,"File",TextLogging::WARNING);

File::File(const std::string& name, const char* mode, bool tryAlternatives)
{
  fullName = name;
  std::list<std::string> names = getFullNames(name);
  if(tryAlternatives)
  {
    for(auto& path : names)
    {
      stream = fopen(path.c_str(), mode);
      if(stream)
      {
        fullName = path;
        break;
      }
    }
  }
  else
  {
    stream = fopen(names.back().c_str(), mode);
    if(stream)
      fullName = names.back();
  }

  if (stream == nullptr)
  {
    if (tlogger.isEnabled(TextLogging::WARNING))
    {
      if (!repeatedFile(name))
        TLOGW(tlogger, "{} not found", name);
    }

    if (tlogger.isEnabled(TextLogging::DEBUG))
    {
      for (auto& nm : names)
      {
        TLOG_USE(nm);
        TLOGD(tlogger, "  not at {}", nm);
      }
    }
  }
  else if (tlogger.isEnabled(TextLogging::INFO))
  {
    if (!repeatedFile(fullName))
      TLOGI(tlogger, "{} found at {}", name, fullName);
  }
}

File::~File()
{
  if(stream != 0)
    fclose(static_cast<FILE*>(stream));
}

void File::read(void* p, size_t size)
{
  VERIFY(!eof());
  VERIFY(fread(p, 1, size, static_cast<FILE*>(stream)) > 0);
}

char* File::readLine(char* p, size_t size)
{
  VERIFY(!eof());
  return fgets(p, static_cast<int>(size), static_cast<FILE*>(stream));
}

void File::write(const void* p, size_t size)
{
  //if opening failed, stream will be 0 and fwrite would crash
  ASSERT(stream != 0);
#ifdef NDEBUG
  static_cast<void>(fwrite(p, 1, size, static_cast<FILE*>(stream)));
#else
  const size_t written = fwrite(p, 1, size, static_cast<FILE*>(stream));
  if(written != size)
  {
    perror("fwrite did not write as many bytes as requested");
    FAIL("File::write failed!");
  }
#endif
}

void File::printf(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  vfprintf(static_cast<FILE*>(stream), format, args);
  va_end(args);
}

bool File::eof()
{
  //never use feof(stream), because it informs you only after reading
  //too far and is never reset, e.g. if the stream grows afterwards,
  //our implementation can handle both correctly:
  if(!stream)
    return false;
  else
  {
    int c = fgetc(static_cast<FILE*>(stream));
    if(c == EOF)
      return true;
    else
    {
      VERIFY(ungetc(c, static_cast<FILE*>(stream)) != EOF);
      return false;
    }
  }
}

size_t File::getSize()
{
  if(!stream)
    return 0;
  else
  {
    const size_t currentPos = getPosition();
    VERIFY(fseek(static_cast<FILE*>(stream), 0, SEEK_END) == 0);
    const size_t size = getPosition();
    VERIFY(fseek(static_cast<FILE*>(stream), currentPos, SEEK_SET) == 0);
    return static_cast<size_t>(size);
  }
}

size_t File::getPosition()
{
  if(!stream)
    return 0;
  else
  {
    const auto currentPos = ftell(static_cast<FILE*>(stream));
    static_assert(sizeof(currentPos) == 8, "ftell/fseek must use 64 bit offsets");
    ASSERT(currentPos >= 0);
    return static_cast<size_t>(currentPos);
  }
}

bool File::isAbsolute(const char* path)
{
  return (path[0] && path[1] == ':') || path[0] == '/' || path[0] == '\\';
}

std::list<std::string> File::getConfigDirs()
{
  std::list<std::string> dirs;
  const std::string configDir = std::string(getBHDir()) + "/Config/";
#ifndef TARGET_TOOL
  // The search paths in search order are:
  //
  // - Config/Robots/someHead/someBody/
  // - Config/Robots/someHead/Head/
  // - Config/Robots/someBody/Body/
  //
  // - Config/Scenarios/someScenario/Locations/someLocation/Sim *** (someScenario can be Default)
  // - Config/Scenarios/someScenario/Locations/someLocation/        (someScenario can be Default)
  //
  // - Config/Scenarios/Default/Locations/someLocation/Sim *** (if scenario was not Default, we search here also - more or less equivalent to Config/Locations...)
  // - Config/Scenarios/Default/Locations/someLocation/        (if scenario was not Default...)
  //
  // - Config/Locations/someLocation/Sim ***
  // - Config/Locations/someLocation/
  //
  // - Config/Scenarios/someScenario/
  //
  // - Config/Robots/Default/
  //
  // - Config/Locations/Default/Sim ***
  // - Config/Locations/Default
  //
  // - Config/Scenarios/Default
  // - Config
  //
  // *** indicates paths that are only searched in SimRobot and not on the robot itself
  
  if (Global::settingsExist())
  {
    dirs.push_back(configDir + "Robots/" + Global::getSettings().headName + "/" + Global::getSettings().bodyName + "/");
    dirs.push_back(configDir + "Robots/" + Global::getSettings().headName + "/Head/");
    dirs.push_back(configDir + "Robots/" + Global::getSettings().bodyName + "/Body/");

    // allow the specific combination of scenario and location to get some customisation
    // unless both are Default
    if (Global::getSettings().location != "Default")
    {
#ifdef TARGET_SIM
      dirs.push_back(configDir + "Scenarios/" + Global::getSettings().scenario + 
                     "/Locations/" + Global::getSettings().location + "/Sim/");
#endif
      dirs.push_back(configDir + "Scenarios/" + Global::getSettings().scenario + 
                     "/Locations/" + Global::getSettings().location + "/");

      if (Global::getSettings().scenario != "Default") // if our actual scenario was not default, make sure we fall back to checking default also
      {
#ifdef TARGET_SIM
        dirs.push_back(configDir + "Scenarios/Default/Locations/" + Global::getSettings().location + "/Sim/");
#endif
        dirs.push_back(configDir + "Scenarios/Default/Locations/" + Global::getSettings().location + "/");
      }
    }

    if (Global::getSettings().location != "Default")
    {
#ifdef TARGET_SIM
      dirs.push_back(configDir + "Locations/" + Global::getSettings().location + "/Sim/");
#endif
      dirs.push_back(configDir + "Locations/" + Global::getSettings().location + "/");
    }

    if (Global::getSettings().scenario != "Default")
      dirs.push_back(configDir + "Scenarios/" + Global::getSettings().scenario + "/");

    dirs.push_back(configDir + "Robots/Default/");
#ifdef TARGET_SIM
    dirs.push_back(configDir + "Locations/Default/Sim/");
#endif
    dirs.push_back(configDir + "Locations/Default/");
    dirs.push_back(configDir + "Scenarios/Default/");
  }
#endif
  dirs.push_back(configDir);
  return dirs;
}

bool File::repeatedFile(const std::string& name)
{
  static std::unordered_set<std::string> files;

  return !files.insert(name).second; // second is true if item inserted which means it is not a repeat
}

std::string File::getRobotHeadBodyPath()
{
  const std::string configDir = std::string(getBHDir()) + "/Config/";
#ifndef TARGET_TOOL
  return configDir + "Robots/" + Global::getSettings().headName + "/" + Global::getSettings().bodyName + "/";
#else
  return configDir;
#endif
}

std::string File::getRobotHeadPath()
{
  const std::string configDir = std::string(getBHDir()) + "/Config/";
#ifndef TARGET_TOOL
  return configDir + "Robots/" + Global::getSettings().headName + "/Head/";
#else
  return configDir;
#endif
}

std::string File::getRobotBodyPath()
{
  const std::string configDir = std::string(getBHDir()) + "/Config/";
#ifndef TARGET_TOOL
  return configDir + "Robots/" + Global::getSettings().bodyName + "/Body/";
#else
  return configDir;
#endif
}

void File::createDirectories(const std::string& path)
{
  // FIXME: migrate to filesystem::create_directories once we're using C++17
  system(fmt::format("mkdir -p {}", path).c_str());
}