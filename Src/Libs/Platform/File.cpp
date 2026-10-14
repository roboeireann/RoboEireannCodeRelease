/**
 * @file Platform/File.cpp
 */

#include "File.h"
#include "Platform/BHAssert.h"
#include "TextLogging/TextLogging.h"

#include <unordered_set>

#include <cstdarg>
#include <cstdio>

#include <sys/stat.h>

#ifdef WINDOWS
#include <Windows.h>
#define ftell _ftelli64
#define fseek _fseeki64
#else
#include <cstring>
#ifndef TARGET_ROBOT
#include <unistd.h>
#endif
#endif

thread_local std::vector<std::string> File::searchPath;

DECL_TLOGGER(tlogger,"File",TextLogging::INFO);


File::File(const std::string& name, const char* mode, bool tryAlternatives, bool isOptional)
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

  // text logging, primarily for debugging config file issues and ensuring
  // the correct files being loaded. (This is needed because of the fallback
  // behaviour and the ability to pick up files from many different locations)
  if (stream == nullptr)
  {
    if (!isRepeatedFile(name))
    {
      if (isOptional)
        TLOGD(tlogger, "Optional file '{}' not found", name);
      else
        TLOGW(tlogger, "Required file '{}' not found", name);
    }

    if (tlogger.isEnabled(TextLogging::VERBOSE))
    {
      for ([[maybe_unused]] auto& nm : names)
        TLOGV(tlogger, "  not at '{}'", nm);
    }
  }
  else if (tlogger.isEnabled(TextLogging::INFO))
  {
    if (!isRepeatedFile(fullName))
#ifdef TARGET_ROBOT
      TLOGI(tlogger, "opened {}", fullName);
#else
      TLOGD(tlogger, "opened {}", fullName);
#endif
  }
}

File::~File()
{
  if(stream != 0)
    fclose(static_cast<FILE*>(stream));
}


bool File::exists(const std::string& name, bool tryAlternatives)
{
  struct stat buf;
  std::list<std::string> fullPaths = getFullNames(name);

  if (tryAlternatives)
  {
    for (const auto &path : fullPaths) // name relative to all directories in the search pth
      if (stat(path.c_str(), &buf) == 0)
        return true;
  }
  else if (stat(fullPaths.back().c_str(), &buf) == 0) // name relative to base config dir only
    return true;
  
  return false;
}

std::list<std::string> File::getFullNames(const std::string& name)
{
  std::list<std::string> names;
  if((name[0] != '.' || (name.size() >= 2 && name[1] == '.')) && !isAbsolute(name)) // given path is relative to getBHDir()
  {
    for(const std::string& dir : searchPath)
      names.push_back(dir + name);
    names.push_back(std::string(getBHDir()) + "/Config/" + name);
  }
  else
    names.push_back(name);
  return names;
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

void File::skip(size_t size)
{
  VERIFY(fseek(static_cast<FILE*>(stream), size, SEEK_CUR) == 0);
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

const char* File::getRuntimeDataRoot()
{
#ifdef WINDOWS
  static char dir[MAX_PATH] = {0};
  if(!dir[0])
  {
    // determine module file from command line
    const char* commandLine = GetCommandLine();
    size_t len;
    if(*commandLine == '"')
    {
      commandLine++;
      const char* end = strchr(commandLine, '"');
      if(end)
        len = end - commandLine;
      else
        len = strlen(commandLine);
    }
    else
      len = strlen(commandLine);

    if(len >= sizeof(dir) - 8)
      len = sizeof(dir) - 8;
    memcpy(dir, commandLine, len);
    dir[len] = '\0';

    // if there is no given directory, use the current working dir
    if(!strchr(dir, '\\'))
    {
      len = int(GetCurrentDirectory(sizeof(dir) - 9, dir));
      if(len && dir[len - 1] != '\\')
      {
        dir[len++] = '\\';
        dir[len] = '\0';
      }
    }

    //drive letter in lower case:
    if(len && dir[1] == ':')
      *dir |= tolower(*dir);

    // try to find the config directory
    char* end = dir + len - 1;
    while(true)
    {
      if(end < dir || *end == '/' || *end == '\\' || *end == ':')
      {
        if(end >= dir && *end == ':')
          *(end++) = '\\';
        strcpy(end + 1, "Config");
        DWORD attr = GetFileAttributes(dir);
        if(attr != INVALID_FILE_ATTRIBUTES && attr & FILE_ATTRIBUTE_DIRECTORY)
        {
          end[end > dir ? 0 : 1] = '\0';
          for(; end >= dir; end--)
            if(*end == '\\')
              *end = '/';
          return dir;
        }
      }
      if(end < dir)
        break;
      end--;
    }
    FAIL("Could not find the Config directory.");
  }
#else
  static char dir[FILENAME_MAX] = {0};
  if(!dir[0])
  {
#ifndef TARGET_ROBOT
    VERIFY(getcwd(dir, sizeof(dir) - 8) != 0);
    char* end = dir + strlen(dir) - 1;
    if(end >= dir && *end != '/')
      *++end = '/';
    struct stat buff;
    while(true)
    {
      if(end < dir || *end == '/')
      {
        strcpy(end + 1, "Config");
        if(stat(dir, &buff) == 0)
          if(S_ISDIR(buff.st_mode))
          {
            end[end > dir ? 0 : 1] = '\0';
            return dir;
          }
      }
      if(end < dir)
        break;
      end--;
    }
#endif
    strcpy(dir, ".");
  }
#endif
  return dir;
}

const std::string& File::getConfigDir()
{
  static std::string configDir = std::string(getRuntimeDataRoot()) + "/Config";
  return configDir;
}

bool File::isAbsolute(const std::string& path)
{
  return (!path.empty() && (path[0] == '/' || path[0] == '\\'))
         || (path.size() > 1 && path[1] == ':');
}

bool File::hasExtension(const std::string& path)
{
  return static_cast<int>(path.rfind('.')) > static_cast<int>(path.find_last_of("\\/"));
}

void File::setSearchPath(const std::vector<std::string>& paths)
{
  searchPath = paths;
}

void File::clearSearchPath()
{
  searchPath.clear();
}

bool File::isRepeatedFile(const std::string& name)
{
  static std::unordered_set<std::string> files;

  return !files.insert(name).second; // second is true if item inserted which means it is not a repeat
}
