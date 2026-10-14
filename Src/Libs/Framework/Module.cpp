/**
 * @file Module.cpp
 * The class attributes of the module handling schema.
 * @author Thomas Röfer
 * @author Rudi Villing (modifications)
 */

#include "Module.h"
#include "Streaming/InStreams.h"
#include "Platform/File.h"

#include "TextLogging/TextLogging.h"

DECL_TLOGGER(tlogger, "ModuleBase", TextLogging::INFO);

ModuleBase* ModuleBase::first = nullptr;

void loadModuleParameters(Streamable& parameters, const char* moduleName, const char* fileName, const char* prefix)
{
  // we will allow config files with name identical to module name in addition to the
  // B-Human convention of forcing initial uppercase to lowercase

  const std::string pfx = prefix ? prefix : "";

  // Build primary name: if no fileName, apply B-Human lowercase convention
  std::string name;
  if (!fileName)
  {
    ASSERT(moduleName);
    name = moduleName;
    name[0] = static_cast<char>(tolower(name[0]));
    if (name.size() > 1 && isupper(name[1]))
      for (int i = 1; i + 1 < static_cast<int>(name.size()) && isupper(name[i + 1]); ++i)
        name[i] = static_cast<char>(tolower(name[i]));
    name = pfx + name + ".cfg";
  }
  else
    name = pfx + fileName;

  // if no explicit filename given, try alternative name based on unmodified module name
  // (rather than B-Human convention of lowercasing the first word)
  std::string primaryName = name;
  std::string fallbackName;
  if (!fileName && !File::exists(name))
  {
    fallbackName = pfx + moduleName + ".cfg";
    if (fallbackName == primaryName) // would be very unusual, but is possible I guess
      fallbackName.clear();
    else
      name = fallbackName;
  }

  // try to open the primary name if it existed or else the fallback (if there is one)
  InMapFile stream(name);
  if (stream.exists())
  {
    TLOGD(tlogger, "Loading config '{}'", name);
    stream >> parameters;
    return;
  }
  else if (!fallbackName.empty())
    TLOG_ABORT(tlogger, "File '{}' (alt: '{}') does not exist in search path", primaryName, fallbackName);
  else
    TLOG_ABORT(tlogger, "File '{}' does not exist in search path", name);
}
