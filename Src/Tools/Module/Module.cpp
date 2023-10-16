/**
 * @file Module.cpp
 * The class attributes of the module handling schema.
 * @author Thomas Röfer
 */

#include "Module.h"
#include "Tools/Streams/InStreams.h"

#include "Tools/TextLogging.h"

DECL_TLOGGER(tlogger, "ModuleBase", TextLogging::INFO);

ModuleBase* ModuleBase::first = nullptr;

void loadModuleParameters(Streamable& parameters, const char* moduleName, const char* fileName, const char* prefix)
{
  std::string name;
  if(!fileName)
  {
    name = moduleName;
    // lowercase the first letter of the config file
    name[0] = static_cast<char>(tolower(name[0]));
    // if the second letter is uppercase, lowercase all the letters in the
    if(name.size() > 1 && isupper(name[1]))
      for(int i = 1; i + 1 < static_cast<int>(name.size()) && isupper(name[i + 1]); ++i)
        name[i] = static_cast<char>(tolower(name[i]));
    name += ".cfg";
  }
  else
    name = fileName;
  if(prefix)
    name = prefix + name;

  TLOGD(tlogger, "loadModuleParameters for {} from '{}'", moduleName, name);

  InMapFile stream(name);
  ASSERT(stream.exists());
  stream >> parameters;
}
