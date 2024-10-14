/**
 * @file LoggingTools.cpp
 *
 * This file implements the functions from the LoggingTools namespace.
 *
 * The original B-Human log file names looked like the following example:
 *
 *   bev_bev_Default_RC2023__Dutch-Nao-Team_1stHalf_4_(5).log
 *
 * where the fields were head, body, scenario, location, identifier, playerNumber, suffix.
 * The suffix in particular was separated by a single underscore and took the form of a
 * number in parentheses.
 *
 * We'll handle this form for older log files, but new log files will be like
 * the following example instead:
 *
 *   bev_bev_Default_RC2023__Dutch-Nao-Team_half1_p4__005.log
 *
 * Note the 'p' before the player number, the double underscore before the suffix
 * and the suffix doesn't use parentheses.
 *
 * @author Arne Hasselbring
 * @author Rudi Villing
 */

#include "LoggingTools.h"
#include "Platform/BHAssert.h"

#include "Tools/TextLogging.h"

#include <regex>

DECL_TLOGGER(tlogger, "LoggingTools", TextLogging::INFO);

std::string LoggingTools::createName(const std::string &prefix, const std::string &headName,
                                     const std::string &bodyName, const std::string &scenario,
                                     const std::string &location, const std::string &identifier, int playerNumber,
                                     const std::string &suffix)
{
  return prefix + (prefix.empty() || prefix[prefix.size() - 1] == '/' ? "" : "_") + headName + "_" + bodyName + "_" +
         scenario + "_" + location + "__" + identifier + "_p" + std::to_string(playerNumber) +
         (!suffix.empty() ? "__" + suffix : "");
}

void LoggingTools::parseName(const std::string &logfileName, std::string *prefix, std::string *headName,
                             std::string *bodyName, std::string *scenario, std::string *location,
                             std::string *identifier, int *playerNumber, std::string *suffix)
{
  // regex for thread, head name, body name, scenario and location
  static std::regex thbsl("([A-Za-z_]+)_([A-Za-z]+)_([A-Za-z]+)_([A-Za-z0-9-]+)_([A-Za-z0-9-]+)__");
  // regex for head name, body name, scenario and location
  static std::regex hbsl("([A-Za-z]+)_([A-Za-z]+)_([A-Za-z0-9-]+)_([A-Za-z0-9-]+)__");
  // regex for only thread, head name and body name
  static std::regex thb("([A-Za-z_]+)_([A-Za-z-]+)_([A-Za-z-]+)__");
  // regex for identifier and player number and suffix (for old BH style log file names and ours)
  static std::regex ipsOld("__([A-Za-z0-9_-]*)_([0-9]+)(_\\([0-9]+\\))?\\.");
  static std::regex ips("__([A-Za-z0-9_-]*)_p([0-9]+)__([0-9]+(_[0-9]+)?)\\.");
  std::smatch match;

  TLOGI(tlogger, "logFileName: \"{}\"", logfileName);

  if (std::regex_search(logfileName, match, thbsl))
  {
    ASSERT(match.size() == 6);
    TLOGI(tlogger, "prefix \"{}\", head \"{}\", body \"{}\", scenario \"{}\", location \"{}\"", 
          std::string(match[1]), std::string(match[2]), std::string(match[3]), std::string(match[4]), std::string(match[5]));

    if (prefix)
      *prefix = match[1];
    if (headName)
      *headName = match[2];
    if (bodyName)
      *bodyName = match[3];
    if (scenario)
      *scenario = match[4];
    if (location)
      *location = match[5];
  }
  else if (std::regex_search(logfileName, match, hbsl))
  {
    ASSERT(match.size() == 5);

    TLOGI(tlogger, "head \"{}\", body \"{}\", scenario \"{}\", location \"{}\"", 
          std::string(match[1]), std::string(match[2]), std::string(match[3]), std::string(match[4]));

    if (headName)
      *headName = match[1];
    if (bodyName)
      *bodyName = match[2];
    if (scenario)
      *scenario = match[3];
    if (location)
      *location = match[4];
  }
  else if (std::regex_search(logfileName, match, thb))
  {

    TLOGI(tlogger, "prefix \"{}\", head \"{}\", body \"{}\"", 
          std::string(match[1]), std::string(match[2]), std::string(match[3]));

    ASSERT(match.size() == 4);
    if (prefix)
      *prefix = match[1];
    if (headName)
      *headName = match[2];
    if (bodyName)
      *bodyName = match[3];
  }

  // due to the way the regex's match, the new pattern must be checked before
  // the old BH one
  if (std::regex_search(logfileName, match, ips))
  {
    ASSERT(match.size() == 5);

    TLOGI(tlogger, "identifier \"{}\", playerNumber \"{}\", suffix \"{}\", suffixExtra \"{}\"", 
          std::string(match[1]), std::string(match[2]), std::string(match[3]), std::string(match[4]));

    if (identifier)
      *identifier = match[1];
    if (playerNumber)
      *playerNumber = std::stoi(match[2]);
    if (suffix)
      *suffix = match[3]; //match[4].matched ? match[4].str().substr(1) : "";
  }
  else if (std::regex_search(logfileName, match, ipsOld))
  {
    ASSERT(match.size() == 4);

    TLOGI(tlogger, "[BH style] identifier \"{}\", playerNumber \"{}\", suffix \"{}\"", 
          std::string(match[1]), std::string(match[2]), std::string(match[3]));

    if (identifier)
      *identifier = match[1];
    if (playerNumber)
      *playerNumber = std::stoi(match[2]);
    if (suffix)
      *suffix = match[3].matched ? match[3].str().substr(1) : "";
  }
}
