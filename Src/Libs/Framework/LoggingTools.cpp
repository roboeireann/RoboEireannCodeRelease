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
 *   bev_bev__Default__RC2023__Dutch-Nao-Team_half1__p4__005_00.log
 * 
 * Note the 'p' before the player number, the double underscores to separate main units
 * and the new suffix format. The double underscores mean that scenario and location
 * names are permitted to include non-doubled underscores in addition to '-' and
 * alphanumerics.
 *
 * TBC: If the SimRobot facility to split a file is used, it will instead look like te following:
 *
 *   bev_bev_Default__RC2023__Dutch-Nao-Team_half1_p4__005_00__part_3.log
 *
 * @author Arne Hasselbring
 * @author Rudi Villing (modifications)
 */

#include "LoggingTools.h"
#include "Framework/Settings.h"
#include "Platform/BHAssert.h"
#include "Streaming/InOut.h"
#include "TextLogging/TextLogging.h"

#include <regex>
#include <type_traits>


DECL_TLOGGER(tlogger, "LoggingTools", TextLogging::INFO);

void LoggingTools::writeSettings(Out& stream, const Settings& settings)
{
  stream << unsigned(1);
  stream << settings.headName;
  stream << settings.bodyName;
  stream << settings.playerNumber;
  stream << settings.location;
  stream << settings.scenario;
}

void LoggingTools::readSettings(In& stream, Settings& settings)
{
  unsigned version;
  stream >> version;
  if(version == 1)
  {
    stream >> settings.headName;
    stream >> settings.bodyName;
    stream >> settings.playerNumber;
    stream >> settings.location;
    stream >> settings.scenario;
  }
  else
    FAIL("Unknown settings version " << version << ".");
}

void LoggingTools::skipSettings(In& stream)
{
  unsigned version;
  stream >> version;
  if(version == 1)
  {
    decltype(Settings::headName) string;
    static_assert(std::is_same<decltype(Settings::headName), decltype(Settings::bodyName)>::value);
    static_assert(std::is_same<decltype(Settings::headName), decltype(Settings::location)>::value);
    static_assert(std::is_same<decltype(Settings::headName), decltype(Settings::scenario)>::value);
    decltype(Settings::playerNumber) number;
    stream >> string;
    stream >> string;
    stream >> number;
    stream >> string;
    stream >> string;
  }
  else
    FAIL("Unknown settings version " << version << ".");
}

std::string LoggingTools::createName(const std::string& headName, const std::string& bodyName, const std::string& scenario,
                                     const std::string& location, const std::string& identifier, int playerNumber)
{
  // old B-Human way
  // return headName + "_" + bodyName + "_" + scenario + "_" + location + "__" + identifier + "_" +
  //        std::to_string(playerNumber) + (!suffix.empty() ? "_" + suffix : "");

  // new name format
  ASSERT(!headName.empty() && !bodyName.empty() && !scenario.empty() && !location.empty() && !identifier.empty());
  ASSERT(playerNumber >= 0); // probably > 0 ???

  return headName + "_" + bodyName + "__" + scenario + "__" + location + "__" + identifier + "_" +
         std::to_string(playerNumber);
}

void LoggingTools::parseName(const std::string& logfileName, std::string* headName, std::string* bodyName,
                             std::string* scenario, std::string* location, std::string* identifier, int* playerNumber, std::string* suffix)
{
  // HACK: (RC2022) regex for head name, body name, scenario and location
  // static std::regex hbslrc22("([A-Za-z]*)_([A-Za-z]*)_([A-Za-z0-9]*)_([A-Za-z0-9]*_Field[A-Z])__"); 

  // regex for head name, body name, scenario and location for old B-Human style and ours.
  // Note that our new regex allows '-' and non-doubled '_' characters in the scenario or location name
  static std::regex hbslOld("([A-Za-z]*)_([A-Za-z]*)_([A-Za-z0-9]*)_([A-Za-z0-9]*)__"); 
  // clang-format off
  static const std::regex hbsl(
      "([A-Za-z]+)"       // group1 = robot_head
      "_"
      "([A-Za-z]+)"       // group2 = robot_body
      "__"
      "([A-Za-z0-9_-]+)"  // group3 = scenario (can contain - or non-double _)
      "__"
      "([A-Za-z0-9_-]+)"  // group4 = location (can contain - or non-double _)
      "__"
  );
  // clang-format on

  // regex for identifier, player number, numeric suffix for old B-Human style and ours
  static std::regex ipsOld(".*__([A-Za-z0-9_-]*)_([0-9]+)(_\\([0-9]+\\))?\\."); 
  // clang-format off
  static const std::regex ips(
      ".*"
      "__([A-Za-z0-9_-]+)"          // group1 = identifier, e.g. opposition team and game phase or NoGC if GC not active
      "__p([0-9]+)"                 // group2 = player number
      "__("                         // open group3 = complete suffix
          "([0-9]+(?:_[0-9]+)?)"    //   group4 = unique id (mmm_nn), where mmm matches tlog number and _nn is optional
          "(?:__([A-Za-z0-9_-]+))?" //   group5 (optional) = part id for divided logs
      ")"                           // close group3
      "\\."                         // literal dot before extension
  );
  // clang-format on

  std::smatch match;

  TLOGI(tlogger, "parsing name for logfile \"{}\"", logfileName);

  // if(std::regex_search(logfileName, match, hbslrc22))
  // {
  //   ASSERT(match.size() == 5);
  //   if(headName)
  //     *headName = match[1];
  //   if(bodyName)
  //     *bodyName = match[2];
  //   if(scenario)
  //     *scenario = match[3];
  //   if(location)
  //     *location = match[4];
  // }
  // else
  if (std::regex_search(logfileName, match, hbsl))
  {
    ASSERT(match.size() == 5);
    TLOGD(tlogger, "head \"{}\", body \"{}\", scenario \"{}\", location \"{}\"", 
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
  else if (std::regex_search(logfileName, match, hbslOld))
  {
    ASSERT(match.size() == 5);
    TLOGD(tlogger, "[BH style] head \"{}\", body \"{}\", scenario \"{}\", location \"{}\"", std::string(match[1]),
          std::string(match[2]), std::string(match[3]), std::string(match[4]));

    if (headName)
      *headName = match[1];
    if (bodyName)
      *bodyName = match[2];
    if (scenario)
      *scenario = match[3];
    if (location)
      *location = match[4];
  }
  else
  {
    TLOGE(tlogger, "could not parse head/body/scenario/location from log file name");
    return;
  }

  if (std::regex_search(logfileName, match, ips))
  {
    ASSERT(match.size() == 5);
    TLOGD(tlogger, "identifier \"{}\", playerNumber \"{}\", suffix \"{}\" (comprising \"{}\" and \"{}\")", 
          std::string(match[1]), std::string(match[2]), std::string(match[3]), std::string(match[4]), std::string(match[5]));

    if (identifier)
      *identifier = match[1];
    if (playerNumber)
      *playerNumber = std::stoi(match[2]);
    if (suffix)
      *suffix = match[3]; // suffix must be present
  }
  else if (std::regex_search(logfileName, match, ipsOld))
  {
    ASSERT(match.size() == 4);
    TLOGD(tlogger, "[BH style] identifier \"{}\", playerNumber \"{}\", suffix \"{}\"", 
          std::string(match[1]), std::string(match[2]), std::string(match[3]));

    if (identifier)
      *identifier = match[1];
    if (playerNumber)
      *playerNumber = std::stoi(match[2]);
    if (suffix)
      *suffix = match[3].matched ? match[3].str().substr(1) : "";
  }
  else {
    TLOGE(tlogger, "could not parse identifier/player/suffix from log file name");
    return;
  }
}
