/**
 * @file GameInfo.h
 * The file declares a struct that encapsulates the structure RoboCupGameControlData
 * defined in the file RoboCupGameControlData.h that is provided with the GameController.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Communication/RoboCupGameControlData.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Tools/Streams/AutoStreamable.h"

struct ReceivedGameControlData : public RoboCup::RoboCupGameControlData, public Streamable
{
public:
  struct RobotInfo : public RoboCup::RobotInfo
  {
    bool isPenalized() const { return penalty != PENALTY_NONE; }
    bool isPenalized(uint8_t penaltyCheck) const { return penalty == penaltyCheck; }

  private:
    static void reg();
  };

  struct TeamInfo : public RoboCup::TeamInfo
  {
    RobotInfo& getPlayer(int i) { return reinterpret_cast<RobotInfo&>(players[i]); }
    const RobotInfo& getPlayer(int i) const { return reinterpret_cast<const RobotInfo&>(players[i]); }

    int getSubstitutedPlayerNumber(int number) const;

  private:
    static void reg();
  };

  unsigned timeLastPacketReceived = 0; // this is the real time and not the frame time (for time sync reasons)

  ReceivedGameControlData();

  std::string getStateString() const;
  std::string getPenaltyString() const;
  std::string getPlayerNumberAndColorString() const;

  // team functions...

  int ourTeamIndex() const { return (teams[0].teamNumber == Global::getSettings().teamNumber) ? 0 : 1; }

  TeamInfo& ourTeam() { return reinterpret_cast<TeamInfo&>(teams[ourTeamIndex()]); }
  const TeamInfo& ourTeam() const { return reinterpret_cast<const TeamInfo&>(teams[ourTeamIndex()]); }

  TeamInfo& opponentTeam() { return reinterpret_cast<TeamInfo&>(teams[1 - ourTeamIndex()]); }
  const TeamInfo& opponentTeam() const { return reinterpret_cast<const TeamInfo&>(teams[1 - ourTeamIndex()]); }

  // functions related to the this robot player... 

  int playerIndex() const { return toPlayerIndex(Global::getSettings().playerNumber); }

  RobotInfo& playerInfo() { return ourTeam().getPlayer(playerIndex()); }
  const RobotInfo& playerInfo() const { return ourTeam().getPlayer(playerIndex()); }

  bool isGoalkeeper() const { return toPlayerNumber(playerIndex()) == 1; }
  bool isPenalized() const { return playerInfo().isPenalized(); };
  bool isPenalized(uint8_t penalty) const { return playerInfo().isPenalized(penalty); };

  bool isOurKick() const { return (kickingTeam == teams[ourTeamIndex()].teamNumber); } 

  bool isPlayingLeftToRight() const { return teams[0].teamNumber == Global::getSettings().teamNumber; } 

  static int toPlayerNumber(int index) { return index + Settings::lowestValidPlayerNumber; } // TODO: this duplicates code in Settings now - should we delete it?
  static int toPlayerIndex(int number) { return number - Settings::lowestValidPlayerNumber; } // TODO: this duplicates code in Settings now - should we delete it?

protected:
  /**
   * Read this object from a stream.
   * @param stream The stream from which the object is read.
   */
  void read(In& stream) override;

  /**
   * Write this object to a stream.
   * @param stream The stream to which the object is written.
   */
  void write(Out& stream) const override;

private:
  static void reg();

  using RoboCup::RoboCupGameControlData::header; // Hide, because it is not streamed
  using RoboCup::RoboCupGameControlData::version; // Hide, because it is not streamed
};


/** The game info as sent by the GameController */
STREAMABLE_WITH_BASE(GameInfo, ReceivedGameControlData,
{
  ENUM(Mode,
  {,
    unstiff,
    active,
    calibration,
  });

  ENUM(BallFreeState,
  {,
    ballIsFree,
    notFreeKickoff,
    notFreeSetPlay,
  });

  bool isKickoff() const { return ballFreeState == notFreeKickoff; }
  bool isBallFree() const { return ballFreeState == ballIsFree; }

  /** Draws the game time in the scene view. */
  void draw() const;
  
  /**** streamable fields follow - note comma at end of this line ****/,

  (bool)(false) gcActive,
  (bool)(false) stateIsGuessed,

  // all following fields added in 2024 to make GameInfo more of a one stop shop
  (BallFreeState)(ballIsFree) ballFreeState, // mainly used during kickoff and set plays

  (int)(1) playerNumber,
  (Mode)(unstiff) playerMode,
  (bool)(true) playingLeftToRight, // from the GC operator point of view
});
