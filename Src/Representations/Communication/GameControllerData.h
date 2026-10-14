/**
 * @file GameControllerData.h
 *
 * This file declares a representation that contains the data received from a GameController.
 *
 * @author Thomas Röfer
 * @author Arne Hasselbring
 */

#pragma once

#include "Network/RoboCupGameControlData.h"
#include "Streaming/Streamable.h"
#include "Streaming/Global.h"
#include "Framework/Settings.h"

#ifndef PENALTY_MANUAL
#define PENALTY_MANUAL 99
#endif
#ifndef STATE_STANDBY
#define STATE_STANDBY 99
#endif

struct GameControllerData : public RoboCup::RoboCupGameControlData, public Streamable
{
  unsigned timeLastPacketReceived = 0; /**< Time when the last GameController packet has been received. */
  bool isTrueData = false; /**< true means the GameController packet reflects whistle transitions without added delay. */

  /** Constructor. */
  GameControllerData();

  int ownTeamIndex() const { return (teams[0].teamNumber == Global::getSettings().teamNumber) ? 0 : 1; }
  int opponentTeamIndex() const { return ownTeamIndex() == 0 ? 1 : 0; }

  bool isNoTeamKick() const { return kickingTeam == KICKING_TEAM_NONE; }
  bool isOwnTeamKick() const { return kickingTeam == Global::getSettings().teamNumber; }
  bool isOpponentTeamKick() const { return kickingTeam == teams[opponentTeamIndex()].teamNumber; }

  const RoboCup::TeamInfo& ownTeamInfo() const { return teams[ownTeamIndex()]; }
  const RoboCup::TeamInfo& opponentTeamInfo() const { return teams[opponentTeamIndex()]; }

  bool isPlayingLeftToRight() const { return teams[0].teamNumber == Global::getSettings().teamNumber; }

  const RoboCup::RobotInfo &ownPlayerInfo() const
  {
    return ownPlayerInfo(Global::getSettings().playerNumber);
  }

  const RoboCup::RobotInfo &ownPlayerInfo(int playerNumber) const
  {
    return ownTeamInfo().players[Settings::toPlayerIndex(playerNumber)];
  }

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
  using RoboCup::RoboCupGameControlData::header; // Hide, because it is not streamed
  using RoboCup::RoboCupGameControlData::version; // Hide, because it is not streamed

  static void reg();
};
