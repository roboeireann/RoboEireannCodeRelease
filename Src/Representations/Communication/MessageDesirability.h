/**
 * @file MessageDesirabilty.h
 * 
 * Declaration of representation that determines whether it is desirable to send
 * a team message in the current frame or not
 * 
 * @author Aidan Colgan
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
 * Representation for Team communication message desirable to send and the reasons why
 */
STREAMABLE(MessageDesirability,
{
  ENUM(ReasonToSend,
  {,
    noReasonToSend,
    pingNeeded, /**< we need to send a message so other robots don't think we're gone */
    readySetPingNeeded,
    roleChanged,
    ballMoved,
    ballUpdateNeeded, ///< keep the team ball valid by sending updates often enough
    playerMoved,
    returnedFromPenalty,
    uprightStatusChanged,
    standbyReadyGestureDetected,
    kickedSinceRestart,
  });

  ENUM(ReasonNotToSend,
  {,
    noReasonNotToSend,
    messageBudgetPreventsSending,
    statePreventsSending, // never allow sending in these states
    minIntervalPreventsSending,
    readySetConstrainsSending, // mostly don't allow sending in these states
    messageRatePreventsSending,
  });

  // note it is possible for both of the following functions to be true since
  // certain reasons to send override reasons not to send
  bool hasReasonToSend() { return reasonToSend != noReasonToSend; }
  bool hasReasonNotToSend() { return reasonNotToSend != noReasonNotToSend; }

  /***** separate header from streamable members, note comma at end *****/,

  (unsigned)(0) messageBudgetThisHalf, //< the message budget remaining in this half
  (float)(0) remainingTeamMessageRateRatio, ///< the ratio of the our message rate to target rate, above 1 means we have capacity
  (bool)(false) shouldSend, //< decision based on prioritisation of reasonToSend and reasonNotToSend
  (ReasonToSend)(noReasonToSend) reasonToSend, //< the first reason why we should send (if any)
  (ReasonNotToSend)(noReasonNotToSend) reasonNotToSend, //< the first reason why we should not send (if any)
});
