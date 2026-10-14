/**
 * @file Joints.cpp
 *
 * This file defines speakable joint names for error messages
 *
 * @author Rudi Villing
 */

#include "Joints.h"

const char * Joints::jointLongNames[numOfJoints] = {
  "head-yaw",
  "head-pitch",

  "left-shoulder-pitch",
  "left-shoulder-roll",
  "left-elbow-yaw",
  "left-elbow-roll",
  "left-wrist-yaw",
  "left-hand",

  "right-shoulder-pitch",
  "right-shoulder-roll",
  "right-elbow-yaw",
  "right-elbow-roll",
  "right-wrist-yaw",
  "right-hand",

  "left-hip-yaw-pitch",
  "left-hip-roll",
  "left-hip-pitch",
  "left-knee-pitch",
  "left-ankle-pitch",
  "left-ankle-roll",

  "right-hip-yaw-pitch",
  "right-hip-roll",
  "right-hip-pitch",
  "right-knee-pitch",
  "right-ankle-pitch",
  "right-ankle-roll"
};