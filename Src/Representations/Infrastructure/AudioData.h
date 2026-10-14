/**
 * @file AudioData.h
 * The file declares a struct that stores audio data of up to four channels.
 * 0: back left microphone
 * 1: back right microphone
 * 2: front left microphone
 * 3: front right microphone
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(AudioData,
{
    using Sample = float,

    (unsigned)channels,
    (unsigned)sampleRate,
    (std::vector<Sample>)samples, /**< Samples are interleaved. */
});
