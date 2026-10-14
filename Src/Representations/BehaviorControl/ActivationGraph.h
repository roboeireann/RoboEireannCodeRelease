/**
 * @file ActivationGraph.h
 *
 * The graph of executed options and states.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(ActivationGraph,
{
  STREAMABLE(Node,
  {
    Node() = default;
    Node(const std::string &option, int depth, const std::string &state, int optionTime, int stateTime,
         const std::vector<std::string> &arguments);
    Node(const std::string &option, int depth, const std::string &state, int optionTime, int stateTime,
         const std::vector<std::string> &arguments, int status, const std::vector<std::string> &strings);

    /**** separate header from streamable fields - comma at end of comment ****/,

    (std::string) option,
    (int)(0) depth,
    (std::string) state,
    (int)(0) optionTime,
    (int)(0) stateTime,
    (std::vector<std::string>) arguments,
    (int)(-1) status, // RE addition - task return status if applicable
    (std::vector<std::string>) strings, // RE addition - additional strings
  });

  ActivationGraph()
  {
    graph.reserve(100);
  };

  /**** separate header from streamable fields - comma at end of comment ****/,

  (std::vector<Node>) graph,
});

inline ActivationGraph::Node::Node(const std::string &option, int depth, const std::string &state, int optionTime,
                                   int stateTime, const std::vector<std::string> &arguments)
    : option(option),
      depth(depth),
      state(state),
      optionTime(optionTime),
      stateTime(stateTime),
      arguments(arguments)
{}

inline ActivationGraph::Node::Node(const std::string &option, int depth, const std::string &state, int optionTime,
                                   int stateTime, const std::vector<std::string> &arguments, int status,
                                   const std::vector<std::string> &strings)
    : option(option),
      depth(depth),
      state(state),
      optionTime(optionTime),
      stateTime(stateTime),
      arguments(arguments),
      status(status),
      strings(strings)
{}

// FIXME - only needed for RE 2025 behaviour but part of BH2021 code release
STREAMABLE_WITH_BASE(TeamActivationGraph, ActivationGraph,
{,
});