/**
 * @file Annotation.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "AnnotationManager.h"
#include "Debugging.h"

#include "fmt/core.h"

/**
 * A macro for sending annotation messages.
 *
 * @param name The part of the system where this annotation originates.
 * @param message A message streamable as text.
 *
 * Examples:
 * <pre>
 * ANNOTATION("Behavior", "Kickin'!");
 * ANNOTATION("TacticProvider", "Changed Tactic from " << oldTactic << " to " << newTactic << ".");
 * </pre>
 */
#define ANNOTATION(name, message) \
  do \
  { \
    Global::getAnnotationManager().addAnnotation(); \
    Global::getAnnotationManager().getOut().out.text << name << message; \
    Global::getAnnotationManager().getOut().out.finishMessage(idAnnotation); \
  } \
  while(false)

/**
 * A macro for sending annotation messages with fmtlib style formatting
 * 
 * usage:
 * 
 *   ANNOTATON_FMT("someName", "someFormatString {}", argsToSubstituteIntoFormatString...);
 */
#define ANNOTATION_FMT(name, ...)  ANNOTATION(name, fmt::format(__VA_ARGS__))
