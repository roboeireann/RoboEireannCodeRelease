/**
 * @file Annotation.h
 * @author Andreas Stolpmann
 * @author Rudi Villing (modifications)
 */

#pragma once

#include "Debugging/AnnotationManager.h"
#include "Debugging/Debugging.h"

// #include "fmt/core.h"
#include "TextLogging/TextLogging.h"

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
#define ANNOTATION(name_, message_)                                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    char buf_[1024];                                                                                                   \
    OutTextMemory text_(sizeof(buf_), buf_);                                                                           \
    text_ << name_ << message_;                                                                                        \
    Global::getAnnotationManager().add().write(text_.data(), text_.size());                                            \
  } while (false)


/**
 * A macro for sending annotation messages with fmtlib style formatting
 * 
 * usage:
 * 
 *   ANNOTATON_FMT("someName", "someFormatString {}", argsToSubstituteIntoFormatString...);
 */
#define ANNOTATION_FMT(name_, ...)                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    fmt::memory_buffer msgBuf_;                                                                                        \
    fmt::format_to(std::back_inserter(msgBuf_), __VA_ARGS__);                                                          \
    msgBuf_.push_back('\0');                                                                                           \
    ANNOTATION(name_, fmt::to_string(msgBuf_));                                                                        \
  } while (false)
