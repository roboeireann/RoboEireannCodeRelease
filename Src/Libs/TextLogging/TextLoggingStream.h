/**
 * @file TextLoggingStream.h
 *
 * This file declares functions to aid with logging text (including added standard
 * information) to text log files. Think of it as a better version of printf/cout
 * 
 * For ostream style output to text logs, use the macros in this file.
 * 
 * Note: the performace of TLOG_STREAM is **much** worse (e.g. 5x) that TLOG due to how
 * stream insertion is implemented in std. Avoid unless absolutely required.
 * 
 * Note: this code is thread safe and can operate in synchronous or asynchronous
 * mode. In asynchronous mode, the output to stdout or destination takes place
 * in an async thread (allowing the calling thread to avoid blocking on I/O).
 *
 * @author Rudi Villing
 */

#pragma once

#include "TextLogging.h"

#include "fmt/format.h"

#include <sstream>


// ============================================================================
// the macros...
// ============================================================================

#ifndef TLOG_DISABLED

namespace detail
{
  class FmtStreambuf : public std::streambuf
  {
  public:
    explicit FmtStreambuf(fmt::memory_buffer &buf) : buffer_(buf) {}

  protected:
    int_type overflow(int_type ch) override
    {
      if (ch != traits_type::eof())
      {
        buffer_.push_back(static_cast<char>(ch));
        return ch;
      }
      return traits_type::eof();
    }
    std::streamsize xsputn(const char *s, std::streamsize count) override
    {
      buffer_.append(s, s + count);
      return count;
    }

  private:
    fmt::memory_buffer &buffer_;
  };

  struct LogStreamProxy
  {
    LogStreamProxy(TextLogger &logger, TextLogging::Level level)
        : logger_(logger), level_(level), sbuf_(buffer_), stream_(&sbuf_)
    {
    }

    ~LogStreamProxy()
    {
      // Passes a string_view (pointer + size) to the .cpp
      // Very low cost jump.
      TextLogging::doLogMsgStream_(logger_, level_, {buffer_.data(), buffer_.size()});
    }

    std::ostream &stream() { return stream_; }

    fmt::memory_buffer buffer_;

    TextLogger &logger_;
    TextLogging::Level level_;
    FmtStreambuf sbuf_;
    std::ostream stream_;
  };
}

// we optimise for loggers that are disabled in the following
#define TLOG_STREAM_IMPL(logger_, lvl_, expr_)                                                                         \
  do                                                                                                                   \
  {                                                                                                                    \
    if (logger_.isEnabled(lvl_) && TextLogging::isLoggingEnabled.load(std::memory_order_relaxed)) [[unlikely]]         \
    {                                                                                                                  \
      ::detail::LogStreamProxy(logger_, lvl_).stream() << expr_;                                                       \
    }                                                                                                                  \
  } while (0)

#define TLOGF_STREAM(logger_,insertable_) TLOG_STREAM_IMPL(logger_, TextLogging::FATAL, insertable_)
#define TLOGE_STREAM(logger_,insertable_) TLOG_STREAM_IMPL(logger_, TextLogging::ERROR, insertable_)
#define TLOGW_STREAM(logger_,insertable_) TLOG_STREAM_IMPL(logger_, TextLogging::WARNING, insertable_)
#define TLOGI_STREAM(logger_,insertable_) TLOG_STREAM_IMPL(logger_, TextLogging::INFO, insertable_)
#define TLOGD_STREAM(logger_,insertable_) TLOG_STREAM_IMPL(logger_, TextLogging::DEBUG, insertable_)
#define TLOGV_STREAM(logger_,insertable_) TLOG_STREAM_IMPL(logger_, TextLogging::VERBOSE, insertable_)

#else // TLOG_DISABLED

#define TLOGF_STREAM(insertable_) (void)0
#define TLOGE_STREAM(insertable_) (void)0
#define TLOGW_STREAM(insertable_) (void)0
#define TLOGI_STREAM(insertable_) (void)0
#define TLOGD_STREAM(insertable_) (void)0
#define TLOGV_STREAM(insertable_) (void)0

#endif // TLOG_DISABLED
