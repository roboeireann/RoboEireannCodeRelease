#pragma once

// force the use of a variable in configurations where it may not be used
// (primarily the release configuration)
#define MAYBE_UNUSED(var_) static_cast<void>(var_)