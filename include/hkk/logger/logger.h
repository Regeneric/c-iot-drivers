#pragma once

#ifdef HLOGGER_HEADER
#include HLOGGER_HEADER
#endif

#ifndef HFATAL
#define HFATAL(...) do {} while (0)
#endif

#ifndef HERROR
#define HERROR(...) do {} while (0)
#endif

#ifndef HWARN
#define HWARN(...) do {} while (0)
#endif

#ifndef HINFO
#define HINFO(...) do {} while (0)
#endif

#ifndef HDEBUG
#define HDEBUG(...) do {} while (0)
#endif

#ifndef HTRACE
#define HTRACE(...) do {} while (0)
#endif