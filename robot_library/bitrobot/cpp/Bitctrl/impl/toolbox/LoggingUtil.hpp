#pragma once
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include "../fmt/format.h"

#ifndef __QNX__
#include <cstdint>
#endif

#ifdef __GNUC__
#if __GNUC__ < 4
    #ifndef nullptr
    #define nullptr 0   // GCC 4.4 doesn't support nullptr yet...
    #endif
#elif __GNUC__ == 4 && __GNUC_MINOR__ < 6
    #ifndef nullptr
    #define nullptr 0   // GCC 4.4 doesn't support nullptr yet...
    #endif
#endif
#endif

namespace logger {

enum {
    LEVEL_DEFAULT = 0,
    LEVEL_COMPUTE
};

enum {
    INFO_MSG = 0,
    INFO_SIGNAL,
    INFO_ERROR,
    INFO_WARN,
    INFO_TEMP,
    INFO_FUNCTION
};

template<int nLogger, int nLevel>
class Logger{
public:
    typedef std::uint32_t tick_t;

    class LoggerInfo {
    public:
        LoggerInfo():
            enabled(false),
            level(LEVEL_DEFAULT), m(1), r(0), ext(nullptr)
        {}

        void set_freq(int m, int r) {
            this->m = m; this->r = r;
        }
        bool valid_at(const tick_t& t) const {
            return (t % m) == r;
        }

        bool enabled;
        int level; // The default
        int m, r;  // This logger is valid
                   //      when tick % m == r
        FILE* ext; // This file will record the logging info
    };

    Logger() {
        loop = 0;
        std::fill(level, level + nLevel, false);
        std::fill(cache, cache + nLogger, false);
        level[LEVEL_DEFAULT] = true;
		level[LEVEL_COMPUTE] = true;
        refresh();
    }

    void tick(tick_t abs_tick) {
        loop = abs_tick;
        refresh();
    }
    void tick() {
        ++loop;
        refresh();
    }
    tick_t  curr_tick() const {
        return loop;
    }
    void refresh() {
        for (int i = 0; i < nLogger; ++i) {
            cache[i] = is_valid(i);
        }
    }

    // Enable/Disable the corresponding log-level/logger name
    bool enable_level(int name, bool on = true) {
        if (name >= 0 && name < nLevel) {
            level[name] = on;
            refresh();
            return true;
        }
        return false;
    }
    bool enable(int name, bool on = true) {
        if (name >= 0 && name < nLogger) {
            info[name].enabled = on;
            cache[name] = is_valid(name);
            return true;
        }
        return false;
    }
    bool config(int name, int level = LEVEL_DEFAULT, int m = 1, int r = 0, FILE* ext_file = nullptr) {
        if (name >= 0 && name < nLogger) {
            LoggerInfo& one = info[name];
            one.level = level;
            one.m = m;
            one.r = r;
            one.ext = ext_file;
            return true;
        }
        return false;
    }
    LoggerInfo* option(int name) {
        if (name >= 0 && name < nLogger) {
            return info + name;
        }
        return nullptr;
    }

    void operator ()(int name, const char* msg) {
        if (cache[name]) {
			if (!__prefix.empty())
				printf("%s ", __prefix.c_str());
            printf("[%u] %s\n", loop, msg);
            if (info[name].ext ) {
                fprintf(info[name].ext, "[%u] %s\n", loop, msg);
            }
        }
    }

    template<class T, class... Rest>
    void operator ()(int name, const char* fmt_str, const T& a, const Rest&... b) {
        if (cache[name])
            return (*this)(name, fmt::format(fmt_str, a, b...).c_str());
    }

	std::string __prefix;
private:
    bool is_valid(int name) const {
        if (name >= 0 && name < nLogger) {
            const LoggerInfo& one = info[name];
            if (one.enabled && level[one.level]) {
                return one.valid_at(curr_tick());
            }
        }
        return false;
    }

    tick_t loop;
    bool   level[nLevel];       // which level is currently enabled
    LoggerInfo info[nLogger];   // information
    bool   cache[nLogger];      // Caching who is valid at current tick
};

}


extern logger::Logger<16, 8> loginfo;
