/*
 * Debug helper: filter which messages are printed during execution by defining the relevance of each message
 * 2013 - Mario Cecchi - http://meriw.com
 *
 * Notes:
 *    - This code is C++11. Remember to use the -std=c++11 (or -std=c++0x if using g++4.6) flag when compiling, otherwise it won't even compile.
 *    - If you do not want to use std::cerr as the output stream for errors, just change the private constructor to use std::cout.
 *    - Should work just the same as the default ostreams with the << operator.
 *
 * Usage:
 *    debug_set_level(DebugLevel::INFO);
 *
 *    debug_debug << "Called debug_debug and set level to Debug::DEBUG\n"; // will not be printed because level is set to INFO
 *    debug_info << "Debug is running! This is an information!\n";
 *    debug_warning << "Warning! This code may be unstable!\n";
 *    debug_error << "Error!! You should not have tried to use this code!\n";
 *    debug_critical << "****SYSTEM FAILURE. ABORTING EXECUTION...****";
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include <iostream>

enum class DebugLevel : char { CRITICAL = 0, ERROR = 1, WARNING = 2, INFO = 3, DEBUG = 4 };

class Debug {
public:
	static Debug& instance() {
		static Debug instance;
		return instance;
	}

	Debug& prepareLevel(DebugLevel n) {
		msgLevel = n;
		return *this;
	}

	template <typename T>
	void print(const T& msg) {
		if (msgLevel <= DebugLevel::WARNING) err << msg;
		else out << msg;
	}

	// To handle std::endl
	Debug& operator << (std::ostream&(*f)(std::ostream&)) {
		if (msgLevel <= DebugLevel::WARNING) err << f;
		else out << f;
		return *this;
	}

	template <typename T>
	Debug& operator << (const T& msg) {
		print(msg);
		return *this;
	}

	void setDebugLevel(DebugLevel n) { debugLevel = n; }
	DebugLevel level() { return debugLevel; }

private:
	Debug() : out(std::cout), err(std::cerr), debugLevel(DebugLevel::WARNING) {}
	std::ostream& out;
	std::ostream& err;
	DebugLevel debugLevel;
	volatile DebugLevel msgLevel;
};

#define debug(lvl) if (lvl <= Debug::instance().level()) Debug::instance().prepareLevel(lvl)
#define debug_set_level(lvl) Debug::instance().setDebugLevel(lvl)
#define dcrit debug(DebugLevel::CRITICAL)
#define derr debug(DebugLevel::ERROR)
#define dwarn debug(DebugLevel::WARNING)
#define dinfo debug(DebugLevel::INFO)
#define ddebug debug(DebugLevel::DEBUG)

#endif /* DEBUG_H_ */
