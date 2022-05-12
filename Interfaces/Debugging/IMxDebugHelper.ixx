export module IMxDebugHelper;

import <string>;

import "interface.h";

export import IMxLogger;

interface IMxDebugHelper {
	virtual ~IMxDebugHelper() = default;
	virtual void Log(std::string string) = 0;
};
