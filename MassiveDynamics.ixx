import <iostream>;

import "rang.hpp";

import MxScene;
import MxSolver;
import MxConsoleLogger;
import MxDebugHelper;

using namespace std;

int main()
{
	unique_ptr<IMxLogger> consoleLogger = make_unique<MxConsoleLogger>();
	consoleLogger->LogTitle("Started Console Logger, injecting into Debug Helper...");

	unique_ptr<IMxDebugHelper> debugHelper = make_unique<MxDebugHelper>(move(consoleLogger));

	debugHelper->Log("Logging from debug helper");

	cout << "Logger should be null here, is it?: " << (consoleLogger.get() == nullptr ? "YES" : "NO");

	return 0;
}
