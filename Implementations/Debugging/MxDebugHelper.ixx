export module MxDebugHelper;

import "rang.hpp";

export import IMxDebugHelper;

export struct MxDebugHelper : IMxDebugHelper {
    MxDebugHelper() = delete;

    MxDebugHelper(std::unique_ptr<IMxLogger> logger)
    {
        _logger = std::move(logger);
    }

    void Log(std::string string) override {
        _logger->Log(string);
    }

private:

    std::unique_ptr<IMxLogger> _logger;
};