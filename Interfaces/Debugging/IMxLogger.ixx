export module IMxLogger;

import <iostream>;

import "interface.h";

export namespace Console {
    export enum class Color : char {
        Black,
        Red,
        Green,
        Yellow,
        Blue,
        Magenta,
        Cyan,
        Gray,
        Default
    };

    export enum class Style : char {
        Default,
        Inverted,
        Bold,
        Italic,
        Underline,
        Crossed,
    };
}

using namespace Console;

interface IMxLogger {
    virtual ~IMxLogger() = default;
    // TODO allow ORing styles as flags
    virtual void Log(const std::string& text, Color foreground = Color::Default, Color background = Color::Default, Style style = Style::Default) = 0;
    virtual void LogTitle(const std::string& text) = 0;
};
