export module MxConsoleLogger;

export import IMxLogger;

import "rang.hpp";

using namespace Console;
using namespace std;

export struct MxConsoleLogger : virtual IMxLogger {
    void Log(const std::string& text, Color foreground, Color background, Style style) override {

        bool invert = style == Style::Inverted;

        if (invert)
            cout << rang::style::reversed;
        else
            applyRangStyle(style);

        applyColor(foreground, true, invert && foreground != background);
        applyColor(background, false, invert && foreground != background);

        std::cout << text << std::endl;
        std::cout << rang::style::reset;
    }
    void LogTitle(const std::string& text) override {
        Log("### " + text, Color::Default, Color::Blue, Style::Default);
    }
private:

    void setRangColorFg(Color color) {
        std::cout << (
            color == Color::Black ? rang::fg::black :
            color == Color::Red ? rang::fg::red :
            color == Color::Green ? rang::fg::green :
            color == Color::Yellow ? rang::fg::yellow :
            color == Color::Blue ? rang::fg::blue :
            color == Color::Magenta ? rang::fg::magenta :
            color == Color::Cyan ? rang::fg::cyan :
            color == Color::Gray ? rang::fg::gray :
            color == Color::Default ? rang::fg::reset :
            rang::fg::reset);
    }

    void setRangColorBg(Color color) {
        std::cout << (
            color == Color::Black ? rang::bg::black :
            color == Color::Red ? rang::bg::red :
            color == Color::Green ? rang::bg::green :
            color == Color::Yellow ? rang::bg::yellow :
            color == Color::Blue ? rang::bg::blue :
            color == Color::Magenta ? rang::bg::magenta :
            color == Color::Cyan ? rang::bg::cyan :
            color == Color::Gray ? rang::bg::gray :
            color == Color::Default ? rang::bg::reset :
            rang::bg::reset);
    }

    void setRangColorFgB(Color color) {
        if (color == Color::Default) {
            cout << rang::fg::reset;
            return;
        }
        std::cout << (
            color == Color::Black ? rang::fgB::black :
            color == Color::Red ? rang::fgB::red :
            color == Color::Green ? rang::fgB::green :
            color == Color::Yellow ? rang::fgB::yellow :
            color == Color::Blue ? rang::fgB::blue :
            color == Color::Magenta ? rang::fgB::magenta :
            color == Color::Cyan ? rang::fgB::cyan :
            color == Color::Gray ? rang::fgB::gray :
            rang::fgB::gray);
    }

    void setRangColorBgB(Color color) {
        if (color == Color::Default) {
            cout << rang::bg::reset;
            return;
        }
        std::cout << (
            color == Color::Black ? rang::bgB::black :
            color == Color::Red ? rang::bgB::red :
            color == Color::Green ? rang::bgB::green :
            color == Color::Yellow ? rang::bgB::yellow :
            color == Color::Blue ? rang::bgB::blue :
            color == Color::Magenta ? rang::bgB::magenta :
            color == Color::Cyan ? rang::bgB::cyan :
            color == Color::Gray ? rang::bgB::gray :
            rang::bgB::gray);
    }

    void applyRangStyle(Style color) {
        cout << (
            color == Style::Bold ? rang::style::bold :
            color == Style::Italic ? rang::style::italic :
            color == Style::Underline ? rang::style::underline :
            color == Style::Crossed ? rang::style::crossed :
            rang::style::reset);
    }

    void applyColor(Color color, bool foreground, bool inverted) {
        if (foreground) {
            if (!inverted) setRangColorFgB(color);
            else setRangColorFg(color);
            return;
        }
        if (inverted) setRangColorBgB(color);
        else setRangColorBg(color);
    }
};