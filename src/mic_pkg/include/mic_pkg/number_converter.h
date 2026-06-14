#pragma once

#include <string>

namespace mic_pkg {

// Converts number words to digits in recognized text.
// Supports English and Spanish.
// Examples: "one hundred twenty three" → "123", "doscientos cuarenta y cinco" → "245"
class NumberConverter {
public:
    static std::string convert(const std::string& text, const std::string& lang);

private:
    static std::string convertEnglish(const std::string& text);
    static std::string convertSpanish(const std::string& text);
};

}  // namespace mic_pkg
