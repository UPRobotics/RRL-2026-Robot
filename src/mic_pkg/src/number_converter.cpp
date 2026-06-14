#include "mic_pkg/number_converter.h"
#include <sstream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cctype>

namespace mic_pkg {

// ── helpers ──────────────────────────────────────────────

static std::string toLower(const std::string& s) {
    std::string r = s;
    std::transform(r.begin(), r.end(), r.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return r;
}

static std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> tokens;
    std::istringstream iss(s);
    std::string t;
    while (iss >> t) tokens.push_back(t);
    return tokens;
}

// ── English ──────────────────────────────────────────────

static const std::unordered_map<std::string, long long>& enUnits() {
    static const std::unordered_map<std::string, long long> m = {
        {"zero",0},{"one",1},{"two",2},{"three",3},{"four",4},{"five",5},
        {"six",6},{"seven",7},{"eight",8},{"nine",9},{"ten",10},
        {"eleven",11},{"twelve",12},{"thirteen",13},{"fourteen",14},
        {"fifteen",15},{"sixteen",16},{"seventeen",17},{"eighteen",18},
        {"nineteen",19},{"twenty",20},{"thirty",30},{"forty",40},
        {"fifty",50},{"sixty",60},{"seventy",70},{"eighty",80},{"ninety",90}
    };
    return m;
}

static const std::unordered_map<std::string, long long>& enScale() {
    static const std::unordered_map<std::string, long long> m = {
        {"hundred", 100}, {"thousand", 1000}, {"million", 1000000},
        {"billion", 1000000000LL}
    };
    return m;
}

static bool isEnNumber(const std::string& w) {
    return enUnits().count(w) || enScale().count(w) || w == "and" || w == "a";
}

// Parse a contiguous run of English number words into a number.
static long long parseEnglishRun(const std::vector<std::string>& words, size_t start, size_t end) {
    long long result = 0;
    long long current = 0;  // accumulator below thousand-scale

    for (size_t i = start; i < end; ++i) {
        const std::string& w = words[i];
        if (w == "and") continue;
        if (w == "a") {
            // "a hundred", "a thousand"
            if (current == 0) current = 1;
            continue;
        }

        auto uit = enUnits().find(w);
        if (uit != enUnits().end()) {
            current += uit->second;
            continue;
        }

        auto sit = enScale().find(w);
        if (sit != enScale().end()) {
            long long scale = sit->second;
            if (scale == 100) {
                if (current == 0) current = 1;
                current *= 100;
            } else {
                // thousand, million, billion
                if (current == 0) current = 1;
                current *= scale;
                result += current;
                current = 0;
            }
        }
    }
    result += current;
    return result;
}

std::string NumberConverter::convertEnglish(const std::string& text) {
    auto words = split(toLower(text));
    if (words.empty()) return text;

    // Preserve original casing word list for non-number words
    auto origWords = split(text);

    std::string out;
    size_t i = 0;
    while (i < words.size()) {
        if (isEnNumber(words[i]) && words[i] != "and" && words[i] != "a") {
            // Find the extent of the number run
            size_t j = i + 1;
            while (j < words.size() && isEnNumber(words[j])) ++j;

            // Must have at least one actual number word (not just "and"/"a")
            long long val = parseEnglishRun(words, i, j);
            if (!out.empty()) out += ' ';
            out += std::to_string(val);
            i = j;
        } else {
            if (!out.empty()) out += ' ';
            out += origWords[i];
            ++i;
        }
    }
    return out;
}

// ── Spanish ──────────────────────────────────────────────

static const std::unordered_map<std::string, long long>& esUnits() {
    static const std::unordered_map<std::string, long long> m = {
        {"cero",0},{"uno",1},{"una",1},{"un",1},{"dos",2},{"tres",3},
        {"cuatro",4},{"cinco",5},{"seis",6},{"siete",7},{"ocho",8},
        {"nueve",9},{"diez",10},{"once",11},{"doce",12},{"trece",13},
        {"catorce",14},{"quince",15},
        {"dieciseis",16},{"dieciséis",16},
        {"diecisiete",17},{"dieciocho",18},{"diecinueve",19},
        {"veinte",20},
        {"veintiuno",21},{"veintiún",21},{"veintiun",21},
        {"veintidos",22},{"veintidós",22},
        {"veintitres",23},{"veintitrés",23},
        {"veinticuatro",24},{"veinticinco",25},{"veintiseis",26},{"veintiséis",26},
        {"veintisiete",27},{"veintiocho",28},{"veintinueve",29},
        {"treinta",30},{"cuarenta",40},{"cincuenta",50},{"sesenta",60},
        {"setenta",70},{"ochenta",80},{"noventa",90},
        {"cien",100},{"ciento",100},
        {"doscientos",200},{"doscientas",200},
        {"trescientos",300},{"trescientas",300},
        {"cuatrocientos",400},{"cuatrocientas",400},
        {"quinientos",500},{"quinientas",500},
        {"seiscientos",600},{"seiscientas",600},
        {"setecientos",700},{"setecientas",700},
        {"ochocientos",800},{"ochocientas",800},
        {"novecientos",900},{"novecientas",900}
    };
    return m;
}

static const std::unordered_map<std::string, long long>& esScale() {
    static const std::unordered_map<std::string, long long> m = {
        {"mil", 1000}, {"millon", 1000000}, {"millón", 1000000},
        {"millones", 1000000}, {"billon", 1000000000000LL},
        {"billón", 1000000000000LL}, {"billones", 1000000000000LL}
    };
    return m;
}

static bool isEsNumber(const std::string& w) {
    return esUnits().count(w) || esScale().count(w) || w == "y";
}

static long long parseSpanishRun(const std::vector<std::string>& words, size_t start, size_t end) {
    long long result = 0;
    long long current = 0;

    for (size_t i = start; i < end; ++i) {
        const std::string& w = words[i];
        if (w == "y") continue;

        auto uit = esUnits().find(w);
        if (uit != esUnits().end()) {
            long long v = uit->second;
            if (v >= 100) {
                // cien/ciento/doscientos etc. — these are hundreds
                current += v;
            } else {
                current += v;
            }
            continue;
        }

        auto sit = esScale().find(w);
        if (sit != esScale().end()) {
            long long scale = sit->second;
            if (scale == 1000) {
                if (current == 0) current = 1;
                current *= 1000;
                result += current;
                current = 0;
            } else {
                // millón/billón
                if (current == 0) current = 1;
                current *= scale;
                result += current;
                current = 0;
            }
        }
    }
    result += current;
    return result;
}

std::string NumberConverter::convertSpanish(const std::string& text) {
    auto words = split(toLower(text));
    if (words.empty()) return text;

    auto origWords = split(text);

    std::string out;
    size_t i = 0;
    while (i < words.size()) {
        if (isEsNumber(words[i]) && words[i] != "y") {
            size_t j = i + 1;
            while (j < words.size() && isEsNumber(words[j])) ++j;

            long long val = parseSpanishRun(words, i, j);
            if (!out.empty()) out += ' ';
            out += std::to_string(val);
            i = j;
        } else {
            if (!out.empty()) out += ' ';
            out += origWords[i];
            ++i;
        }
    }
    return out;
}

// ── public API ───────────────────────────────────────────

std::string NumberConverter::convert(const std::string& text, const std::string& lang) {
    if (lang == "es") return convertSpanish(text);
    return convertEnglish(text);
}

}  // namespace mic_pkg
