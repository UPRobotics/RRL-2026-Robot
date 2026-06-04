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

// ── NATO phonetic alphabet ────────────────────────────────

static const std::unordered_map<std::string, std::string>& natoAlphabet() {
    static const std::unordered_map<std::string, std::string> m = {
        {"alpha","A"},{"bravo","B"},{"charlie","C"},{"delta","D"},
        {"echo","E"},{"foxtrot","F"},{"golf","G"},{"hotel","H"},
        {"india","I"},{"juliet","J"},{"kilo","K"},{"lima","L"},
        {"mike","M"},{"november","N"},{"oscar","O"},{"papa","P"},
        {"quebec","Q"},{"romeo","R"},{"sierra","S"},{"tango","T"},
        {"uniform","U"},{"victor","V"},{"whiskey","W"},{"x-ray","X"},
        {"yankee","Y"},{"zulu","Z"}
    };
    return m;
}

// Replace NATO phonetic words with uppercase letters: "alpha one" → "A one"
static std::string applyNato(const std::string& text) {
    auto words = split(toLower(text));
    auto origWords = split(text);
    std::string out;
    for (size_t i = 0; i < words.size(); ++i) {
        if (!out.empty()) out += ' ';
        auto it = natoAlphabet().find(words[i]);
        out += (it != natoAlphabet().end()) ? it->second : origWords[i];
    }
    return out;
}

// Merge runs of single uppercase letters followed by digit tokens: "A 1" → "A1", "B C 2" → "BC2"
static std::string mergeAlphanumeric(const std::string& text) {
    auto words = split(text);
    std::vector<std::string> out;
    size_t i = 0;
    while (i < words.size()) {
        bool isSingleCap = (words[i].size() == 1 && std::isupper((unsigned char)words[i][0]));
        if (isSingleCap) {
            std::string code = words[i++];
            while (i < words.size() && words[i].size() == 1 &&
                   std::isupper((unsigned char)words[i][0])) {
                code += words[i++];
            }
            while (i < words.size() && !words[i].empty() &&
                   std::all_of(words[i].begin(), words[i].end(), ::isdigit)) {
                code += words[i++];
            }
            out.push_back(code);
        } else {
            out.push_back(words[i++]);
        }
    }
    std::string result;
    for (size_t j = 0; j < out.size(); ++j) {
        if (j) result += ' ';
        result += out[j];
    }
    return result;
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

static bool isScaleWord(const std::string& w) {
    return enScale().count(w) > 0;
}

// Combine a run that contains scale words (hundred/thousand/…) into one number.
static long long parseEnglishRun(const std::vector<std::string>& words, size_t start, size_t end) {
    long long result = 0;
    long long current = 0;

    for (size_t i = start; i < end; ++i) {
        const std::string& w = words[i];
        if (w == "and") continue;
        if (w == "a") { if (current == 0) current = 1; continue; }

        auto uit = enUnits().find(w);
        if (uit != enUnits().end()) { current += uit->second; continue; }

        auto sit = enScale().find(w);
        if (sit != enScale().end()) {
            long long scale = sit->second;
            if (scale == 100) {
                if (current == 0) current = 1;
                current *= 100;
            } else {
                if (current == 0) current = 1;
                current *= scale;
                result += current;
                current = 0;
            }
        }
    }
    return result + current;
}

std::string NumberConverter::convertEnglish(const std::string& text) {
    auto words = split(toLower(text));
    if (words.empty()) return text;

    auto origWords = split(text);

    std::string out;
    size_t i = 0;
    while (i < words.size()) {
        const std::string& w = words[i];

        // Scale words anchor a compound number run ("two hundred", "a thousand …")
        if (isScaleWord(w) ||
            ((enUnits().count(w) && w != "and" && w != "a") &&
             i + 1 < words.size() &&
             (isScaleWord(words[i + 1]) ||
              (words[i + 1] == "and") ||
              (enUnits().count(words[i + 1]) && enUnits().at(words[i + 1]) >= 10))))
        {
            // Find extent of this compound-number run
            size_t j = i + 1;
            auto isRunWord = [](const std::string& x) {
                return enUnits().count(x) || enScale().count(x) || x == "and" || x == "a";
            };
            // Only extend while scale words keep the run compound
            bool hasScale = isScaleWord(w);
            while (j < words.size() && isRunWord(words[j])) {
                if (isScaleWord(words[j])) hasScale = true;
                ++j;
            }
            if (hasScale) {
                if (!out.empty()) out += ' ';
                out += std::to_string(parseEnglishRun(words, i, j));
                i = j;
                continue;
            }
        }

        // Single digit word (zero–nine, ten–nineteen, twenty…ninety) → individual token
        if (enUnits().count(w) && w != "and" && w != "a") {
            if (!out.empty()) out += ' ';
            out += std::to_string(enUnits().at(w));
            ++i;
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
    // NATO substitution → number words → alphanumeric merge
    return mergeAlphanumeric(convertEnglish(applyNato(text)));
}

}  // namespace mic_pkg
