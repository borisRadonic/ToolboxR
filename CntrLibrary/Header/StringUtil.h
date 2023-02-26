#pragma once
#include <string>
#include <vector>
#include <regex>

namespace CntrlLibrary
{

#define WHITESPACE "\n\r\t\f\v"

    class StringUtil
    {
    public:

        static std::string ltrim(const std::string& s);

        static std::string rtrim(const std::string& s);

        static std::string trim(const std::string& s);

        static std::vector<std::string> split(std::string str, std::string delimiter);

        static void remove_substring(std::string& str, const std::string& remove);

        static bool is_number(const std::string& str);

    };
};
