#include "StringUtil.h"

namespace CntrlLibrary
{
    std::string StringUtil::ltrim(const std::string& s)
    {
        size_t start = s.find_first_not_of(WHITESPACE);
        return (start == std::string::npos) ? "" : s.substr(start);
    }

    std::string StringUtil::rtrim(const std::string& s)
    {
        size_t end = s.find_last_not_of(WHITESPACE);
        return (end == std::string::npos) ? "" : s.substr(0, end + 1);
    }

    std::string StringUtil::trim(const std::string& s)
    {
        return rtrim(ltrim(s));
    }


    std::vector<std::string> StringUtil::split(std::string str, std::string delimiter)
    {
        std::vector<std::string> substrings;
        size_t pos = 0;

        while ((pos = str.find(delimiter)) != std::string::npos) {
            substrings.push_back(str.substr(0, pos));
            str.erase(0, pos + delimiter.length());
        }

        substrings.push_back(str);
        return substrings;
    }

    void StringUtil::remove_substring(std::string& str, const std::string& remove)
    {
        size_t pos = 0;

        while ((pos = str.find(remove, pos)) != std::string::npos)
        {
            str.erase(pos, remove.length());
        }
    }

    bool StringUtil::is_number(const std::string& str)
    {
        for (char const& c : str)
        {
            if (c != '-')
            {
                if (std::isdigit(c) == 0)
                {
                    return false;
                }
            }
        }
        return true;
    }
}