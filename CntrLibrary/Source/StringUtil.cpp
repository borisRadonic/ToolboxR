/******************************************************************************
The MIT License(MIT)

ToolboxR Control Library
https://github.com/borisRadonic/ToolboxR

Copyright(c) 2023 Boris Radonic

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

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