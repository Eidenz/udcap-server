//
// Created by max_3 on 2025/5/9.
//
#include "BoostHelper.hpp"

std::pair<iterator, bool> match_whitespace(iterator begin, iterator)
{
    return std::make_pair(begin, true);
}
