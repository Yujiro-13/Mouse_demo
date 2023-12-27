#ifndef SEARCH_HPP
#define SEARCH_HPP

#include "Micromouse.hpp"

class Search : public Micromouse
{
    void main_task() override;
};

class All_Search : public Micromouse
{
    void main_task() override;
};

#endif // SEARCH_HPP