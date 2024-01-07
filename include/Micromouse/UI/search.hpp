#ifndef SEARCH_HPP
#define SEARCH_HPP

#include <iostream>
#include "UI.hpp"

class Search : public UI
{
    void main_task() override;
};

class All_Search : public UI
{
    void main_task() override;
};

#endif // SEARCH_HPP