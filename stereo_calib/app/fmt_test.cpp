#include <iostream>
#include <fmt/core.h>
#include <fmt/format.h>

int main(int argc, char** argv)
{
    for (int i = 0; i < 10; ++i)
    {
        std::cout << fmt::format("{:0>6}", i) << std::endl;
    }

    return 0;
}