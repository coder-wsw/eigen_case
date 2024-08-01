#include "algo_visual.hpp"
#include "gen_coord.hpp"
#include "inv_case.hpp"
#include "pcl_visual.hpp"

void print(int a, int b)
{
    std::cout << "print:" << a << b << std::endl;
}

int main(int, char*[])
{
    Run(0, print, 1, 2);
    Run(0, inv_case);
    Run(0, gen_coord_example);
    Run(1, pcl_visual);
    Run(0, algo_visual);
}