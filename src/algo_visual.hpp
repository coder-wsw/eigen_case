#include "com_header.h"

Eigen::VectorXd rande(int num, int range)
{
    std::vector<double> vec;
    srand((unsigned)time(0));
    for (int i = 0; i < num; i++) {
        auto x = (rand() % range);
        vec.push_back(x);
    }

    std::sort(vec.begin(), vec.end());
    auto last = std::unique(vec.begin(), vec.end());
    vec.erase(last, vec.end());

    Eigen::VectorXd vet = Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size());
    Debug(vet);
    return vet;
}

auto to_std_vector(Eigen::VectorXd vec)
{
    std::vector<double> vec_std(vec.data(), vec.data() + vec.size());
    return vec_std;
}

void algo_visual()
{
    auto vec = rande(10, 10);
    auto x = to_std_vector(vec);

    // std::vector<double> x = mp::randn(5000, 5, 2);

    auto h = mp::hist(x);
    h->normalization(mp::histogram::normalization::pdf);
    mp::hold(mp::on);

    double mu = 5;
    double sigma = 2;
    auto f = [&](double y) {
        return exp(-pow((y - mu), 2.) / (2. * pow(sigma, 2.))) /
               (sigma * sqrt(2. * mp::pi));
    };
    mp::fplot(f, std::array<double, 2>{-5, 15})->line_width(1.5);
    mp::show();
}
