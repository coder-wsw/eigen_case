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

/// 直线拟合 y = ax + b
void least_square_line()
{
    std::vector<double> x = mp::linspace(0, 1, 100);
    Debug_Vec(x);
    auto y =
        mp::transform(x, [&](double x) { return cos(x) + mp::rand(0, 1); });
    Debug_Vec(y);

    Eigen::MatrixXd MatX;
    Eigen::MatrixXd MatY;

    MatX.resize(x.size(), 2);
    MatY.resize(x.size(), 1);

    for (int i = 0; i < x.size(); i++) {
        MatX.row(i) = Eigen::Vector2d(x[i], 1);
        MatY(i, 0) = y[i];
    }

    auto coeffic =
        (MatX.transpose() * MatX).inverse() * MatX.transpose() * MatY;

    Debug(coeffic);
    std::vector<double> ry = mp::transform(
        x, [coeffic](auto x) { return coeffic(0, 0) * x + coeffic(1, 0); });

    mp::scatter(x, y);
    mp::hold(mp::on);
    mp::plot(x, ry);
    mp::show();
}

/// 曲线拟合(多项式拟合) y = a0x^0 + a1x^1 + ... + anx^n
void least_square_curve(int order)
{
    // std::vector<double> x = mp::linspace(0, 10, 11);
    std::vector<double> x{0,
                          0.0192341804504395,
                          0.0394501686096191,
                          0.059575080871582,
                          0.0790810585021973,
                          0.0792751312255859,
                          0.0987141132354736,
                          0.119336366653442,
                          0.138712167739868,
                          0.159000158309937,
                          0.178890228271484,
                          0.19960618019104,
                          0.219112157821655,
                          0.23919415473938,
                          0.259442090988159,
                          0.279186248779297,
                          0.299112319946289,
                          0.319219350814819,
                          0.339494228363037,
                          0.339675188064575,
                          0.359552145004272,
                          0.37941837310791,
                          0.399189233779907,
                          0.419828176498413,
                          0.439810276031494,
                          0.459331274032593,
                          0.479461193084717,
                          0.499663114547729,
                          0.519809246063232,
                          0.539092063903809,
                          0.559118270874023,
                          0.579315185546875,
                          0.598889112472534,
                          0.619685173034668,
                          0.638863086700439,
                          0.639052152633667,
                          0.658920288085938,
                          0.679149150848389,
                          0.699787139892578,
                          0.71905517578125,
                          0.73898720741272,
                          0.739143371582031,
                          0.758654117584229,
                          0.779210329055786,
                          0.799195289611816,
                          0.819046258926392,
                          0.839539289474487,
                          0.85923433303833,
                          0.87903618812561,
                          0.899263143539429,
                          0.919251203536987,
                          0.939138174057007,
                          0.959244251251221,
                          0.979074239730835,
                          0.998935222625732,
                          1.01904726028442,
                          1.0387852191925,
                          1.03895926475525,
                          1.05906510353088,
                          1.07873225212097,
                          1.09908628463745,
                          1.11907029151917,
                          1.13899827003479,
                          1.15879201889038};
    Debug_Vec(x);
    // auto y =
    //     mp::transform(x, [&](double x) { return cos(x) + mp::rand(0, 1); });
    // auto y = mp::transform(x, [&](double x) { return x; });

    std::vector<double> y{
        1.8,  1.86, 2.03, 2.08, 2.14, 2.14, 2.25, 2.36, 2.42, 2.59, 2.7,
        2.81, 2.87, 3.04, 3.15, 3.26, 3.32, 3.43, 3.54, 3.54, 3.6,  3.71,
        3.83, 3.94, 4.11, 4.22, 4.33, 4.44, 4.56, 4.67, 4.78, 4.84, 4.84,
        4.89, 4.89, 4.89, 4.95, 5.01, 5.06, 5.06, 5.06, 5.06, 5.01, 5.06,
        5.12, 5.18, 5.18, 5.23, 5.23, 5.23, 5.29, 5.34, 5.29, 5.4,  5.4,
        5.46, 5.51, 5.51, 5.51, 5.46, 5.4,  5.34, 5.34, 5.34};
    Debug_Vec(y);

    Eigen::MatrixXd MatX;
    Eigen::MatrixXd MatY;
    Eigen::VectorXd VecY;

    MatX.resize(x.size(), order);
    MatY.resize(x.size(), 1);

    VecY = Eigen::VectorXd::Map(&y.front(), y.size());

    for (int i = 0; i < x.size(); ++i) {
        for (int j = 0; j < order; ++j) {
            double e = std::pow(x[i], j);
            MatX(i, j) = e;
        }
        MatY(i, 0) = y[i];
    }

    Debug(MatX);
    Debug(MatY);
    Debug(VecY);
    // auto coeffic = MatX.householderQr().solve(VecY);

    auto coeffic =
        (MatX.transpose() * MatX).inverse() * MatX.transpose() * MatY;

    Debug(coeffic);
    std::vector<double> ry = mp::transform(x, [&](auto x) {
        double y = 0;
        for (int i = 0; i < order; i++) {
            y += coeffic(i, 0) * std::pow(x, i);
        }
        return y;
    });
    Debug_Vec(ry);

    mp::scatter(x, y);
    mp::hold(mp::on);
    mp::plot(x, ry);
    mp::show();
}

/// 平面拟合 y = ax + by + c
void least_square_plane()
{
    std::vector<double> x = mp::linspace(0, 10, 10);
    Debug_Vec(x);
    auto y =
        mp::transform(x, [&](double x) { return cos(x) + mp::rand(0, 1); });
    Debug_Vec(y);

    auto z =
        mp::transform(y, [&](double y) { return sin(y) + mp::rand(0, 1); });

    Eigen::MatrixXd MatX;
    Eigen::MatrixXd MatY;

    MatX.resize(x.size(), 3);
    MatY.resize(x.size(), 1);

    for (int i = 0; i < x.size(); i++) {
        MatX.row(i) = Eigen::RowVector3d(x[i], y[i], 1);
        MatY(i, 0) = z[i];
    }

    auto coeffic =
        (MatX.transpose() * MatX).inverse() * MatX.transpose() * MatY;

    Debug(coeffic);

    auto [mx, my] = mp::meshgrid(x, y);

    auto mz = mp::transform(mx, my, [&](double x, double y) {
        return coeffic(0, 0) * x + coeffic(1, 0) * y + coeffic(2, 0);
    });

    mp::scatter3(x, y, z);
    mp::hold(mp::on);
    mp::mesh(mx, my, mz);
    mp::show();
}

auto to_std_vector(Eigen::VectorXd vec)
{
    std::vector<double> vec_std(vec.data(), vec.data() + vec.size());
    return vec_std;
}

void algo_visual()
{
    // auto vec = rande(10, 10);
    // auto x = to_std_vector(vec);
    // std::vector<double> x = mp::randn(5000, 5, 2);

    Run(0, least_square_line);
    Run(1, least_square_curve, 10);
    Run(0, least_square_plane);
}
