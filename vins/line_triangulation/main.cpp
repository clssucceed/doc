#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <ctime>
#include <cmath>

Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    // std::cout << v.transpose() << std::endl;
    // std::cout << m << std::endl;
    return m;
}

int main(int, char**) {
    std::srand(std::time(nullptr));
    // step 1: 随机生成n,d
    Eigen::Vector3d x = Eigen::Vector3d::Random();
    Eigen::Vector3d y = Eigen::Vector3d::Random();
    Eigen::Vector3d z = SkewMatrix(x)*y;  
    y = SkewMatrix(z) * x;
    Eigen::Vector3d n = x.normalized() * (std::rand() % 100);
    Eigen::Vector3d d = y.normalized();
    // std::cout << n.transpose() << std::endl;
    // std::cout << d.transpose() << std::endl;
    // std::cout << n.transpose() * d << std::endl;
    // std::cout << n.norm() / d.norm() << std::endl;
    assert(std::fabs(n.transpose() * d) < 1.0e-6);

    // step 2: 随机生成Ri, ti和si, ei 
    constexpr int kObvNum = 3;
    std::array<Eigen::Matrix3d, kObvNum> Rs;
    std::array<Eigen::Vector3d, kObvNum> ts;
    std::array<std::pair<Eigen::Vector3d, Eigen::Vector3d>, kObvNum> ses;
    for (int i = 0; i < kObvNum; ++i) {
        std::cout << "##############################" << i << std::endl;
        Eigen::Matrix3d Ri = Eigen::Quaterniond::UnitRandom().toRotationMatrix();
        Eigen::Vector3d ti = Eigen::Vector3d::Random(); 
        std::cout << Ri << std::endl;
        std::cout << ti.transpose() << std::endl;
        Eigen::Vector3d ni = Ri * n + SkewMatrix(ti) * Ri * d;
        double si_x = -641, si_y = -361;
        while (si_x < -640 || si_x > 640 || si_y < -360 || si_y > 360) {
            si_x = (std::rand() % 1280);
            si_y = -(ni(0) * si_x + ni(2)) / ni(1);
            // std::cout << si_x << "&&&&&" << si_y << std::endl;
        }
        Eigen::Vector3d si(si_x, si_y, 1);
        double si_theta = ni.dot(si) / (ni.norm() * si.norm());
        std::cout << "$$$$$$$$$$$" << si_theta << std::endl;
        assert(si_theta < 1.0e-6);
        std::cout << si.transpose() << std::endl;
        double ei_x = -641, ei_y = -361;
        while (ei_x < -640 || ei_x > 640 || ei_y < -360 || ei_y > 360 || si_x == ei_x) {
            ei_x = (std::rand() % 1280);
            ei_y = -(ni(0) * ei_x + ni(2)) / ni(1);
        }
        Eigen::Vector3d ei(ei_x, ei_y, 1);
        double ei_theta = ni.dot(ei) / (ni.norm() * ei.norm());
        std::cout << "$$$$$$$$$$$" << ei_theta << std::endl;
        assert(ei_theta < 1.0e-6);
        std::cout << ei.transpose() << std::endl;
        Rs[i] = Ri;
        ts[i] = ti;
        si = si + Eigen::Vector3d(std::rand() % 100 / 100.0, std::rand() % 100 / 100.0, 0);
        ei = ei + Eigen::Vector3d(std::rand() % 100 / 100.0, std::rand() % 100 / 100.0, 0);
        ses[i] = std::pair<Eigen::Vector3d, Eigen::Vector3d>(si, ei);
    }

    // step 3: 生成线性求解系数矩阵,每一行是pi^T[Ri, [ti]_{\times}Ri][n, d]^T = 0;
    Eigen::Matrix<double, 2 * kObvNum, 6> A;
    for (int i = 0; i < kObvNum; ++i) {
        Eigen::Matrix<double, 3, 6> temp;
        temp.leftCols(3) = Rs.at(i);
        temp.rightCols(3) = SkewMatrix(ts.at(i)) * Rs.at(i); 
        A.row(i * 2) = ses.at(i).first.transpose() * temp; 
        A.row(i * 2 + 1) = ses.at(i).second.transpose() * temp; 
    }
    std::cout << A << std::endl;
    
    // step 4; 分析A的奇异值
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::cout << "A.singular_values: " << svd.singularValues().transpose() << std::endl;
    std::cout << "svd.V: " << std::endl << svd.matrixV() << std::endl;
    std::cout << "least square solution: " << svd.solve(Eigen::Matrix<double, 2 * kObvNum, 1>::Zero()).transpose() << std::endl;
    Eigen::VectorXd L_est = svd.matrixV().rightCols(1);
    std::cout << "L_est: " << svd.matrixV().rightCols(1).normalized().transpose() << std::endl;
    Eigen::Matrix<double, 6, 1> L_gt;
    L_gt << n, d;
    std::cout << "L_gt: " << L_gt.normalized().transpose() << std::endl;
    Eigen::Vector3d n_est = L_est.head(3);
    Eigen::Vector3d d_est = L_est.tail(3);
    std::cout << std::acos(n_est.dot(d_est) / n_est.norm() / d_est.norm()) * 180 / M_PI << std::endl;
}
