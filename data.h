#ifndef GRAVITY3D_DATA_H
#define GRAVITY3D_DATA_H

#include <atomic>
#include <deque>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Constants
constexpr int WIDTH = 1000;
constexpr int HEIGHT = 1000;
constexpr double maxQ = 1;
constexpr double maxRadius = 2;
constexpr double minDist = 1e-6;
constexpr double minQ = 1e-6;

// Utility functions
double rnd();

// Forward declaration
struct Simulation;

// Particle structure
struct Particle {
    Vec3d position{};
    Vec3d velocity{};
    Vec3d force{};

    double realR{};
    double showR{};

    bool active = true;
    deque<Vec3d> trace;

    void addTrace(const Simulation& sim);
    void shiftTrace(const Vec3d &shift);
    void setQ(double q);
    [[nodiscard]] double q() const;

protected:
    friend bool operator<(const Particle &lhs, const Particle &rhs) {
        return lhs._q > rhs._q;
    }
    double _q{};
};

// System parameters
struct SystemParams {
    Vec3d position{0, 0, 0};
    Vec3d impuls{0, 0, 0};
    Vec3d momentum{0, 0, 0};

    Matx33d inertialTensor = Matx33d::zeros();
    double q{0};
};

#endif // GRAVITY3D_DATA_H