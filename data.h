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
constexpr int cParticles = 2000;
constexpr int cBlackHole = 500;
constexpr double maxQ = 1;
constexpr double maxRadius = 2;
constexpr double minDist = 1e-6;
constexpr double minQ = 1e-6;

// Particle structure
struct Particle {
    Vec3d position{};
    Vec3d velocity{};
    Vec3d force{};

    double realR{};
    double showR{};

    bool active = true;
    deque<Vec3d> trace;

    void addTrace();
    void shiftTrace(const Vec3d &shift);
    void setQ(double q);
    [[nodiscard]] double q() const;

protected:
    friend bool operator<(const Particle &lhs, const Particle &rhs);
    double _q{};
};

bool operator<(const Particle &lhs, const Particle &rhs);

// Thread synchronization
struct Locker {
    atomic<int> &lock;
    explicit Locker(atomic<int> &l);
    ~Locker();
};

// System parameters
struct SystemParams {
    Vec3d position{0, 0, 0};
    Vec3d impuls{0, 0, 0};
    Vec3d momentum{0, 0, 0};

    Matx33d inertialTensor = Matx33d::zeros();
    double q{0};
};

// Global variables (to be encapsulated later)
extern int windowWidth;
extern int windowHeight;
extern int cTailSize;
extern int numThreads;
extern const double radiusC;

extern double totalPotentialEnergy;
extern double totalKineticEnergy;
extern int inactiveCount;
extern int frameCount;
extern int frameCountPerTrace;

extern const Matx33d E;
extern SystemParams init;
extern int observerIndex;
extern Vec3d observer;

extern Particle particles[cParticles];
extern atomic<int> locks[cParticles];

// Utility functions
double rnd();

#endif // GRAVITY3D_DATA_H