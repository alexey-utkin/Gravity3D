constexpr int WIDTH = 1000;
constexpr int HEIGHT = 1000;

// Scalable dimensions
int windowWidth = WIDTH;
int windowHeight = HEIGHT;

constexpr int cParticles = 2000;
constexpr int cBlackHole = 500;
constexpr int cTailSize = 10;
constexpr double maxQ = 1;
constexpr double maxRadius = 2;
constexpr double minDist = 1e-6;
constexpr double minQ = 1e-6;
int numThreads = 1;
const double radiusC = (exp(maxRadius) - 1) / maxQ;

double totalPotentialEnergy = 0.0;
double totalKineticEnergy = 0.0;
int inactiveCount = 0;

// Random generator
double rnd() { return rand() / 32767.0; }

// Particle structure
struct Particle {
    atomic<int> lock{0};
    Vec3d position{};
    Vec3d velocity{};
    Vec3d force{};

    double realR{};
    double showR{};

    bool active = true;
    deque<Vec3d> trace;

    void addTrace() {
        trace.push_back(position);
        if (trace.size() > cTailSize) {
            trace.pop_front();
        }
    }

    void shiftTrace(const Vec3d &shift) {
        for (auto &t : trace) {
            t -= shift;
        }
    }

    void setQ(double q) {
        if (q < minQ) {
            q = minQ;
        }
        _q = q;
        showR = log(1 + radiusC * _q);
        realR = max(showR / 2, minDist / 2);
    }

    [[nodiscard]] double q() const { return _q; }

protected:
    double _q{};
};

// Particle container
alignas(64) Particle particles[cParticles];

struct Locker {
    Particle &p;

    explicit Locker(Particle &p1) : p(p1) {
        int expected = 0;
        while (!p.lock.compare_exchange_strong(expected, 1)) {
            expected = 0;
            this_thread::yield();
        }
    }

    ~Locker() { p.lock.store(0); }
};

struct SystemParams {
    Vec3d position{0, 0, 0};
    Vec3d impuls{0, 0, 0};
    Vec3d momentum{0, 0, 0};

    Matx33d inertialTensor = Matx33d::eye();
    double q{0};
};

const Matx33d E = Matx33d::eye();
SystemParams init;