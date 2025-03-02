constexpr int WIDTH = 1000;
constexpr int HEIGHT = 1000;

// Scalable dimensions
int windowWidth = WIDTH;
int windowHeight = HEIGHT;

constexpr int cParticles = 2000;
constexpr int cBlackHole = 500;
int cTailSize = 10;
constexpr double maxQ = 1;
constexpr double maxRadius = 2;
constexpr double minDist = 1e-6;
constexpr double minQ = 1e-6;
int numThreads = 1;
const double radiusC = (exp(maxRadius) - 1) / maxQ;

double totalPotentialEnergy = 0.0;
double totalKineticEnergy = 0.0;
int inactiveCount = 0;
int frameCount = 0;
int frameCountPerTrace = 10;

// Random generator
double rnd() { return rand() / 32767.0; }

// Particle structure
struct Particle {
    Vec3d position{};
    Vec3d velocity{};
    Vec3d force{};

    double realR{};
    double showR{};

    bool active = true;
    deque<Vec3d> trace;

    void addTrace() {
        if ((frameCount % frameCountPerTrace) != 0)
            return;

        trace.push_front(position);
        if (trace.size() > cTailSize) {
            trace.pop_back();
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
    friend bool operator<(const Particle &lhs, const Particle &rhs);
    double _q{};
};

bool operator<(const Particle &lhs, const Particle &rhs) {
    return lhs._q > rhs._q;
}

// Particle container
alignas(64) Particle particles[cParticles];
alignas(64) atomic<int> locks[cParticles];

struct Locker {
    atomic<int> &lock;

    explicit Locker(atomic<int> &l) : lock(l) {
        int expected = 0;
        while (!lock.compare_exchange_strong(expected, 1)) {
            expected = 0;
            this_thread::yield();
        }
    }

    ~Locker() { lock.store(0); }
};

struct SystemParams {
    Vec3d position{0, 0, 0};
    Vec3d impuls{0, 0, 0};
    Vec3d momentum{0, 0, 0};

    Matx33d inertialTensor = Matx33d::zeros();
    double q{0};
};

const Matx33d E = Matx33d::eye();
SystemParams init;
int observerIndex = -1;
Vec3d observer{0, 0, 0};
