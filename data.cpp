#include "data.h"

// Scalable dimensions
int windowWidth = WIDTH;
int windowHeight = HEIGHT;

int cTailSize = 10;
int numThreads = 1;
const double radiusC = (exp(maxRadius) - 1) / maxQ;

double totalPotentialEnergy = 0.0;
double totalKineticEnergy = 0.0;
int inactiveCount = 0;
int frameCount = 0;
int frameCountPerTrace = 10;

// Random generator
double rnd() { return rand() / 32767.0; }

// Particle methods implementation
void Particle::addTrace() {
    if ((frameCount % frameCountPerTrace) != 0)
        return;

    trace.push_front(position);
    if (trace.size() > cTailSize) {
        trace.pop_back();
    }
}

void Particle::shiftTrace(const Vec3d &shift) {
    for (auto &t : trace) {
        t -= shift;
    }
}

void Particle::setQ(double q) {
    if (q < minQ) {
        q = minQ;
    }
    _q = q;
    showR = log(1 + radiusC * _q);
    realR = max(showR / 2, minDist / 2);
}

double Particle::q() const { return _q; }

bool operator<(const Particle &lhs, const Particle &rhs) {
    return lhs._q > rhs._q;
}

// Particle container
alignas(64) Particle particles[cParticles];
alignas(64) atomic<int> locks[cParticles];

// Locker implementation
Locker::Locker(atomic<int> &l) : lock(l) {
    int expected = 0;
    while (!lock.compare_exchange_strong(expected, 1)) {
        expected = 0;
        this_thread::yield();
    }
}

Locker::~Locker() { lock.store(0); }

const Matx33d E = Matx33d::eye();
SystemParams init;
int observerIndex = -1;
Vec3d observer{0, 0, 0};

//using json = nlohmann::json;

// Serialization for Vec3d (if not implemented already)
//
//namespace nlohmann {
//
//static auto to_json(const Vec3d &vec) noexcept -> json {
//    return json{vec[0], vec[1], vec[2]};
//}
//
//void from_json(const json &j, Vec3d &vec) {
//            vec = Vec3d(j[0], j[1], j[2]);
//}
//
//json w(const Matx33d &mat) {
//    return json{mat(0, 0), mat(0, 1), mat(0, 2),
//             mat(1, 0), mat(1, 1), mat(1, 2),
//             mat(2, 0), mat(2, 1), mat(2, 2)};
//}
//
//json vecToJson(const cv::Vec3d &vec) {
//    return json{vec[0], vec[1], vec[2]};
//}
//
//void to_json(nlohmann::json &j, const SystemParams &params) {
//    j = {
//        {"position", params.position},  // Uses overloaded to_json for cv::Vec3d
//        {"impuls", params.impuls},      // Uses overloaded to_json for cv::Vec3d
//        {"momentum", params.momentum},  // Uses overloaded to_json for cv::Vec3d
////        {"inertialTensor", w(params.inertialTensor)},
//        {"q", params.q}
//    };
//}
//
//
//
