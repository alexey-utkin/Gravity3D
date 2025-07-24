#include "data.h"
#include "simulation.h"

// Scalable dimensions
const double radiusC = (exp(maxRadius) - 1) / maxQ;

// Random generator
double rnd() { return rand() / 32767.0; }

// Particle methods implementation
void Particle::addTrace(const Simulation& sim) {
    if ((sim.getFrameCount() % sim.getFrameCountPerTrace()) != 0)
        return;

    trace.push_front(position);
    if (trace.size() > sim.getTailSize()) {
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

// Locker implementation
Locker::Locker(atomic<int> &l) : lock(l) {
    int expected = 0;
    while (!lock.compare_exchange_strong(expected, 1)) {
        expected = 0;
        this_thread::yield();
    }
}

Locker::~Locker() { lock.store(0); }



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
