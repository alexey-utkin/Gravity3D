#include "simulation.h"
#include <algorithm>
#include <cmath>
#include <omp_llvm.h>
#include <thread>

Simulation::Simulation()
    : numThreads(1)
    , cTailSize(10)
    , totalPotentialEnergy(0.0)
    , totalKineticEnergy(0.0)
    , inactiveCount(0)
    , frameCount(0)
    , frameCountPerTrace(10)
    , observerIndex(-1)
    , observer(0, 0, 0)
{
    // Initialize locks
    for (auto &lock : locks) {
        lock.store(0);
    }
}

// Initialization methods
void Simulation::initParticles_WithOldBlackHoles() {
    int i = 0;
    for (auto &p : particles) {
        if (++i < cBlackHole)
            p.setQ(maxQ);
        else
            p.setQ(maxQ * exp(rnd() - 1.0));
        p.position = {WIDTH / 4 + (WIDTH * rnd() - WIDTH / 2) * 0.5,
                      HEIGHT / 4 + (HEIGHT * rnd() - HEIGHT / 2) * 0.5,
                      HEIGHT / 4 + (HEIGHT * rnd() - HEIGHT / 2) * 0.5};
        p.velocity = {rnd(), rnd(), rnd()};
    }
}

void Simulation::initParticles_3Centers() {
    double offsets[][3] = {{0, 0, 0},
                           {WIDTH / 2, HEIGHT / 2, HEIGHT / 2},
                           {0, HEIGHT / 2, HEIGHT / 2}};

    int i = 0;
    for (auto &p : particles) {
        ++i;
        p.setQ(maxQ * rnd());
        int z = i % 3;
        const double *offset = offsets[z];
        p.position = {offset[0] + (WIDTH * rnd() - WIDTH / 2) * 0.5,
                      offset[1] + (HEIGHT * rnd() - HEIGHT / 2) * 0.5,
                      offset[2] + (HEIGHT * rnd() - HEIGHT / 2) * 0.5};
        p.velocity = {rnd(), rnd(), rnd()};
    }
}

void Simulation::sortParticles() {
    std::sort(particles, particles + cParticles);
}

// Simulation methods
double Simulation::processInteraction(int i, int j) {
    Locker lockI(locks[i]);
    Locker lockJ(locks[j]);

    Particle &pi = particles[i];
    Particle &pj = particles[j];
    if (!pi.active || !pj.active)
        return 0.0;

    Vec3d delta = pj.position - pi.position;
    double distSq = delta.dot(delta);
    if (distSq < minDist) {
        distSq = minDist;
    }
    double dist = sqrt(distSq);
    Vec3d normal = delta / dist;

    double potentialEnergyChange = 0.0;

    if (dist < (pi.realR + pj.realR)) {
        double v1n = pi.velocity.dot(normal);
        double v2n = pj.velocity.dot(normal);
        double totalMass = pi.q() + pj.q();
        if (rnd() < 0.05) {
            // Elastic collision
            double m1 = pi.q();
            double m2 = pj.q();
            double v1nNew = (v1n * (m1 - m2) + 2 * m2 * v2n) / totalMass;
            double v2nNew = (v2n * (m2 - m1) + 2 * m1 * v1n) / totalMass;

            pi.velocity += normal * (v1nNew - v1n);
            pj.velocity += normal * (v2nNew - v2n);
        } else {
            // Inelastic collision
            pi.velocity = (pi.velocity * pi.q() + pj.velocity * pj.q()) / totalMass;
            pi.position = (pi.position * pi.q() + pj.position * pj.q()) / totalMass;
            pi.setQ(totalMass);
            pj.active = false; // Mark the particle as inactive
        }
    } else {
        double qq = pj.q() * pi.q();
        potentialEnergyChange = -qq / dist;

        // Calculate forces
        double F = qq / distSq;
        const Vec<double, 3> &vecF = normal * F;
        pi.force += vecF;
        pj.force -= vecF;
    }
    
    return potentialEnergyChange;
}

void Simulation::processPairInteractons() {
    int intConst = cParticles * (cParticles - 1);
    // Update forces
    int totalInteractions = intConst / 2;
#pragma omp for schedule(static) // alt: Dynamic scheduling with chunk size 64
    for (auto index = 0; index < totalInteractions; ++index) {
        // Compute (i, j) indices from linear index
        int i = cParticles - 2 -
                (int)(std::sqrt(-8 * index + 4 * intConst - 7) / 2.0 - 0.5);
        int j = index + i + 1 - intConst / 2 +
                ((cParticles - i) * ((cParticles - i) - 1)) / 2;

        // Get potential energy change from interaction
        double energyChange = processInteraction(i, j);

#pragma omp atomic
        totalPotentialEnergy += energyChange;
    }
}

void Simulation::updateParticles() {
#pragma omp barrier
    totalPotentialEnergy = 0.0;
    totalKineticEnergy = 0.0;
    inactiveCount = 0;

    // Update forces
    processPairInteractons();

#pragma omp for schedule(static)
    // Update system params and drop force
    for (auto i = 0; i < cParticles; ++i) {
        Particle &p = particles[i];
        if (!p.active) {
#pragma omp atomic
            inactiveCount += 1;
            continue;
        }
        p.addTrace(*this);
        auto a = p.force / p.q();
        p.position += p.velocity + a * 0.5;
        p.velocity += a;
        p.force = {0, 0, 0};
#pragma omp atomic
        totalKineticEnergy += p.velocity.dot(p.velocity) * p.q() * 0.5;
    }
}

// Normalization methods
SystemParams Simulation::calcParams() {
    // Compute the center of mass based on active particles
#pragma omp barrier
    vector<SystemParams> centerOfMassesInThreadFraction(numThreads, SystemParams{});
#pragma omp parallel
    {
#pragma omp for schedule(static)
        for (auto i = 0; i < cParticles; ++i) {
            Particle &p = particles[i];
            if (!p.active)
                continue; // Skip inactive particles
            SystemParams &tl_cm = centerOfMassesInThreadFraction[omp_get_thread_num()];
            double q = p.q();
            Vec3d r = p.position;
            Vec3d v = p.velocity;

            tl_cm.position += q * r;
            tl_cm.impuls += q * v;
            tl_cm.momentum += q * r.cross(v);

            const double r2 = r.dot(r);
            tl_cm.inertialTensor += q * (r2 * E - r * r.t());
            tl_cm.q += q;
        }
    }
#pragma omp barrier
    SystemParams cm{};
#pragma omp single
    {
        for (const auto &threadCM : centerOfMassesInThreadFraction) {
            cm.position += threadCM.position;
            cm.impuls += threadCM.impuls;
            cm.momentum += threadCM.momentum;
            cm.inertialTensor += threadCM.inertialTensor;
            cm.q += threadCM.q;
        }

        if (cm.q <= 0) {
            cm.q = minQ;
        }

        cm.position /= cm.q;
    }
    return cm;
}

void Simulation::renormalize() {
    SystemParams &init = getInitParams();
    const SystemParams &current = calcParams();

#pragma omp barrier
    Matx33d I_inv;
    invert(current.inertialTensor, I_inv, DECOMP_SVD); // Compute inverse of inertia tensor
    Vec3d A = I_inv * (current.momentum - init.momentum);
    Vec3d dV = -(current.impuls - init.impuls) / init.q;
#pragma omp parallel
    {
#pragma omp for schedule(static)
        for (auto i = 0; i < cParticles; ++i) {
            Particle &p = particles[i];
            if (!p.active)
                continue; // Skip inactive particles
            p.velocity += dV - A.cross(p.position);
        }
    }
}

void Simulation::recenterAndZeroV(bool forObserver) {
#pragma omp barrier
    forObserver &= (observerIndex >= 0 && particles[observerIndex].active);
    Vec3d pos = forObserver
                    ? particles[observerIndex].position
                    : init.position;
    Vec3d V = forObserver
                  ? particles[observerIndex].velocity
                  : init.impuls / init.q;
#pragma omp parallel
    {
#pragma omp for schedule(static)
        for (auto i = 0; i < cParticles; ++i) {
            Particle &p = particles[i];
            if (!p.active)
                continue; // Skip inactive particles
            p.position -= pos;
            p.shiftTrace(pos);
            p.velocity -= V;
        }
    }
    init = calcParams();
}