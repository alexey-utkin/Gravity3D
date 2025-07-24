#ifndef GRAVITY3D_SIMULATION_H
#define GRAVITY3D_SIMULATION_H

#include "data.h"
#include <atomic>

struct Simulation {
    static constexpr int cParticles = 2000;
    static constexpr int cBlackHole = 500;

    Simulation();
    ~Simulation() = default;

    void initParams() {  init = calcParams(); }

    // Initialization methods
    void initParticles_WithOldBlackHoles();
    void initParticles_3Centers();
    void sortParticles(); // Sort particles by mass (q)

    // Simulation methods
    void updateParticles();
    double processInteraction(int i, int j);
    void processPairInteractions();

    // Normalization methods
    SystemParams calcParams();
    void renormalize();
    void recenterAndZeroV(bool forObserver);

    Vec3d& getObserver() {
        if (observerIndex >= 0) {
            observer = particles[observerIndex].position;
        }
        return observer;
    }

    bool inputProcessing();

    // Simulation state
    int numThreads;
    int cTailSize;
    double totalPotentialEnergy;
    double totalKineticEnergy;
    int inactiveCount;
    int frameCount;
    int frameCountPerTrace;
    SystemParams init;
    int observerIndex;
    Vec3d observer;
    
    // Particle container
    alignas(64) Particle particles[cParticles];
    alignas(64) atomic<int> locks[cParticles];
};

#endif // GRAVITY3D_SIMULATION_H