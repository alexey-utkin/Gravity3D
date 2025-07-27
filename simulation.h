#ifndef GRAVITY3D_SIMULATION_H
#define GRAVITY3D_SIMULATION_H

#include "data.h"
#include <atomic>

#include "viewData.h"

struct Simulation {
    static constexpr int cParticles = 2000;
    static constexpr int cBlackHole = 500;

    Simulation();

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
    double galaxyR(Vec3d centerOfMass, double totalMass);

    // Save/Restore methods
    bool save(const std::string& filename) const;
    bool restore(const std::string& filename);

    Vec3d& getObserver() {
        if (observerIndex >= 0) {
            observer = particles[observerIndex].position;
        }
        return observer;
    }

    bool inputProcessing();

    // Simulation state
    int numThreads = 1;
    int cTailSize = 10;
    double totalPotentialEnergy;
    double totalKineticEnergy = 0.0;
    int inactiveCount = 0;
    int frameCount = 0;
    int frameCountPerTrace = -1;
    SystemParams init{};
    int observerIndex = -1;
    Vec3d observer{};
    
    // Particle container
    alignas(64) Particle particles[cParticles];
    alignas(64) atomic<int> locks[cParticles];

    // UX
    Camera camera;
    double cubeScale = 1.0; // Scale factor for the bounding cube
};

#endif // GRAVITY3D_SIMULATION_H