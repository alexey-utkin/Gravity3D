#ifndef GRAVITY3D_SIMULATION_H
#define GRAVITY3D_SIMULATION_H

#include "data.h"
#include <atomic>
#include <omp.h>

class Simulation {
public:
    Simulation();
    ~Simulation() = default;

    // Initialization methods
    void initParticles_WithOldBlackHoles();
    void initParticles_3Centers();
    void sortParticles(); // Sort particles by mass (q)

    // Simulation methods
    void updateParticles();
    double processInteraction(int i, int j);
    void processPairInteractons();

    // Normalization methods
    SystemParams calcParams();
    SystemParams renormalize(SystemParams &init, SystemParams &current);
    void recenterAndZeroV(bool forObserver);

    // Accessors
    int getNumThreads() const { return numThreads; }
    void setNumThreads(int threads) { numThreads = threads; }
    
    double getTotalPotentialEnergy() const { return totalPotentialEnergy; }
    double getTotalKineticEnergy() const { return totalKineticEnergy; }
    int getInactiveCount() const { return inactiveCount; }
    int getFrameCount() const { return frameCount; }
    void incrementFrameCount() { ++frameCount; }
    
    int getFrameCountPerTrace() const { return frameCountPerTrace; }
    void setFrameCountPerTrace(int count) { frameCountPerTrace = count; }
    
    int getTailSize() const { return cTailSize; }
    void setTailSize(int size) { cTailSize = size; }
    
    SystemParams& getInitParams() { return init; }
    
    int getObserverIndex() const { return observerIndex; }
    void setObserverIndex(int index) { observerIndex = index; }
    
    Vec3d& getObserver() { return observer; }
    
    Particle* getParticles() { return particles; }
    atomic<int>* getLocks() { return locks; }

private:
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