inline void processInteraction(Particle &pi, Particle &pj) {
    Locker lockI(pi);
    Locker lockJ(pj);
    if (!pi.active || !pj.active)
        return;

    Vec3d delta = pj.position - pi.position;
    double distSq = delta.dot(delta);
    if (distSq < minDist) {
        distSq = minDist;
    }
    double dist = sqrt(distSq);
    Vec3d normal = delta / dist;

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
        } else { // Inelastic collision
            pi.velocity = (pi.velocity * pi.q() + pj.velocity * pj.q()) / totalMass;
            pi.position = (pi.position * pi.q() + pj.position * pj.q()) / totalMass;
            pi.setQ(totalMass);
            pj.active = false; // Mark the particle as inactive
        }
    } else {
        double qq = pj.q() * pi.q();
        totalPotentialEnergy += -qq / dist;

        // Calculate forces
        double F = qq / distSq;
        const Vec<double, 3> &vecF = normal * F;
        pi.force += vecF;
        pj.force -= vecF;
    }
}

inline void processPairInteractons() {
    // Update forces
    int totalInteractions = (cParticles * (cParticles - 1)) / 2;
#pragma omp barrier
#pragma omp for reduction(+ : totalPotentialEnergy) schedule(static) // alt: Dynamic scheduling with chunk size 64
    for (auto index = 0; index < totalInteractions; ++index) {
        // Compute (i, j) indices from linear index
        int i = cParticles - 2 -
                (int)(std::sqrt(-8 * index + 4 * cParticles * (cParticles - 1) - 7) / 2.0 - 0.5);
        int j = index + i + 1 - (cParticles * (cParticles - 1)) / 2 +
                ((cParticles - i) * ((cParticles - i) - 1)) / 2;
        processInteraction(particles[i], particles[j]);
    }
}

inline void updateParticles() {
#pragma omp barrier
    totalPotentialEnergy = 0.0;
    totalKineticEnergy = 0.0;
    inactiveCount = 0;

    // Update forces
    processPairInteractons();
#pragma omp for reduction(+ : totalPotentialEnergy, inactiveCount) schedule(static)
    // Update system params and drop force
    for (auto i = 0; i < cParticles; ++i) {
        Particle &p = particles[i];
        if (!p.active) {
            inactiveCount += 1;
            continue;
        }
        p.addTrace();
        p.position += p.velocity;
        p.velocity += p.force / p.q();
        p.force = {0, 0, 0};
        totalKineticEnergy += p.velocity.dot(p.velocity) * p.q() * 0.5;
    }
}