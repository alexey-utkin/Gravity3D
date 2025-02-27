inline void normalize() {
    // Compute the center of mass based on active particles
    vector<SystemParams> centerOfMasses(numThreads, SystemParams{});
#pragma omp for schedule(static)
    for (auto i = 0; i < cParticles; ++i) {
        Particle &p = particles[i];
        if (!p.active)
            continue; // Skip inactive particles
        SystemParams &tl_cm = centerOfMasses[omp_get_thread_num()];
        double q = p.q();
        Vec3d r = p.position;
        Vec3d v = p.velocity;

        tl_cm.position += q * r;
        tl_cm.impuls += q * v;
        tl_cm.momentum += q * r.cross(v);

        const double r2 = r.dot(r);
        tl_cm.inertiaTensor += q * (r2 * E - r * r.t());
        tl_cm.q += q;
    }

#pragma omp barrier
    SystemParams cm{};
    Vec3d A{};
    Vec3d dV{};
#pragma omp single
    {
        for (const auto &threadCM : centerOfMasses) {
            cm.position += threadCM.position;
            cm.impuls += threadCM.impuls;
            cm.momentum += threadCM.momentum;
            cm.inertiaTensor += threadCM.inertiaTensor;
            cm.q += threadCM.q;
        }

        if (cm.q <= 0) {
            cm.q = minQ;
        }

        fullMass = cm.q;
        fullCenter = cm.position / fullMass;
        if (!useFullMomentum) {
            A = {0, 0, 0};
            dV = -cm.impuls / fullMass;
            useFullMomentum = true;
            // for (auto i = 0; i < cParticles; ++i) {
            //     Particle &p = particles[i];
            //     if (!p.active)
            //         continue;              // Skip inactive particles
            //     p.position -= fullCenter; // Shift position
            //     p.shiftTrace(fullCenter);
            //     p.velocity += dV;
            // }
        } else {
            Matx33d I_inv;
            invert(cm.inertiaTensor, I_inv, DECOMP_SVD); // Compute inverse of inertia tensor
            A = I_inv * (cm.momentum - fullMomentum);    // Solve for correction factor
            dV = -(cm.impuls - fullImpuls) / fullMass;
        }
        fullMomentum = cm.momentum;
        fullImpuls = cm.impuls;
    }

// Apply offset to move the center of mass to the origin
#pragma omp barrier
#pragma omp for schedule(static)
    for (auto i = 0; i < cParticles; ++i) {
        Particle &p = particles[i];
        if (!p.active)
            continue; // Skip inactive particles
        // p.velocity += dV;
        p.velocity -= A.cross(p.position);
        // p.position -= fullCenter; // Shift position
        // p.shiftTrace(fullCenter);
    }

// control
#pragma omp barrier
#pragma omp single
    {
        cm = SystemParams{0};
        for (auto i = 0; i < cParticles; ++i) {
            Particle &p = particles[i];
            if (!p.active)
                continue; // Skip inactive particles
            cm.impuls += p.q() * p.velocity;
        }
        std::cout << "P: " << fullImpuls << " PN: " << (fullImpuls - cm.impuls) << std::endl;
        dV = -(cm.impuls - fullImpuls) / fullMass;
        for (auto i = 0; i < cParticles; ++i) {
            Particle &p = particles[i];
            if (!p.active)
                continue; // Skip inactive particles
            // p.velocity += dV;
        }
    }
#pragma omp barrier
}