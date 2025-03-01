SystemParams calcParams() {
    // Compute the center of mass based on active particles
#pragma omp barrier
    vector<SystemParams> centerOfMasses(numThreads, SystemParams{});
#pragma omp parallel
    {
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
            tl_cm.inertialTensor += q * (r2 * E - r * r.t());
            tl_cm.q += q;
        }
    }
#pragma omp barrier
    SystemParams cm{};
#pragma omp single
    {
        for (const auto &threadCM : centerOfMasses) {
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

inline SystemParams renormalize(SystemParams &init, SystemParams &current) {
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
    return init;
}

inline void recenterAndZeroV(bool forObserver) {
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