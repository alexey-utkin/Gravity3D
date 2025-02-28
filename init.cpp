[[maybe_unused]] void initParticles_WithOldBlackHoles() {
    Particle S;

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

[[maybe_unused]] void initParticles_3Centers() {
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
