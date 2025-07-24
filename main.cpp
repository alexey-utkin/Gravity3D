/**
@author: Alexey Utkin
@email:  alexey.utkin@gmail.com

    My God, it's full of stars!
                      Arthur C.
*/

#include <iostream>
#include <omp.h>
#include <opencv2/opencv.hpp>

#include "data.h"
#include "simulation.h"
#include "viewData.h"
#include "render.h"

using namespace cv;
using namespace std;

constexpr char windowName[] = "Simulation";

// teams distribute parallel for simd
int main() {
    // Global simulation instance
    Simulation sim;

    sim.numThreads = max(omp_get_max_threads() - 1, 1);
    // sim.numThreads = 1;
    omp_set_num_threads(sim.numThreads);
    cout << "Running with " << sim.numThreads << " threads." << endl;

    srand(time(nullptr));
    // initParticles:
    //sim.initParticles_3Centers();
    sim.initParticles_WithOldBlackHoles();
    sim.sortParticles();

    namedWindow(windowName, WINDOW_NORMAL);
    setMouseCallback(windowName, Camera::onMouse, &sim.camera);

    sim.initParams();
    sim.recenterAndZeroV(false);
    while (sim.inputProcessing() && getWindowProperty(windowName, WND_PROP_VISIBLE) >= 1) {
        ++sim.frameCount;
        Rect windowRect = getWindowImageRect(windowName);
        Mat canvas = Mat::zeros(windowRect.height, windowRect.width, CV_8UC3);

        auto start = chrono::high_resolution_clock::now();
#pragma omp parallel
        {
            sim.updateParticles();
            renderScene(canvas, sim);
        }
        SystemParams current = sim.calcParams();
        sim.renormalize();

        auto duration = chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - start);

        std::cout << sim.inactiveCount << "P: " << (sim.init.impuls - current.impuls)
                 << " E: " << (sim.totalKineticEnergy + sim.totalPotentialEnergy)
                 << ", " << sim.totalKineticEnergy << ", " << sim.totalPotentialEnergy << std::endl;
        imshow(windowName, canvas);
    }
    destroyAllWindows();
    return 0;
}
