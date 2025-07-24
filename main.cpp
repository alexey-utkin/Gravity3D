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


// teams distribute parallel for simd
int main() {
    // Global simulation instance
    Simulation sim;

    int numThreads = max(omp_get_max_threads() - 1, 1);
    // numThreads = 1;
    sim.setNumThreads(numThreads);
    omp_set_num_threads(numThreads);
    cout << "Running with " << numThreads << " threads." << endl;

    srand(time(nullptr));
    // initParticles:
    sim.initParticles_3Centers();
    sim.sortParticles();

    //sim.initParticles_WithOldBlackHoles();

    namedWindow(windowName, WINDOW_NORMAL);
    setMouseCallback(windowName, onMouse, nullptr);

    sim.getInitParams() = sim.calcParams();
    sim.recenterAndZeroV(false);
    while (inputProcessing(sim) && getWindowProperty(windowName, WND_PROP_VISIBLE) >= 1) {
        sim.incrementFrameCount();
        Rect windowRect = getWindowImageRect(windowName);
        windowWidth = windowRect.width;
        windowHeight = windowRect.height;
        Mat canvas = Mat::zeros(windowHeight, windowWidth, CV_8UC3);

        auto start = chrono::high_resolution_clock::now();
#pragma omp parallel
        {
            sim.updateParticles();
            renderScene(canvas, sim);
        }
        SystemParams current = sim.calcParams();
        sim.renormalize();

        auto duration = chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - start);

        std::cout << sim.getInactiveCount() << "P: " << (sim.getInitParams().impuls - current.impuls) 
                 << " E: " << (sim.getTotalKineticEnergy() + sim.getTotalPotentialEnergy()) 
                 << ", " << sim.getTotalKineticEnergy() << ", " << sim.getTotalPotentialEnergy() << std::endl;
        imshow(windowName, canvas);
    }
    destroyAllWindows();
    return 0;
}
