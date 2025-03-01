/**
@author: Alexey Utkin
@email:  alexey.utkin@gmail.com

    My God, it's full of stars!
                      Arthur C.
*/

#include <atomic>
#include <deque>
#include <iostream>
#include <omp.h>
#include <opencv2/opencv.hpp>
#include <thread>

using namespace cv;
using namespace std;

// @format: off
#include "data.cpp"
#include "init.cpp"
#include "interaction.cpp"
#include "normalize.cpp"
#include "viewData.cpp"
#include "render.cpp"
// @format: on

int main() {
    numThreads = max(omp_get_max_threads() - 1, 1);
    // numThreads = 1;
    omp_set_num_threads(numThreads);
    cout << "Running with " << numThreads << " threads." << endl;

    srand(time(nullptr));
    // initParticles:
    initParticles_3Centers();
    std::sort(std::begin(particles), std::end(particles));

    //initParticles_WithOldBlackHoles();

    namedWindow(windowName, WINDOW_NORMAL);
    setMouseCallback(windowName, onMouse, nullptr);

    init = calcParams();
    recenterAndZeroV();
    while (inputProcessing() && getWindowProperty(windowName, WND_PROP_VISIBLE) >= 1) {
        ++frameCount;
        Rect windowRect = getWindowImageRect(windowName);
        windowWidth = windowRect.width;
        windowHeight = windowRect.height;
        Mat canvas = Mat::zeros(windowHeight, windowWidth, CV_8UC3);

        auto start = chrono::high_resolution_clock::now();
#pragma omp parallel
        {
            updateParticles();
            renderScene(canvas);
        }
        SystemParams current = calcParams();
        renormalize(init, current);

        auto duration = chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - start);
        // std::cout << "Execution time: " << duration.count() << " ms" << std::endl;

        std::cout << inactiveCount << "P: " << (init.impuls - current.impuls) << " E: " << (totalKineticEnergy + totalPotentialEnergy) << ", " << totalKineticEnergy << ", " << totalPotentialEnergy << std::endl;
        imshow(windowName, canvas);
    }
    destroyAllWindows();
    return 0;
}
