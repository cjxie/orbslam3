#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <gtest/gtest.h>
#include <omp.h>
#include <numeric>
#include "ORBextractor.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM3;

namespace ORB_SLAM3
{

class ORBTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Load test image
        testImage = imread("test_image.jpg", IMREAD_GRAYSCALE);
        if(testImage.empty()) {
            testImage = Mat(800, 848, CV_8UC1);
            randu(testImage, 0, 255);
        }
        
        nfeatures = 1500;
        scaleFactor = 1.2;
        nlevels = 8;
        iniThFAST = 20;
        minThFAST = 7;
        
        orbExtractor = new ORBextractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);
    }

    void TearDown() override {
        delete orbExtractor;
    }

    // Helper function to calculate average execution time with warmup
    template<typename Func>
    double measureAverageTime(Func&& func, int warmupRuns, int measureRuns) {
        vector<double> times;
        
        // Warmup runs
        for(int i = 0; i < warmupRuns; i++) {
            func();
        }
        
        // Measurement runs
        for(int i = 0; i < measureRuns; i++) {
            auto start = std::chrono::high_resolution_clock::now();
            func();
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end - start;
            times.push_back(duration.count());
        }
        
        // Calculate average excluding warmup runs
        return std::accumulate(times.begin(), times.end(), 0.0) / measureRuns;
    }

    // Previous helper functions remain the same
    void TestComputePyramid() {
        for (int level = 0; level < nlevels; ++level)
        {
            const float &scale = orbExtractor->mvInvScaleFactor[level];
            Size sz(cvRound((float)testImage.cols*scale), cvRound((float)testImage.rows*scale));
            Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
            Mat temp(wholeSize, testImage.type());
            orbExtractor->mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

            // Compute the resized image
            if( level != 0 )
            {   
                // INTER_AREA is better for image downsampling and computation expansive
                // resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_AREA);
                resize(orbExtractor->mvImagePyramid[level-1], orbExtractor->mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);

                copyMakeBorder(orbExtractor->mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                            BORDER_REFLECT_101+BORDER_ISOLATED);
            }
            else
            {
                copyMakeBorder(testImage, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                            BORDER_REFLECT_101);
            }
        }; // This works now
    }

    void TestComputePyramid_MP() {
        // orbExtractor->mvImagePyramid.clear();
        #pragma omp parallel for
        for (int level = 0; level < nlevels; ++level)
        {
            const float &scale = orbExtractor->mvInvScaleFactor[level];
            Size sz(cvRound((float)testImage.cols*scale), cvRound((float)testImage.rows*scale));
            Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
            Mat temp(wholeSize, testImage.type());
            orbExtractor->mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

            // Compute the resized image
            if( level != 0 )
            {   
                // INTER_AREA is better for image downsampling and computation expansive
                // resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_AREA);
                resize(testImage, orbExtractor->mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);
                

                copyMakeBorder(orbExtractor->mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                            BORDER_REFLECT_101+BORDER_ISOLATED);
            }
            else
            {
                copyMakeBorder(testImage, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                            BORDER_REFLECT_101);
            }
            // string windowName = "Pyramid Level " + to_string(level);
            // imshow(windowName, orbExtractor->mvImagePyramid[level]);
            // imshow("temp", temp);

            // std::cout << "Level " << level << " size: " << orbExtractor->mvImagePyramid[level].size() 
            //  << " (includes " << EDGE_THRESHOLD << "px border)" << std::endl;
     
            // cv::waitKey(0);
            // cv::destroyAllWindows();
        };
    }

    Mat testImage;
    ORBextractor* orbExtractor;
    int nfeatures;
    float scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;
    std::vector<int> threadCounts = {1, 2, 4, 8, 12};
    const int WARMUP_RUNS = 5;
    const int MEASURE_RUNS = 10;
};

// Test 1: Image Pyramid Computation
TEST_F(ORBTest, PyramidComputationPerformance) {
    cout << "\nTesting Pyramid Computation Performance" << endl;
    cout << "Performing " << WARMUP_RUNS << " warmup run(s) and " 
         << MEASURE_RUNS << " measurement runs" << endl;
    
    vector<double> times;
    
    for (auto &numThreads : threadCounts) {
        omp_set_num_threads(numThreads);
        
        // Serial version
        double serialTime = measureAverageTime([&]() {
            TestComputePyramid();
        }, WARMUP_RUNS, MEASURE_RUNS);

        // Parallel version
        double parallelTime = measureAverageTime([&]() {
            TestComputePyramid_MP();
        }, WARMUP_RUNS, MEASURE_RUNS);

        cout << "\nThread count: " << numThreads << endl;
        cout << "Average serial execution time: " << serialTime << "ms" << endl;
        cout << "Average parallel execution time: " << parallelTime << "ms" << endl;
        cout << "Speedup: " << serialTime/parallelTime << "x" << endl;
        times.push_back(parallelTime);
    }

    auto minTimeIter = min_element(times.begin(), times.end());
    int optimalThreads = threadCounts[minTimeIter - times.begin()];
    cout << "\nOptimal thread count: " << optimalThreads << endl;
}

// Test 2: FAST Feature Detection
TEST_F(ORBTest, FASTDetectionPerformance) {
    cout << "\nTesting FAST Detection Performance" << endl;
    cout << "Performing " << WARMUP_RUNS << " warmup run(s) and " 
         << MEASURE_RUNS << " measurement runs" << endl;
    
    vector<KeyPoint> keypoints;
    
    // Serial version
    double serialTime = measureAverageTime([&]() {
        keypoints.clear();
        FAST(testImage, keypoints, iniThFAST, true);
    }, WARMUP_RUNS, MEASURE_RUNS);

    // Parallel version
    double parallelTime = measureAverageTime([&]() {
        keypoints.clear();
        FAST(testImage, keypoints, iniThFAST, true);
    }, WARMUP_RUNS, MEASURE_RUNS);

    cout << "\nAverage serial execution time: " << serialTime << "ms" << endl;
    cout << "Average parallel execution time: " << parallelTime << "ms" << endl;
    cout << "Speedup: " << serialTime/parallelTime << "x" << endl;
}

// Test 3: Orientation Computation
TEST_F(ORBTest, OrientationComputationPerformance) {
    cout << "\nTesting Orientation Computation Performance" << endl;
    cout << "Performing " << WARMUP_RUNS << " warmup run(s) and " 
         << MEASURE_RUNS << " measurement runs" << endl;
    
    vector<KeyPoint> keypoints;
    FAST(testImage, keypoints, iniThFAST, true);
    vector<int> umax(HALF_PATCH_SIZE + 1);
    
    // Serial version
    double serialTime = measureAverageTime([&]() {
        for(auto& kp : keypoints) {
            kp.angle = IC_Angle(testImage, kp.pt, umax);
        }
    }, WARMUP_RUNS, MEASURE_RUNS);

    // Parallel version
    double parallelTime = measureAverageTime([&]() {
        #pragma omp parallel for
        for(size_t i = 0; i < keypoints.size(); i++) {
            keypoints[i].angle = IC_Angle(testImage, keypoints[i].pt, umax);
        }
    }, WARMUP_RUNS, MEASURE_RUNS);

    cout << "\nAverage serial execution time: " << serialTime << "ms" << endl;
    cout << "Average parallel execution time: " << parallelTime << "ms" << endl;
    cout << "Speedup: " << serialTime/parallelTime << "x" << endl;
}

// Test 4: Descriptor Computation
TEST_F(ORBTest, DescriptorComputationPerformance) {
    cout << "\nTesting Descriptor Computation Performance" << endl;
    cout << "Performing " << WARMUP_RUNS << " warmup run(s) and " 
         << MEASURE_RUNS << " measurement runs" << endl;
    
    vector<KeyPoint> keypoints;
    FAST(testImage, keypoints, iniThFAST, true);
    vector<Point> pattern;
    const int npoints = 512;
    const Point* pattern0 = (const Point*)bit_pattern_31_;
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

    Mat descriptors, descriptors_p;
    
    // Serial version
    double serialTime = measureAverageTime([&]() {
        descriptors = Mat::zeros((int)keypoints.size(), 32, CV_8UC1);
        for(size_t i = 0; i < keypoints.size(); i++) {
            computeOrbDescriptor(keypoints[i], testImage, &pattern[0], 
                               descriptors.ptr((int)i));
        }
    }, WARMUP_RUNS, MEASURE_RUNS);

    // Parallel version
    double parallelTime = measureAverageTime([&]() {
        descriptors_p = Mat::zeros((int)keypoints.size(), 32, CV_8UC1);
        #pragma omp parallel for
        for(size_t i = 0; i < keypoints.size(); i++) {
            computeOrbDescriptor(keypoints[i], testImage, &pattern[0], 
                               descriptors_p.ptr((int)i));
        }
    }, WARMUP_RUNS, MEASURE_RUNS);

    cout << "\nAverage serial execution time: " << serialTime << "ms" << endl;
    cout << "Average parallel execution time: " << parallelTime << "ms" << endl;
    cout << "Speedup: " << serialTime/parallelTime << "x" << endl;
}

// Thread Scaling Test
TEST_F(ORBTest, ThreadScalingTest) {
    cout << "\nTesting Thread Scaling" << endl;
    cout << "Performing " << WARMUP_RUNS << " warmup run(s) and " 
         << MEASURE_RUNS << " measurement runs" << endl;
    
    vector<KeyPoint> keypoints;
    FAST(testImage, keypoints, iniThFAST, true);
    vector<int> umax(HALF_PATCH_SIZE + 1);
    
    vector<double> times;
    
    for(int numThreads : threadCounts) {
        omp_set_num_threads(numThreads);
        
        double time = measureAverageTime([&]() {
            #pragma omp parallel for
            for(size_t i = 0; i < keypoints.size(); i++) {
                keypoints[i].angle = IC_Angle(testImage, keypoints[i].pt, umax);
            }
        }, WARMUP_RUNS, MEASURE_RUNS);
        
        times.push_back(time);
        cout << "\nThread count: " << numThreads << endl;
        cout << "Average execution time: " << time << "ms" << endl;
    }
    
    auto minTime = min_element(times.begin(), times.end());
    int optimalThreads = threadCounts[minTime - times.begin()];
    cout << "\nOptimal thread count: " << optimalThreads << endl;
}

} // namespace ORB_SLAM3

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}