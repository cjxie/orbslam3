#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <gtest/gtest.h>
#include <omp.h>
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
            // Create synthetic test image if no file available
            testImage = Mat(800, 848, CV_8UC1);
            randu(testImage, 0, 255);
        }
        
        // Initialize ORB extractor with test parameters
        nfeatures = 1000;
        scaleFactor = 1.2;
        nlevels = 8;
        iniThFAST = 20;
        minThFAST = 7;
        
        orbExtractor = new ORBextractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);
    }

    void TearDown() override {
        delete orbExtractor;
    }

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

    // Helper function to measure execution time
    template<typename Func>
    double measureExecutionTime(Func&& func) {
        auto start = std::chrono::high_resolution_clock::now();
        func();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
        return duration.count();
    }

    Mat testImage;
    ORBextractor* orbExtractor;
    int nfeatures;
    float scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;
    std::vector<int> threadCounts = {1, 2, 4, 8, 12};
};

// Test 1: Image Pyramid Computation
TEST_F(ORBTest, PyramidComputationPerformance) {
    cout << "Testing Pyramid Computation Performance" << endl;
    const int num = 5;
    vector<double> times1, times2;

    // for (int i = 0; i < num; i++)
    // {
    //     // Parallel version
    //     double parallelTime = measureExecutionTime([&]() {
    //         // TODO: Implement parallel version
    //         TestComputePyramid_MP();
    //     });
    //     times2.push_back(parallelTime);
    //     // Serial version
    //     double serialTime = measureExecutionTime([&]() {
    //         TestComputePyramid();
            
    //     });
    //     times1.push_back(serialTime);
    // }

    // double serialTime = std::accumulate(times1.begin()+1, times1.end(), 0.0) / (num-1);
    // double parallelTime = std::accumulate(times2.begin()+1, times2.end(), 0.0) / (num-1);
    // cout << "Serial execution time: " << serialTime << "ms" << endl;
    // cout << "Parallel execution time: " << parallelTime << "ms" << endl;
    // cout << "Speedup: " << serialTime/parallelTime << "x" << endl;

    vector<double> times;
    
    for (auto &numThreads : threadCounts)
    {
        omp_set_num_threads(numThreads);
        times1.clear();
        times2.clear();
        for (int i = 0; i < num; i++)
        {
            // Serial version
            double serialTime = measureExecutionTime([&]() {
                TestComputePyramid();
                
            });
            times1.push_back(serialTime);

            // Parallel version
            double parallelTime = measureExecutionTime([&]() {
                // TODO: Implement parallel version
                TestComputePyramid_MP();
            });
            times2.push_back(parallelTime);
        }

        double serialTime = std::accumulate(times1.begin()+1, times1.end(), 0.0) / (num-1);
        double parallelTime = std::accumulate(times2.begin()+1, times2.end(), 0.0) / (num-1);
        cout << "Serial execution time: " << serialTime << "ms" << endl;
        cout << "Parallel execution time: " << parallelTime << "ms" << endl;
        cout << "Speedup: " << serialTime/parallelTime << "x" << endl;
        times.push_back(parallelTime);
    }

    auto minTimeIter = min_element(times.begin(), times.end());
    int optimalThreads = threadCounts[minTimeIter - times.begin()];
    cout << "Optimal thread count: " << optimalThreads << endl;

}

// Test 2: FAST Feature Detection
TEST_F(ORBTest, FASTDetectionPerformance) {
    cout << "Testing FAST Detection Performance" << endl;
    
    const int num = 5;
    vector<KeyPoint> keypoints;
    
    vector<double> times1, times2;

    // Serial version
    for(int i = 0; i < num ; i++)
    {
        double serialTime = measureExecutionTime([&]() {
            FAST(testImage, keypoints, iniThFAST, true);
        });
        // int serialKeypoints = keypoints.size();
        times1.push_back(serialTime);
        keypoints.clear();
        // Parallel version (using OpenCV's parallel FAST implementation)
        double parallelTime = measureExecutionTime([&]() {
            // Note: OpenCV's FAST implementation might already be parallel
            FAST(testImage, keypoints, iniThFAST, true);
        });
        times2.push_back(parallelTime);
        // int parallelKeypoints = keypoints.size();
    }
    
    double serialTime = std::accumulate(times1.begin()+1, times1.end(), 0.0) / (num-1);
    double parallelTime = std::accumulate(times2.begin()+1, times2.end(), 0.0) / (num-1);
    cout << "Serial execution time: " << serialTime << "ms" << endl;
    cout << "Parallel execution time: " << parallelTime << "ms" << endl;
    cout << "Speedup: " << serialTime/parallelTime << "x" << endl;
    // cout << "Keypoints detected: " << serialKeypoints << " (serial) vs " 
    //      << parallelKeypoints << " (parallel)" << endl;
}

// Test 3: Orientation Computation
TEST_F(ORBTest, OrientationComputationPerformance) {
    cout << "Testing Orientation Computation Performance" << endl;
    
    vector<KeyPoint> keypoints;
    FAST(testImage, keypoints, iniThFAST, true);
    vector<int> umax(HALF_PATCH_SIZE + 1);
    
    // Serial version
    double serialTime = measureExecutionTime([&]() {
        for(auto& kp : keypoints) {
            kp.angle = IC_Angle(testImage, kp.pt, umax);
        }
    });
    cout << "Serial execution time: " << serialTime << "ms" << endl;
    
    // Parallel version
    double parallelTime = measureExecutionTime([&]() {
        #pragma omp parallel for
        for(size_t i = 0; i < keypoints.size(); i++) {
            keypoints[i].angle = IC_Angle(testImage, keypoints[i].pt, umax);
        }
    });
    cout << "Parallel execution time: " << parallelTime << "ms" << endl;
    cout << "Speedup: " << serialTime/parallelTime << "x" << endl;
}

// Test 4: Descriptor Computation
TEST_F(ORBTest, DescriptorComputationPerformance) {
    cout << "Testing Descriptor Computation Performance" << endl;
    
    vector<KeyPoint> keypoints;
    FAST(testImage, keypoints, iniThFAST, true);
    vector<Point> pattern;  // Initialize pattern
    const int npoints = 512;
    const Point* pattern0 = (const Point*)bit_pattern_31_;
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

    Mat descriptors;
    
    // Serial version
    double serialTime = measureExecutionTime([&]() {
        descriptors = Mat::zeros((int)keypoints.size(), 32, CV_8UC1);
        for(size_t i = 0; i < keypoints.size(); i++) {
            computeOrbDescriptor(keypoints[i], testImage, &pattern[0], 
                               descriptors.ptr((int)i));
        }
    });
    cout << "Serial execution time: " << serialTime << "ms" << endl;
    

    Mat descriptors_p;
    vector<Point> pattern_p;  // Initialize pattern
    // Parallel version
    double parallelTime = measureExecutionTime([&]() {
        descriptors_p = Mat::zeros((int)keypoints.size(), 32, CV_8UC1);
        #pragma omp parallel for
        for(size_t i = 0; i < keypoints.size(); i++) {
            computeOrbDescriptor(keypoints[i], testImage, &pattern[0], 
                               descriptors_p.ptr((int)i));
        }
    });
    cout << "Parallel execution time: " << parallelTime << "ms" << endl;
    cout << "Speedup: " << serialTime/parallelTime << "x" << endl;
}

// Thread count test to find optimal number of threads
TEST_F(ORBTest, ThreadScalingTest) {
    cout << "Testing Thread Scaling" << endl;
    
    vector<KeyPoint> keypoints;
    FAST(testImage, keypoints, iniThFAST, true);
    vector<int> umax(HALF_PATCH_SIZE + 1);
    
    vector<double> times;
    vector<int> threadCounts = {1, 2, 4, 8, 12};
    
    for(int numThreads : threadCounts) {
        omp_set_num_threads(numThreads);
        
        double time = measureExecutionTime([&]() {
            #pragma omp parallel for
            for(size_t i = 0; i < keypoints.size(); i++) {
                keypoints[i].angle = IC_Angle(testImage, keypoints[i].pt, umax);
            }
        });
        
        times.push_back(time);
        cout << "Threads: " << numThreads << ", Time: " << time << "ms" << endl;
    }
    
    // Find optimal thread count
    auto minTime = min_element(times.begin(), times.end());
    int optimalThreads = threadCounts[minTime - times.begin()];
    cout << "Optimal thread count: " << optimalThreads << endl;
}
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}