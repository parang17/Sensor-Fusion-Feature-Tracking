/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    // Summary results
    int total_detected_keypoints = 0;
    double detection_time = 0;
    double extraction_time = 0;
    int total_matched_keypoins = 0;
    double matching_time = 0;
    double total_time = 0;
    double total_t1 = 0;
    double total_t2 = 0;
    double total_t3 = 0;
    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        double time1 = 0;
        double time2 = 0;
        double time3 = 0;



        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);

        if (dataBuffer.size() > dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
        }

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        string detectorType = "SIFT";

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("ShiTomasi") == 0)
        {
            time1 = detKeypointsShiTomasi(keypoints, imgGray, bVis);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            time1 = detKeypointsHarris(keypoints, imgGray, bVis);
        }
        else if (detectorType.compare("FAST") == 0)
        {
            time1 = detKeypointsFAST(keypoints, imgGray, bVis);
        }
        else if (detectorType.compare("BRISK") == 0)
        {
            time1 = detKeypointsBRISK(keypoints, imgGray, bVis);
        }
        else if (detectorType.compare("ORB") == 0)
        {
            time1 = detKeypointsORB(keypoints, imgGray, bVis);
        }
        else if (detectorType.compare("AKAZE") == 0)
        {
            time1 = detKeypointsAKAZE(keypoints, imgGray, bVis);
        }
        else if (detectorType.compare("SIFT") == 0)
        {
            time1 = detKeypointsSIFT(keypoints, imgGray, bVis);
        }
        else
        {
            cout << "The keypoint detection method does not exist. " << endl;
        }
        //// EOF STUDENT ASSIGNMENT
        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        total_detected_keypoints += keypoints.size();

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);

        vector<cv::KeyPoint> filteredkeypoints;
        if (bFocusOnVehicle)
        {
            for (auto& keypoint:keypoints)
            {
                if (vehicleRect.contains(keypoint.pt))
                {
                    filteredkeypoints.push_back(keypoint);
                }
            }
        }
//        // Up[date the keypoints
        keypoints = filteredkeypoints;

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = true;
        if (bLimitKpts)
        {
            int maxKeypoints = 500;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        string descriptorType = "BRIEF"; // BRIEF, ORB, FREAK, AKAZE, SIFT
        time2 = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        // Summary of Results


        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */
            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
            string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);

            total_matched_keypoins += matches.size();

            //// EOF STUDENT ASSIGNMENT
            double t = (double)cv::getTickCount();
            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;
            t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
            time3 = t;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }

            total_time += time1 + time2 + time3;
            total_t1 += time1;
            total_t2 += time2;
            total_t3 += time3;


        }

        cout << endl;
        cout << "#===============================================" << endl;
        cout << "Image number: " << imgIndex << endl;
        cout << "Detector Type: " << detectorType << endl;
        cout << "Descriptor Type:" << descriptorType << endl;
        cout << "Number of keypoints: " << keypoints.size() << endl;
        cout << "Time for keypoint detection (ms): " << time1 * 1000.0 << endl;
        cout << "Time for descriptor (ms): " << time2 * 1000.0 << endl;
        cout << "Time for matching (ms): " << time3 * 1000.0 << endl;
        cout << "#===============================================" << endl;
        cout << endl;


    } // eof loop over all images

    cout << "#===============================================" << endl;
    cout << "#============= SUMMARY OF RESULT ===============" << endl;
    cout << "#===============================================" << endl;
    cout << "Avg. t1 computational time (ms): " << total_t1* 1000.0 /10 << endl;
    cout << "Avg. t2 computational time (ms): " << total_t2* 1000.0 /10 << endl;
    cout << "Avg. t3 computational time (ms): " << total_t3* 1000.0 /10 << endl;
    cout << "Avg. total computational time (ms): " << total_time* 1000.0 /10 << endl;
    cout << total_detected_keypoints << endl;
    cout << "Avg. number of keypoints: " << total_detected_keypoints / 10 << endl;
    cout << "Avg. number of matched points: " << total_matched_keypoins / 10 << endl;
    cout << "Avg. efficiency for matching (matches/ms): " << total_matched_keypoins/10  / (total_time * 1000.0) << endl;
    cout << "#===============================================" << endl;


    return 0;
}
