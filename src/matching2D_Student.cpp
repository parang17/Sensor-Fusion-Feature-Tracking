#include <numeric>
#include "matching2D.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{


    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    int normType;

    if (matcherType.compare("MAT_BF") == 0)
    {
        if (descriptorType.compare("DES_BINARY"))
        {
            normType = cv::NORM_HAMMING;
        }
        else if (descriptorType.compare("DES_HOG"))
        {
            normType = cv::NORM_L2;
        }
        else{
            cout << "The descriptor type does not exist" << endl;
        }
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        {
            descSource.convertTo(descSource, CV_32F);
        }

        if (descRef.type() != CV_32F)
        {
            descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    {

        vector<vector<cv::DMatch>> knn_match_result;
        matcher->knnMatch(descSource, descRef, knn_match_result, 2);

        // implement the descriptor distance ratio test as a filtering method to remove bad keypoint matches.
        double min_desc_distance_ratio = 0.8;
        for (auto& knn_match: knn_match_result){
            if (knn_match[0].distance < min_desc_distance_ratio * knn_match[1].distance)
            {
                matches.push_back(knn_match[0]);
            }
        }
        cout << "Removed keypoints by KNN: " << knn_match_result.size() - matches.size() << endl;
    }

//    return t;
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
double descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor - BRIEF, ORB, FREAK, AKAZE, SIFT

    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {

        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {

        int nfeatures = 500; //The maximum number of features to retain.
        float scaleFactor = 1.2f; //Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid,
        int nlevels = 8; //The number of pyramid levels.
        int edgeThreshold = 31; //This is size of the border where the features are not detected.
        int firstLevel = 0; //The level of pyramid to put source image to
        int WTA_K = 2; //The number of points that produce each element of the oriented BRIEF descriptor.
        auto scoreType = cv::ORB::HARRIS_SCORE;
        int patchSize = 31; //size of the patch used by the oriented BRIEF descriptor.
        int fastThreshold = 20; //the fast threshold

        extractor = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold,
                                    firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        bool orientationNormalized = true;
        bool scaleNormalized = true;
        float patternScale = 22.0f;
        int nOctaves = 4;

        extractor = cv::xfeatures2d::FREAK::create(orientationNormalized, scaleNormalized, patternScale, nOctaves);
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        auto descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB;
        int descriptor_size = 0;
        int descriptor_channels = 3;
        float threshold = 0.001f;
        int nOctaves = 4;
        int nOctaveLayers = 4;
        auto diffusivity = cv::KAZE::DIFF_PM_G2;


        extractor = cv::AKAZE::create(descriptor_type, descriptor_size, descriptor_channels,
                                      threshold, nOctaves, nOctaveLayers, diffusivity);
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        int nfeatures = 0;
        int nOctaveLayers = 3;
        double contrastThreshold = 0.04;
        double edgeThreshold = 10;
        double sigma = 1.6;

        extractor = cv::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
    }
    else
    {
        cout << "The descriptor does not exist" << endl;
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;

    return t;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
double detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    double t = (double)cv::getTickCount();
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return t;
}

double detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{

    double t = (double)cv::getTickCount();
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    int apertureSize = 3; // Aperture in Sobel operator(Should be odd)
    double k = 0.04; // Harris parameters
    int min_resp = 100; // minimum value for a corner in the 8bit scaled response matrix
    double maxOverlap = 0.0;// Maximum permissible overlap

    vector<cv::Point2f> corners;
    cv::Mat dst = cv::Mat::zeros( img.size(), CV_32FC1 );

    /// Detecting corners
    cornerHarris( img, dst, blockSize, apertureSize, k );
    cv::Mat dst_norm, dst_norm_scaled;
    normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );

    for( int j = 0; j < dst_norm.rows ; j++ )
    {
        for( int i = 0; i < dst_norm.cols; i++ )
        {
            int resp = static_cast<int>(dst_norm.at<float>(j,i));

            if (resp < min_resp)
            {
                continue;
            }

            //Update new key points
            cv::KeyPoint newKeyPoint;
            newKeyPoint.pt = cv::Point2f(i, j);
            newKeyPoint.size = 2 * apertureSize;
            newKeyPoint.response = resp; //number of points

            // Non-Maxima Suppression (NMS) in local neighbourhood around new key point
            // Scan the savaed keypoint vectors
            bool IsOverlap = false;
            for (auto& keypoint: keypoints)
            {
                double keypoint_overlap = cv::KeyPoint::overlap(newKeyPoint, keypoint);
                if (keypoint_overlap > maxOverlap)
                {
                    IsOverlap = true;
                    if (newKeyPoint.response > keypoint.response)
                    {
                        keypoint = newKeyPoint;
                        break;
                    }
                }
            }

            if (IsOverlap == false){
                keypoints.push_back(newKeyPoint);
            }
        }
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris corner detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    if (bVis)
    {
        string windowName = "Harris Resp. Matrix";
        cv::namedWindow(windowName);
        cv::imshow(windowName, dst_norm_scaled);
        cv::waitKey(0);
    }
    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return t;
}

double detKeypointsFAST(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{

    double t = (double)cv::getTickCount();
    // compute detector parameters based on image size
    int threshold = 30; //Intensity difference between two pixel
    bool isNMS = true; //Determine Nun-Mazima Suppression

    // Apply corner detection
    vector<cv::Point2f> corners;
    cv::Ptr<cv::FeatureDetector> detector;

    cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; //TYPE_9_16, TYPE_7_12, TYPE_5_8
    detector = cv::FastFeatureDetector::create(threshold, isNMS, type);
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "FAST detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;


    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "FAST Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return t;
}

double detKeypointsBRISK(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{

    double t = (double)cv::getTickCount();

    // compute detector parameters based on image size
    int threshold = 30; //Intensity difference between two pixel
    int octaves=3; //detection octaves. Use 0 to do single scale.
    float patternScale=1.0f; //pply this scale to the pattern used for sampling the neighbourhood of a keypoint.

    // Apply corner detection
    vector<cv::Point2f> corners;
    cv::Ptr<cv::FeatureDetector> detector;

    detector = cv::BRISK::create(threshold, octaves, patternScale);
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "BRISK detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;


    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "BRISK Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return t;
}

double detKeypointsORB(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{

    double t = (double)cv::getTickCount();

    // compute detector parameters based on image size
    int nfeatures=500;
    float scaleFactor=1.2f;
    int nlevels=8;
    int edgeThreshold=31;
    int firstLevel=0;
    int WTA_K=2;
    auto scoreType=cv::ORB::HARRIS_SCORE;
    int patchSize=31;

    // Apply corner detection
    vector<cv::Point2f> corners;
    cv::Ptr<cv::FeatureDetector> detector;

    detector = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize);
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "ORB detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;


    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "ORB Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return t;
}


double detKeypointsAKAZE(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{

    double t = (double)cv::getTickCount();

    // compute detector parameters based on image size
    auto descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB;
    int descriptor_size = 0;
    int descriptor_channels = 3;
    float threshold = 0.001f;
    int nOctaves = 4;
    int nOctaveLayers = 4;
    auto diffusivity = cv::KAZE::DIFF_PM_G2;

    // Apply corner detection
    vector<cv::Point2f> corners;
    cv::Ptr<cv::FeatureDetector> detector;

    detector = cv::AKAZE::create(descriptor_type, descriptor_size, descriptor_channels, threshold, nOctaves, nOctaveLayers, diffusivity);
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "AKAZE detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;


    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "AKAZE Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return t;
}


double detKeypointsSIFT(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{

    double t = (double)cv::getTickCount();

    // compute detector parameters based on image size
    int nfeatures = 0;
    int nOctaveLayers = 3;
    double contrastThreshold = 0.04;
    double edgeThreshold = 10;
    double sigma = 1.6;

    // Apply corner detection
    vector<cv::Point2f> corners;
    cv::Ptr<cv::FeatureDetector> detector;

    detector = cv::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "SIFT detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;


    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "SIFT Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return t;
}