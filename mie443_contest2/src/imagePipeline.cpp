#include <imagePipeline.h>
#include "opencv2/core.hpp"
#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this); /// this file we are in (image pipeline node) subscribes to the kinect sensor which is the IMAGETOPIC (gives us the image data messages)
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {   // everytime spinONCE is called in the main contest folder loop, this is run (this file is connected to the other file through #include, which includes the header file which contains the pipeline node, get template and callback)
    try {                                                                     // I think this callback converts the ROS images from the video stream to CV image using CV bridge and stores it in img
        if(isValid) {                                                           
            img.release();              
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();     // the video stream image is stored in the img variable
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {    //This is called in main contest file and is passed boxes templates (the given images) in the main contest file, and recieves the video stream image from the image callback
    int template_id = -1;
    if(!isValid) {          ///isValid is set from the image callback to check if there is a valid video stream image, this is executed if not valid, next is if empty
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {     //(if the image is valid)
        /***YOUR CODE HERE***/                 /// this should compare the boxes images (passed in the argument) to the vide str
        // Use: boxes.templates
        cv::imshow("view", img);   ///img is the video stream, we compare this with boxes
        cv::waitKey(10);
    }  
    return template_id;      ///this function will return the ID index to the contest file
}


/////////OPEN CV LINK - this will go in the getTemplateID since that is the main function that takes in the boxe imagers and returns the ID index


const char* keys =
        "{ help h |                          | Print help message. }"
        "{ input1 | ../data/box.png          | Path to input image 1. }"
        "{ input2 | ../data/box_in_scene.png | Path to input image 2. }";
int main( int argc, char* argv[] )
{
    CommandLineParser parser( argc, argv, keys );
    Mat img_object = imread( parser.get<String>("input1"), IMREAD_GRAYSCALE );
    Mat img_scene = imread( parser.get<String>("input2"), IMREAD_GRAYSCALE );
    if ( img_object.empty() || img_scene.empty() )
    {
        cout << "Could not open or find the image!\n" << endl;
        parser.printMessage();
        return -1;
    }
    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints_object, keypoints_scene;
    Mat descriptors_object, descriptors_scene;
    detector->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object );
    detector->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene );
    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 );
    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.75f;
    std::vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    //-- Draw matches
    Mat img_matches;
    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Localize the object
    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    Mat H = findHomography( obj, scene, RANSAC );
    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = Point2f(0, 0);
    obj_corners[1] = Point2f( (float)img_object.cols, 0 );
    obj_corners[2] = Point2f( (float)img_object.cols, (float)img_object.rows );
    obj_corners[3] = Point2f( 0, (float)img_object.rows );
    std::vector<Point2f> scene_corners(4);
    perspectiveTransform( obj_corners, scene_corners, H);
    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    line( img_matches, scene_corners[0] + Point2f((float)img_object.cols, 0),
          scene_corners[1] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4 );
    line( img_matches, scene_corners[1] + Point2f((float)img_object.cols, 0),
          scene_corners[2] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[2] + Point2f((float)img_object.cols, 0),
          scene_corners[3] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, scene_corners[3] + Point2f((float)img_object.cols, 0),
          scene_corners[0] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
    //-- Show detected matches
    imshow("Good Matches & Object detection", img_matches );
    waitKey();
    return 0;
}
#else
int main()
{
    std::cout << "This tutorial code needs the xfeatures2d contrib module to be run." << std::endl;
    return 0;
}
#endif
