#include <imagePipeline.h>

// sensor_msgs::image_encodings is a message type, BGR8 is an 8 bit encoding field
#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
// #define IMAGE_TOPIC "camera/image" // kinect:"camera/rgb/image_raw" webcam:"camera/image"
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"


#include <iostream>
#include "opencv2/core.hpp"
//#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"                                            
using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;
const char* keys =
        "{ help h |                          | Print help message. }"
        "{ input1 | ../data/box.png          | Path to input image 1. }"
        "{ input2 | ../data/box_in_scene.png | Path to input image 2. }";

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
    // subscribing to receive the image
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // image is converted from the ROS message to a cv::Mat object
    try {
        if(isValid) {
            img.release(); //previous image is released from memory 
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        /***YOUR CODE HERE***/
        // CommandLineParser parser( argc, argv, keys );
        //cv::imshow("view", img);
        //cv::waitKey(30000); // waits for 10 milliseconds        
        Mat img_scene = img;
        int templateMatchArray[3];

        for (int i = 0; i <= 2; i++)
        {
            Mat img_object = boxes.templates[i];

        if (img_object.empty() || img_scene.empty() )
        {
            cout << "Could not open or find the image!\n" << endl;
            return -1;
        }
            
        //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors for img_object and img_scene
        int minHessian = 400; //determining the threshold/sensitivity of the detector to blob-like structure. increasing will result in fewer keypoints detected.
        Ptr<SURF> detector = SURF::create( minHessian ); //creates an instance of the SURF feature detector
        std::vector<KeyPoint> keypoints_object, keypoints_scene; //creating two empty vectors to hold the keypoints detected in img_object and img_scene
        Mat descriptors_object, descriptors_scene; //creating two empty MAT matrices to hold the descriptors computed for the keypoints
        detector->detectAndCompute( img_object, noArray(), keypoints_object, descriptors_object ); //detects keypoints and computes descriptors for img_object. 
        detector->detectAndCompute( img_scene, noArray(), keypoints_scene, descriptors_scene ); //detects keypoints and computes descriptors for img_scene. 
        //noArray() is used to specify that no mask is used for the detections
            
        //-- Step 2: Matching descriptor vectors with a FLANN based matcher
        // FLANN: fast library for approximate nearest neighbors --> used to match keypoints between two images using their feature descriptors
        // Since SURF is a floating-point descriptor NORM_L2 is used
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED); // creates an instance of of the FLANN-based descriptor matcher
        std::vector< std::vector<DMatch> > knn_matches; // creates an empty vector of vectors of DMatch objects to store the matches found by the descriptor matcher.
        // each element of the outer vector correspond to a query descriptor, and the inner vector contains the top k matches
        // kNN --> 'k'-nearest-neighbor --> search method for finding the k closest matches to a given query point
        matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2 ); // performs the kNN search on the descriptors of img_object and img_scene.
        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.75f; // set the ratio threshold used for filtering the matches.
        // The Lowe's ratio test is a common method used to remove false matches by copmaring the distance between the best and second-best for each query descriptor.
        std::vector<DMatch> good_matches; // creates an empty vector to store the good matches that pass the Lowe's ratio test
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }
        // adds valid matches to the goodmatches vector
            
        //-- Draw matches
        Mat img_matches; 
        drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1),
                    Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        // draws lines between the matching keypoints in the object and scene images. Stores the resulting image in img_matches. 
            
        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for( size_t i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }
        // obj and scene vectors are populated with the pixel coordinates of the matching keypoints    
            
        Mat H = findHomography( obj, scene, RANSAC );
        // findHomography function estimates the homography matrix H that maps the points in the object image to their corresponding points in the scene image.
            
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = Point2f(0, 0);
        obj_corners[1] = Point2f( (float)img_object.cols, 0 );
        obj_corners[2] = Point2f( (float)img_object.cols, (float)img_object.rows );
        obj_corners[3] = Point2f( 0, (float)img_object.rows );
        std::vector<Point2f> scene_corners(4);
        perspectiveTransform( obj_corners, scene_corners, H);
        // The four corners of img_object are defined.
        // perspectiveTransform() applies the homography matrix to map the corners from the object image to the scene image
            
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line( img_matches, scene_corners[0] + Point2f((float)img_object.cols, 0),
            scene_corners[1] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + Point2f((float)img_object.cols, 0),
            scene_corners[2] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + Point2f((float)img_object.cols, 0),
            scene_corners[3] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + Point2f((float)img_object.cols, 0),
            scene_corners[0] + Point2f((float)img_object.cols, 0), Scalar( 0, 255, 0), 4 );
        // draws lines between the mapped corners, which essentially draws a rectangle around the detected object in the scene image.

        templateMatchArray[i] = good_matches.size();
        std::cout << good_matches.size() << std::endl;

        // if (good_matches.size() < 100){
        //     std::cout << "No Match" << std::endl;
        //     continue;
        // }
        
        // else{
        //     template_id = i;
        //     std::cout << template_id << std::endl;
        //     imshow("Good Matches & Object detection", img_matches );
        //     waitKey(); // waits until a key is pressed
        //     break;
        // }

        // if (templateIter == 3)
        // {
        //     template_id = 4; //blank
        //     std::cout << template_id << std::endl;
        // }

        // Use: boxes.templates --> vector whose elements are grayscale cv::mat images of box templates
        // template id 1 = kelloggs raisin bran
        // template id 2 = cinnamon toast crunch
        // template id 3 = kelloggs rice krispies
        // this code will be triggered when the robot comes to a stop and is ready for image processing
        
        }

        int curr = 0;
        int max = 0;
        for (int j = 0; j < 3; j++)
        {
            curr = templateMatchArray[j];
            if (curr > max)
            {
                max = curr;
                template_id = j;
            }

        }
        if (max < 63)
        {
            template_id = 3; //blank
        }

    }  
    std::cout << template_id << std::endl;
    return template_id;
}