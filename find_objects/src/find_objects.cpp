#include <ros/ros.h>

#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core/core.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>
#include <iostream>
#include <fstream>



using namespace cv;
using namespace std;



class ObjectsFinder
{
public:
	ros::NodeHandle nh_;
	ros::NodeHandle n;
	ros::Publisher pub ;
	image_transport::ImageTransport it_;    
	image_transport::Subscriber image_sub_; //image subscriber 
	image_transport::Publisher image_pub_; //image publisher
	//sensor_msgs::CvBridge bridge;


	//internal vars
	vector<Mat> img_objects;
	vector <string> object_names;
	int minHessian; 
	SurfFeatureDetector *detector;
	vector<KeyPoint> keypoints_scene;
	vector< vector<KeyPoint> > keypoints_objects;
	SurfDescriptorExtractor extractor;
	Mat descriptors_scene;
	vector<Mat> descriptors_objects;
	FlannBasedMatcher matcher;

	int current_object;
	bool current_only;

	//graph: id0 -> id1 id2 ed4 ..
	vector < vector <int> > visibility_graph;

	// image size 
	Size scene_size;




	string removePath(string filename) {
	    size_t lastdot = filename.find_last_of("/");
	    if (lastdot == 0) return filename;
	    return filename.substr(lastdot+1, filename.size()); 
	}
	string removeExtension(string filename) {
	    size_t lastdot = filename.find_last_of(".");
	    if (lastdot == string::npos) return filename;
	    return filename.substr(0, lastdot); 
	}


	int dist(Point a, Point b)
	{
	  return sqrt( (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) );
	}


	bool isSquared( vector<Point> points)
	{
	    if( abs( dist(points[0],points[1]) - dist(points[2],points[3]) ) < 50
	      &&
	        abs( dist(points[1],points[2]) - dist(points[0],points[3]) ) < 50
	      &&
	        abs( dist(points[0],points[1]) - dist(points[1],points[2]) ) < 100
	        )
	      return true;
	    
	  return false;

	}


	void readme()
	{ 
	  std::cout << " Usage: ./find_objects <camera id> <objects_file> <graph_file>" << std::endl; 
	}



	// read object images from a text file
	bool readObjectsFile(char *filename)
	{
	  ifstream myReadFile;
	  myReadFile.open(filename);
	  char img_name[100];
	  vector<KeyPoint> keypoints_object;
	  Mat descriptors_object;


	  if (myReadFile.is_open()) {
	   while (!myReadFile.eof()) 
	   {
	      myReadFile >> img_name;
	      if(myReadFile.eof()) break;
	      cout<<"Loading object: "<< img_name << "..."<<endl;
	      Mat img_object = imread( img_name, CV_LOAD_IMAGE_GRAYSCALE );
	      if(!img_object.data)
	        return false;
	      object_names.push_back(removePath(removeExtension(img_name)));
	      resize(img_object,img_object, Size((float)scene_size.width/2.0, (float)scene_size.width/2.0* ((float)img_object.rows/(float)img_object.cols)));
	      char s[20];
	      sprintf(s,"object %d",(int)img_objects.size());
	      imshow(s,img_object);

	      img_objects.push_back(img_object);
	      
	      detector->detect( img_object, keypoints_object );
	      extractor.compute( img_object, keypoints_object, descriptors_object );

	      keypoints_objects.push_back(keypoints_object);
	      descriptors_objects.push_back(descriptors_object);

	   }
	   myReadFile.close();
	  }
	  else 
	    return false;
	  return true;
	}


	// read object images from a text file
	bool readGraphFile(char *filename)
	{

	  visibility_graph.resize(img_objects.size());
	  ifstream myReadFile;
	  myReadFile.open(filename);
	  char img_name[100];

	  int a,b;

	  if (myReadFile.is_open()) {
	   while (!myReadFile.eof()) 
	   {
	      myReadFile >> a >> b;
	      if(myReadFile.eof()) break;

	      visibility_graph[a].push_back(b);
	   }
	   myReadFile.close();

	   for(int i=0;i<visibility_graph.size();i++)
	   {
	    cout << i <<"-->";
	    for(int j=0;j<visibility_graph[i].size();j++)
	       cout << visibility_graph[i][j] << " "; 
	         cout << endl;

	    }
	  }
	  else 
	    return false;
	  return true;
	}


	ObjectsFinder() : it_(nh_)
	{

		current_only=false;
		current_object=0;
		scene_size.width=320;
		scene_size.height=240;
		minHessian=100;
		detector= new SurfFeatureDetector(minHessian);
		
		// init pub and subscr
		image_sub_ = it_.subscribe("/gscam/image_raw", 1, &ObjectsFinder::imageCb, this);
    	image_pub_= it_.advertise("/find_objects/debug",1);

    	//read objects and graph from file	
		if(!readObjectsFile("/home/stefano/catkin_ws/src/find_objects/data/objects_lab10.txt"))
		{
			cout << "Error reading object files." << endl; 
	
		}  

		if(!readGraphFile("/home/stefano/catkin_ws/src/find_objects/data/graph.txt"))
		{
			cout << "Error reading graph file." << endl; 
		
		}  


	}


	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{

		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}


		//IplImage* img = bridge.imgMsgToCv(msg,"bgr8");  //image being converted from ros to opencv using cvbridge
		// IplImage* out1 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );   //make sure to feed the image(img) data to the parameters necessary for canny edge output 
		// IplImage* gray_out = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 ); 
		// IplImage* canny_out = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
		// IplImage* gray_out1=cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
		// IplImage* img1 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 ); 
		// cvCvtColor(img, gray_out, CV_BGR2GRAY);
		// cvSmooth(gray_out, gray_out, CV_GAUSSIAN, 9, 9); 
		// cvCanny( gray_out, canny_out, 50, 125, 3 );
		// cvCvtColor(canny_out ,gray_out1, CV_GRAY2BGR);
		// cvShowImage( "ARDRONE FEED",img);
		// cvShowImage( " CANNY EDGE DETECTION ",gray_out1);
		// cvWaitKey(2);   

        Mat result= detectObjects(cv_ptr->image);


		cv_bridge::CvImage cv_out_ptr;
    	cv_out_ptr.encoding = sensor_msgs::image_encodings::RGB8;
    	cv_out_ptr.image = result;		
		cv_out_ptr.header.stamp = ros::Time::now();
		std::cout << "time_stamp = " << cv_out_ptr.header.stamp << std::endl;

  		image_pub_.publish(cv_out_ptr.toImageMsg());

	}



	Mat detectObjects(Mat frame)
	{

  	resize(frame,frame,scene_size);
    Mat img_scene(frame.size(),CV_8UC1);
    cvtColor(frame,img_scene,CV_BGR2GRAY);

    detector->detect( img_scene, keypoints_scene );
    extractor.compute( img_scene, keypoints_scene, descriptors_scene );


    Mat img_matches=frame.clone();


    // search for each object
    for(int n=0;n<img_objects.size();n++)
      if(!current_only || current_object==n)
    {
      std::vector< DMatch > matches;

      if(keypoints_objects[n].size()<4 || keypoints_scene.size()<4)
        continue;

      matcher.match( descriptors_objects[n], descriptors_scene, matches );

      double max_dist = 0; double min_dist = 100;

      //-- Quick calculation of max and min distances between keypoints
      for( int i = 0; i < descriptors_objects[n].rows; i++ )
      { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }

      cout << "Object " << n << ":" << endl;
      printf("  Max dist : %f \n", max_dist );
      printf("  Min dist : %f \n", min_dist );

      //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
      std::vector< DMatch > good_matches;

      for( int i = 0; i < descriptors_objects[n].rows; i++ )
      { if( matches[i].distance < 3*min_dist )
         { good_matches.push_back( matches[i]); }
      }

      // Mat img_matches;
      if(n==current_object && current_only)
      drawMatches( img_objects[n], keypoints_objects[n], img_scene, keypoints_scene,
                   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

      //-- Localize the object
      std::vector<Point2f> obj;
      std::vector<Point2f> scene;

      for( int i = 0; i < good_matches.size(); i++ )
      {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_objects[n][ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
      }

      if(obj.size()>4 && scene.size()>4)
      {
        Mat H = findHomography( obj, scene, CV_RANSAC );

        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0); 
        obj_corners[1] = cvPoint( img_objects[n].cols, 0 );
        obj_corners[2] = cvPoint( img_objects[n].cols, img_objects[n].rows ); 
        obj_corners[3] = cvPoint( 0, img_objects[n].rows );
        std::vector<Point2f> scene_corners(4);

        perspectiveTransform( obj_corners, scene_corners, H);

        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
         // line( img_matches, scene_corners[0] + Point2f( img_objects[n].cols, 0), scene_corners[1] + Point2f( img_objects[n].cols, 0), Scalar(0, 0, 255), 4 );
         //  line( img_matches, scene_corners[1] + Point2f( img_objects[n].cols, 0), scene_corners[2] + Point2f( img_objects[n].cols, 0), Scalar( 0, 0, 255), 4 );
         //  line( img_matches, scene_corners[2] + Point2f( img_objects[n].cols, 0), scene_corners[3] + Point2f( img_objects[n].cols, 0), Scalar( 0, 0, 255), 4 );
         //  line( img_matches, scene_corners[3] + Point2f( img_objects[n].cols, 0), scene_corners[0] + Point2f( img_objects[n].cols, 0), Scalar( 0, 0, 255), 4 );



        vector<Point> obj_corners_p(4);
        obj_corners_p[0] = scene_corners[0]; 
        obj_corners_p[1] = scene_corners[1]; 
        obj_corners_p[2] = scene_corners[2]; 
        obj_corners_p[3] = scene_corners[3]; 


        vector<Point> hull;  // Convex hull points 
        vector<Point> contour;  // Convex hull contour points        

        // Calculate convex hull of original points (which points positioned on the boundary)
        convexHull(obj_corners_p,hull,false);
        // Approximating polygonal curve to convex hull
        approxPolyDP(Mat(hull), contour, 1, true);
        cout << Mat(contour) << endl;

        float cont=(contourArea(Mat(contour)));
        ROS_INFO("Area: %f",cont);

        if(isSquared(obj_corners_p) && dist(obj_corners_p[0],obj_corners_p[1])>50 && dist(obj_corners_p[1],obj_corners_p[2])>50 )
        {    
          char s[20];
          sprintf(s,"Object %d",n);
        if(current_only)
        {
          line( img_matches, scene_corners[0] + Point2f( img_objects[n].cols, 0), scene_corners[1] + Point2f( img_objects[n].cols, 0), Scalar(255, 0, 0), 4 );
         line( img_matches, scene_corners[1] + Point2f( img_objects[n].cols, 0), scene_corners[2] + Point2f( img_objects[n].cols, 0), Scalar(255, 0, 0), 4 );
         line( img_matches, scene_corners[2] + Point2f( img_objects[n].cols, 0), scene_corners[3] + Point2f( img_objects[n].cols, 0), Scalar( 255, 0, 0), 4 );
         line( img_matches, scene_corners[3] + Point2f( img_objects[n].cols, 0), scene_corners[0] + Point2f( img_objects[n].cols, 0), Scalar( 255, 0, 0), 4 );
          putText(img_matches, object_names[n], Point(scene_corners[0].x+img_objects[n].cols, scene_corners[0].y-10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0), 2);        
        }
        else
        {

          // TODO: asccociate real height to every image for distance estimation		
          float obj_dist_m =  ( (float)scene_size.height / (float)dist(obj_corners_p[1],obj_corners_p[2]) ) * 0.3;   // x oggetto alto 20 cm	
          char s[20];
          sprintf(s,"%.2f",obj_dist_m);

          if(n==current_object)
          {  
            line( img_matches, scene_corners[0], scene_corners[1], Scalar(0, 0, 255), 4 );
            line( img_matches, scene_corners[1], scene_corners[2], Scalar( 0, 0, 255), 4 );
            line( img_matches, scene_corners[2], scene_corners[3], Scalar( 0, 0, 255), 4 );
            line( img_matches, scene_corners[3], scene_corners[0], Scalar( 0, 0, 255), 4 );

   //          putText(img_matches, "0", scene_corners[0], FONT_HERSHEY_SIMPLEX, 0.2, Scalar(0,0,255), 2);
			// putText(img_matches, "1", scene_corners[1], FONT_HERSHEY_SIMPLEX, 0.2, Scalar(0,0,255), 2);
   //          putText(img_matches, "2", scene_corners[2], FONT_HERSHEY_SIMPLEX, 0.2, Scalar(0,0,255), 2);
			// putText(img_matches, "3", scene_corners[3], FONT_HERSHEY_SIMPLEX, 0.2, Scalar(0,0,255), 2);

			putText(img_matches, object_names[n], Point(scene_corners[0].x, scene_corners[0].y-10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);
			putText(img_matches, s, Point(scene_corners[0].x, scene_corners[0].y+10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);


          }
          else
          {  
            line( img_matches, scene_corners[0], scene_corners[1], Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1], scene_corners[2], Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2], scene_corners[3], Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3], scene_corners[0], Scalar( 0, 255, 0), 4 );

   //          putText(img_matches, "0", scene_corners[0], FONT_HERSHEY_SIMPLEX, 0.2, Scalar( 0, 255, 0), 2);
			// putText(img_matches, "1", scene_corners[1], FONT_HERSHEY_SIMPLEX, 0.2, Scalar( 0, 255, 0), 2);
   //          putText(img_matches, "2", scene_corners[2], FONT_HERSHEY_SIMPLEX, 0.2, Scalar( 0, 255, 0), 2);
			// putText(img_matches, "3", scene_corners[3], FONT_HERSHEY_SIMPLEX, 0.2, Scalar( 0, 255, 0), 2);

            putText(img_matches, object_names[n], Point(scene_corners[0].x, scene_corners[0].y-10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0), 2);
            putText(img_matches, s, Point(scene_corners[0].x, scene_corners[0].y+10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0), 2);

          }  
        }
        }
      }

    }
    return img_matches;

	}


};




int main( int argc, char** argv )
{
	ros::init(argc, argv, "find_objects");
	ObjectsFinder of;
	ros::spin();
	return 0;
}


