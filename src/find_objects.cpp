/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2015, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
*****************************************************************************/

#include "find_objects.h"
#include "utils.h"

// ObjectsFinderFeatureBase
//
void ObjectsFinderFeature2D::init(int width, int height)
{
            
    current_only=true;
    current_object=2;
    scene_size.width=width;
    scene_size.height=height;
}


string ObjectsFinderFeature2D::removePath(string filename) {
	    size_t lastdot = filename.find_last_of("/");
	    if (lastdot == 0) return filename;
	    return filename.substr(lastdot+1, filename.size()); 
	}
string ObjectsFinderFeature2D::removeExtension(string filename) {
	    size_t lastdot = filename.find_last_of(".");
	    if (lastdot == string::npos) return filename;
	    return filename.substr(0, lastdot); 
	}


int dist(Point a, Point b)
	{
	  return sqrt( (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) );
	}


void readme()
	{ 
	  std::cout << " Usage: ./find_objects <camera id> <objects_file> <graph_file>" << std::endl; 
	}


// read object images from a text file
bool ObjectsFinderFeature2D::readObjectsFile(char *filename)
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

        //equalizeHist(img_object, img_object);

        std::string obj_name = removePath(removeExtension(img_name));
        /*if (obj_name == "ipad") {
          newWindow(img_name, &img_object);
          //waitKey(0);
        }
        */

	      object_names.push_back(obj_name);
	      resize(img_object,img_object, Size((float)scene_size.width/2.0, (float)scene_size.width/2.0* ((float)img_object.rows/(float)img_object.cols)));
	      char s[20];
	      sprintf(s,"object %d",(int)img_objects.size());
	      //imshow(s,img_object);

	      img_objects.push_back(img_object);

        detectAndCompute(img_object, keypoints_object, descriptors_object);	      

	      keypoints_objects.push_back(keypoints_object);
	      descriptors_objects.push_back(descriptors_object);

	   }
	   myReadFile.close();
	  }
	  else 
	    return false;
	    return true;
	}


#if 1
// ObjectsFinderSURF
//
void ObjectsFinderSURF::init(int width, int height)
{
    ObjectsFinderFeature2D::init(width, height);

    minHessian=100;
    detector= new SurfFeatureDetector(minHessian);

    if(!readObjectsFile("/home/vibek/Human_intention/data/objects_lab10.txt"))
    {
      cout << "Error reading object files." << endl; 

    }  
    if(!readGraphFile("/home/vibek/Human_intention/data/graph.txt"))
    {
      cout << "Error reading graph file." << endl; 

    }      
}

bool ObjectsFinderSURF::detectAndCompute(Mat &img, vector<KeyPoint> &kp, Mat &desc)
{
    detector->detect( img, kp );
    extractor.compute( img, kp, desc );
    return true;
}

bool ObjectsFinderSURF::isSquared( vector<Point> points)
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

// read object images from a text file
bool ObjectsFinderSURF::readGraphFile(char *filename)
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



Mat ObjectsFinderSURF::detectObjects(Mat frame)
{
  	//resize(frame,frame,scene_size);
    Mat img_scene(frame.size(),CV_8UC1);
    cvtColor(frame,img_scene,CV_BGR2GRAY);

#if 1
    detector->detect( img_scene, keypoints_scene );
    extractor.compute( img_scene, keypoints_scene, descriptors_scene );
#endif


    Mat img_matches=frame.clone();

#if 1
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

//      cout << "Object " << n << ":" << endl;
//      printf("  Max dist : %f \n", max_dist );
//      printf("  Min dist : %f \n", min_dist );

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
//        cout << Mat(contour) << endl;

        float cont=(contourArea(Mat(contour)));
//        printf("Area: %f",cont);

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

			putText(img_matches, object_names[n], Point(scene_corners[0].x, scene_corners[0].y-10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);
			putText(img_matches, s, Point(scene_corners[0].x, scene_corners[0].y+10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255), 2);


          }
          else
          {  
            line( img_matches, scene_corners[0], scene_corners[1], Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1], scene_corners[2], Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2], scene_corners[3], Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3], scene_corners[0], Scalar( 0, 255, 0), 4 );


            putText(img_matches, object_names[n], Point(scene_corners[0].x, scene_corners[0].y-10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0), 2);
            putText(img_matches, s, Point(scene_corners[0].x, scene_corners[0].y+10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0), 2);

          }  
        }
        }
      }

    }
#endif

    return img_matches;
}
#endif


// ObjectsFinderORB
//
const double  NN_MATCH_RATIO  = 0.9f; // Nearest-neighbour matching ratio
const double  RANSAC_THRESH = 5.5f; // RANSAC inlier threshold

void ObjectsFinderORB::init(int width, int height)
{
  ObjectsFinderFeature2D::init(width, height);

  orbDetector = new cv::ORB();
  //orbDetector = cv::Feature2D::create("ORB");
  //orbDetector = cv::Feature2D::create("BRISK");

  matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");


  if(!readObjectsFile("/home/vibek/Human_intention/data/objects_lab10.txt"))
  {
    cout << "Error reading object files." << endl; 

  }  

  //newWindow("scene");
}

bool ObjectsFinderORB::detectAndCompute(Mat &img, vector<KeyPoint> &kp, Mat &desc)
{
  if (orbDetector) {
    (*orbDetector)(img, cv::noArray(), kp, desc);

    return true;
  }
  return false;
}

Mat ObjectsFinderORB::detectObjects(Mat frame)
{
    //resize(frame,frame,scene_size);
    Mat img_scene(frame.size(),CV_8UC1);
    cvtColor(frame,img_scene,CV_BGR2GRAY);

    //equalizeHist(img_scene, img_scene);

    //imshow("scene", img_scene);
    //waitKey(0);

    if (!detectAndCompute(img_scene, keypoints_scene, descriptors_scene))
    {
      return img_scene.clone();
    }

    Mat img_matches=frame.clone();


    // search for each object
    for(int n=0;n<img_objects.size();n++)
      if(!current_only || current_object==n)
    {
      if(keypoints_objects[n].size()<4 || keypoints_scene.size()<4)
        continue;

    //Match
      std::vector<cv::KeyPoint> matched1, matched2;
      cv::Mat matMatched1, matMatched2;
      {
          std::vector< std::vector<cv::DMatch> > matches;

          matcher->knnMatch(descriptors_objects[n], descriptors_scene, matches, 2);

          for(unsigned i = 0; i < matches.size(); i++)
          {
            if(matches[i][0].distance < NN_MATCH_RATIO * matches[i][1].distance)
            {
              matched1.push_back( keypoints_objects[n][matches[i][0].queryIdx] );
              matched2.push_back( keypoints_scene[matches[i][0].trainIdx] );
            }
          }
      }

    // Homography
      cv::Mat inlierMask, homography;
      {
        if(matched1.size() >= 4)
        {
          homography = cv::findHomography(Points(matched1), Points(matched2),
                  cv::RANSAC, RANSAC_THRESH, inlierMask);

        }
      }

    // Result
      if(matched1.size() > 4 && !homography.empty())
      {
      // Compute inliers
        std::vector<cv::KeyPoint> inliers1, inliers2;
        std::vector<cv::DMatch>   inlierMatches;

        for(unsigned i = 0; i < matched1.size(); i++)
        {
          if(inlierMask.at<uchar>(i))
          {
            int new_i = static_cast<int>(inliers1.size());

            inliers1.push_back(matched1[i]);
            inliers2.push_back(matched2[i]);
            inlierMatches.push_back(cv::DMatch(new_i, new_i, 0));
          }
        }

      // compute bounding box
        {
          std::vector<cv::Point2f> objectBB(4);
          objectBB[0] = cvPoint(0, 0);
          objectBB[1] = cvPoint(img_objects[n].cols, 0);
          objectBB[2] = cvPoint(img_objects[n].cols, img_objects[n].rows);
          objectBB[3] = cvPoint(0, img_objects[n].rows);

          std::vector<cv::Point2f> newBB;
          cv::perspectiveTransform(objectBB, newBB, homography);

          //if(nInliers >= BB_MIN_INLIERS) {
            drawBoundingBox(img_matches, newBB);
          //}
        }
      }


    } // end foreach(objectToFind)

    return img_matches;
}
#if 1
void ObjectsFinderCMB::init(int width, int height)
{
  ObjectsFinderFeature2D::init(width, height);

  CMBdetector = new cv::DynamicAdaptedFeatureDetector (new FastAdjuster(10, true), 5000, 10000, 10);
  CMBextractor = cv::DescriptorExtractor::create("SIFT");
  //orbDetector = cv::Feature2D::create("ORB");
  //orbDetector = cv::Feature2D::create("BRISK");

  matcher = cv::DescriptorMatcher::create("BruteForce");


  if(!readObjectsFile("/home/vibek/Human_intention/data/objects_lab10.txt"))
  {
    cout << "Error reading object files." << endl; 

  }  

  //newWindow("scene");
}

bool ObjectsFinderCMB::detectAndCompute(Mat &img, vector<KeyPoint> &kp, Mat &desc)
{
    CMBdetector->detect( img, kp );
    CMBextractor->compute( img, kp, desc );
    return true;
}

Mat ObjectsFinderCMB::detectObjects(Mat frame)
{
    //resize(frame,frame,scene_size);
    Mat img_scene(frame.size(),CV_8UC1);
    cvtColor(frame,img_scene,CV_BGR2GRAY);

    //equalizeHist(img_scene, img_scene);

    //imshow("scene", img_scene);
    //waitKey(0);

    if (!detectAndCompute(img_scene, keypoints_scene, descriptors_scene))
    {
      return img_scene.clone();
    }

    Mat img_matches=frame.clone();


    // search for each object
    for(int n=0;n<img_objects.size();n++)
      if(!current_only || current_object==n)
    {
      if(keypoints_objects[n].size()<4 || keypoints_scene.size()<4)
        continue;

    //Match
      std::vector<cv::KeyPoint> matched1, matched2;
      cv::Mat matMatched1, matMatched2;
      {
          std::vector< std::vector<cv::DMatch> > matches;

          matcher->knnMatch(descriptors_objects[n], descriptors_scene, matches, 2);

          for(unsigned i = 0; i < matches.size(); i++)
          {
            if(matches[i][0].distance < NN_MATCH_RATIO * matches[i][1].distance)
            {
              matched1.push_back( keypoints_objects[n][matches[i][0].queryIdx] );
              matched2.push_back( keypoints_scene[matches[i][0].trainIdx] );
            }
          }
      }

    // Homography
      cv::Mat inlierMask, homography;
      {
        if(matched1.size() >= 4)
        {
          homography = cv::findHomography(Points(matched1), Points(matched2),
                  cv::RANSAC, RANSAC_THRESH, inlierMask);

        }
      }

    // Result
      if(matched1.size() > 4 && !homography.empty())
      {
      // Compute inliers
        std::vector<cv::KeyPoint> inliers1, inliers2;
        std::vector<cv::DMatch>   inlierMatches;

        for(unsigned i = 0; i < matched1.size(); i++)
        {
          if(inlierMask.at<uchar>(i))
          {
            int new_i = static_cast<int>(inliers1.size());

            inliers1.push_back(matched1[i]);
            inliers2.push_back(matched2[i]);
            inlierMatches.push_back(cv::DMatch(new_i, new_i, 0));
          }
        }

      // compute bounding box
        {
          std::vector<cv::Point2f> objectBB(4);
          objectBB[0] = cvPoint(0, 0);
          objectBB[1] = cvPoint(img_objects[n].cols, 0);
          objectBB[2] = cvPoint(img_objects[n].cols, img_objects[n].rows);
          objectBB[3] = cvPoint(0, img_objects[n].rows);

          std::vector<cv::Point2f> newBB;
          cv::perspectiveTransform(objectBB, newBB, homography);

          //if(nInliers >= BB_MIN_INLIERS) {
            drawBoundingBox(img_matches, newBB);
          //}
        }
      }


    } // end foreach(objectToFind)

    return img_matches;
}

#endif
