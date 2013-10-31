#include <stdexcept>
#include "slaw_vision/TargetArray.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Char.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h> // cvFindContours

#include <slaw_navigation/switchOff.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "slaw_vision/TargetArray.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Char.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h> // cvFindContours
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>
#include <algorithm>
#include <math.h>
#include <std_msgs/String.h>

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

using namespace slaw_vision; 

// Function prototypes
void subtractPlane(const cv::Mat& depth, cv::Mat& mask, std::vector<CvPoint>& chain, double f);

std::vector<CvPoint> maskFromTemplate(const std::vector<cv::linemod::Template>& templates,
                                      int num_modalities, cv::Point offset, cv::Size size,
                                      cv::Mat& mask, cv::Mat& dst, Target& target);

void templateConvexHull(const std::vector<cv::linemod::Template>& templates,
                        int num_modalities, cv::Point offset, cv::Size size,
                        cv::Mat& dst, Target& target);

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T);

cv::Mat displayQuantized(const cv::Mat& quantized);

// Copy of cv_mouse from cv_utilities
class Mouse
{
public:
  static void start(const std::string& a_img_name)
  {
    cvSetMouseCallback(a_img_name.c_str(), Mouse::cv_on_mouse, 0);
  }
  static int event(void)
  {
    int l_event = m_event;
    m_event = -1;
    return l_event;
  }
  static int x(void)
  {
    return m_x;
  }
  static int y(void)
  {
    return m_y;
  }

private:
  static void cv_on_mouse(int a_event, int a_x, int a_y, int, void *)
  {
    m_event = a_event;
    m_x = a_x;
    m_y = a_y;
  }

  static int m_event;
  static int m_x;
  static int m_y;
};
int Mouse::m_event;
int Mouse::m_x;
int Mouse::m_y;

static void help()
{
  printf("Usage: openni_demo [templates.yml]\n\n"
         "Place your object on a planar, featureless surface. With the mouse,\n"
         "frame it in the 'color' window and right click to learn a first template.\n"
         "Then press 'l' to enter online learning mode, and move the camera around.\n"
         "When the match score falls between 90-95%% the demo will add a new template.\n\n"
         "Keys:\n"
         "\t h   -- This help page\n"
         "\t l   -- Toggle online learning\n"
         "\t m   -- Toggle printing match result\n"
         "\t t   -- Toggle printing timings\n"
         "\t w   -- Write learned templates to disk\n"
         "\t [ ] -- Adjust matching threshold: '[' down,  ']' up\n"
         "\t e r d u i -- undocumented...\n"
         "\t q   -- Quit\n\n");
}

// Adapted from cv_timer in cv_utilities
class Timer
{
public:
  Timer() : start_(0), time_(0) {}

  void start()
  {
    start_ = cv::getTickCount();
  }

  void stop()
  {
    CV_Assert(start_ != 0);
    int64 end = cv::getTickCount();
    time_ += end - start_;
    start_ = 0;
  }

  double time()
  {
    double ret = time_ / cv::getTickFrequency();
    time_ = 0;
    return ret;
  }

private:
  int64 start_, time_;
};

// Functions to store detector and templates in single XML/YAML file
static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename)
{
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}

static void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  detector->write(fs);

  std::vector<std::string> ids = detector->classIds();
  fs << "classes" << "[";
  for (int i = 0; i < (int)ids.size(); ++i)
    {
      fs << "{";
      detector->writeClass(ids[i], fs);
      fs << "}"; // current class
    }
  fs << "]"; // classes
}


namespace enc = sensor_msgs::image_encodings;

class Linemod
{

private:

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter rgb_image_sub_;
  image_transport::SubscriberFilter depth_image_sub_;
  //image_transport::SubscriberFilter pcl_sub_;
  //  image_transport::SubscriberFilter blob_image_sub_; //todo del
  //  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy; //blob
   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer< MySyncPolicy > sync;
  ros::Subscriber cam_spec_sub_;
  ros::Subscriber menue_sub_;
  ros::Publisher tar_pub_;
  ros::Publisher img_pub_;
  ros::Publisher cam_pos_pub_;
  ros::Subscriber pcl_sub_;
  pcl::PointCloud<pcl::PointXYZ> pc;  
  tf::TransformListener tf_listener;
  ros::ServiceServer serviceOff;
  
  // Various settings and flags
  bool server_mode;
  bool show_match_result;
  bool detect_all;
  bool show_timings;
  bool learn_online;
  int num_classes;
  int matching_threshold;
  /// @todo Keys for changing these?
  int learning_lower_bound;
  int learning_upper_bound;
  int learning_roi;
  int roi_length;
  char menue_choice;
  std::string pcl_frame_id;

  unsigned int median_filter;
  unsigned int median_counter;
  unsigned int median_classes;
  bool median_send;
  std::vector<TargetArray> median_list;

  
  double focal_length;

  // Timers
  Timer extract_timer;
  Timer match_timer;
  cv_bridge::CvImage out_msg;  
  int num_modalities;
  cv::Ptr<cv::linemod::Detector> detector;
  std::string filename;

  bool pcl_received;
  bool cont;
  bool blob;
  bool show_hull;
  bool pause;
  
  int cutX;
  int cutY;
  cv::Point cutoff[1][4];
  
  int treshold_lee;
  int treshold_classes;
  float treshold_filter;
  int treshold_counter;
  std::vector<TargetArray> treshold_list;
  std::vector<int> treshold_hit;
  std::string tarFrame;
  
public:

  bool turnOff(slaw_navigation::switchOff::Request &req, slaw_navigation::switchOff::Response &res) {
    pause=req.pause;
    res.success = true;
    return true;
  }

  
  Linemod(int argc, char * argv[])
    : it_(nh_),
      rgb_image_sub_( it_, "/camera/rgb/image_color", 1 ),
      depth_image_sub_( it_, "/camera/depth_registered/image", 1 ),
      //pcl_sub_( it_, "/camera/depth_registered/points", 1),
      //blob_image_sub_( it_, "/camera/depth/image", 1 ),
      //sync( MySyncPolicy( 10 ), rgb_image_sub_, depth_image_sub_, blob_image_sub_)
      sync( MySyncPolicy( 10 ), rgb_image_sub_, depth_image_sub_)
  {
    pause = false;
    cam_spec_sub_ = nh_.subscribe("/camera/depth_registered/camera_info", 1, &Linemod::cam_spec_cb, this);
    //cam_spec_sub_ = nh_.subscribe("/camera/rgb/camera_info", 1, &Linemod::cam_spec_cb, this); //blob
    menue_sub_ = nh_.subscribe("/vision/menue_choice", 1, &Linemod::menue_cb, this);
    tar_pub_ = nh_.advertise<TargetArray>("/vision/targets", 1);
    cam_pos_pub_ = nh_.advertise<std_msgs::String>("/cameramount/position", 1);
    img_pub_ = nh_.advertise<sensor_msgs::Image>("/cam_feed", 1);
    sync.registerCallback( boost::bind( &Linemod::callback, this, _1, _2 ) );
    //sync.registerCallback( boost::bind( &Linemod::callback, this, _1, _2, _3 ) ); ///wblob
    tarFrame = "/base_footprint";
    pcl_received = false;
    serviceOff = nh_.advertiseService("/scan_registration/switchOffRegistration", &Linemod::turnOff, this);
    
    cutX = 0;
    cutY = 0;
    cutoff[0][0] = (cv::Point(639,479));    
    cutoff[0][1] = (cv::Point(639,479-cutY));
    cutoff[0][2] = (cv::Point(639-cutX,479));
    cutoff[0][3] = (cv::Point(639,479));
    
    ros::NodeHandle param_nh_("~");
    server_mode = false;
    param_nh_.param("server", server_mode, server_mode);
    if(server_mode) 
      ROS_INFO("server_mode true");
    else 
      ROS_INFO("server_mode false");
    
    pcl_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &Linemod::pcl_cb, this);
    sleep(1);

    int threshold_size = 3;
    double threshold_dist = 0.01;
    param_nh_.param("threshold_size", threshold_size, threshold_size);
    param_nh_.param("threshold_dist", threshold_dist, threshold_dist);
    treshold_classes = -1;
    treshold_list.resize(threshold_size);
    treshold_counter = 0;
    treshold_filter = threshold_dist;
    treshold_lee = 0;
    
    median_filter = 0;
    median_list.resize(median_filter);
    median_counter = 0;
    median_send = false;
    median_classes = 0;
    
    show_hull = false;
    blob = false;
    detect_all = false;
    cont = true;
    show_match_result = false;
    show_timings = false;
    learn_online = false;
    num_classes = 0;
    menue_choice = '0';
    matching_threshold = 91;
    learning_lower_bound = 97;
    learning_upper_bound = 99;
    roi_length = 150;
    learning_roi = roi_length/10;
    
    // Initialize HighGUI
    help();
    if (!server_mode){
      cv::namedWindow("color");
      cv::namedWindow("normals");
      Mouse::start("color");
    }
    // Initialize LINEMOD data structures

    bool preload = true;
    param_nh_.param("preload", preload, preload);
	
    if (preload)
      {
	filename = "linemod_templates.yml";
	
	std::string path = ros::package::getPath("slaw_vision");
	
	filename = path +"/"+ filename;
	

	detector = readLinemod(filename);

	std::vector<std::string> ids = detector->classIds();
	num_classes = detector->numClasses();
	printf("Loaded %s with %d classes and %d templates\n",
	       "linemod_templates.yml", num_classes, detector->numTemplates());
	if (!ids.empty())
	  {
	    printf("Class ids:\n");
	    std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
	  }
      }
    else
      if (argc == 1)
	{
	  filename = "linemod_templates.yml";
	
	  std::string path = ros::package::getPath("slaw_vision");
	
	  filename = path +"/"+ filename;
	
	  detector = cv::linemod::getDefaultLINEMOD();
	}
      else
	{
	  detector = readLinemod(argv[1]);

	  std::vector<std::string> ids = detector->classIds();
	  num_classes = detector->numClasses();
	  printf("Loaded %s with %d classes and %d templates\n",
		 argv[1], num_classes, detector->numTemplates());
	  if (!ids.empty())
	    {
	      printf("Class ids:\n");
	      std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
	    }
	}
    num_modalities = (int)detector->getModalities().size();
    std_msgs::String leftMsg;
    leftMsg.data = "left";
    cam_pos_pub_.publish(leftMsg);
  }

  ~Linemod(){
  }

  void bubblesort(float *array, int length)
  {
    int i, j;
    for (i = 0; i < length -1; ++i)
      {
	
	for (j = 0; j < length - i - 1; ++j)
	  {
	    if (array[j] > array[j + 1])
	      {
		float tmp = array[j];
		array[j] = array[j + 1];
		array[j + 1] = tmp;
	      }
	  }
      }
  }
  
  
  void menue_cb(const std_msgs::Char choice){
    menue_choice = choice.data;
  }

  void cam_spec_cb(const sensor_msgs::CameraInfo info){
    focal_length = info.K[0];
    printf("Depth camera focal length: %f \n", focal_length);
    cam_spec_sub_ = nh_.subscribe("/camera/depth/camera_info_received", 1, &Linemod::cam_spec_cb, this);
  }
  
  void callback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg){
    //void callback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& blob_msg){
    if(cont && pcl_received){
      cv_bridge::CvImagePtr rgb_cv_ptr;
      cv_bridge::CvImagePtr depth_cv_ptr;
      //      cv_bridge::CvImagePtr blob_cv_ptr;//todo
      try
	{
	  rgb_cv_ptr = cv_bridge::toCvCopy(rgb_msg, std::string()); //in Bayer GRBG,  req = cv_8UC3 = CV_CAP_OPENNI_BGR_IMAGE
	  depth_cv_ptr = cv_bridge::toCvCopy(depth_msg, std::string()); //in = req = cv_16UC1 = CV_CAP_OPENNI_DEPTH_MAP
	  //	  blob_cv_ptr = cv_bridge::toCvCopy(blob_msg, std::string()); //in = req = cv_16UC1 = CV_CAP_OPENNI_DEPTH_MAP //todo
	}
      catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}
      cont = false;
      pcl_received = false;

      out_msg.header = rgb_msg->header;
      out_msg.encoding = sensor_msgs::image_encodings::BGR8;

      if (!pause)
	go(rgb_cv_ptr->image, depth_cv_ptr->image);
      //go(rgb_cv_ptr->image, depth_cv_ptr->image, blob_cv_ptr->image);
    }
  }
  
  void pcl_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud){
    if(cont && pcl_received == false) {
      cont = false;
      pcl_received = true;
      pcl_frame_id = cloud->header.frame_id; 
      pcl::fromROSMsg(*cloud, pc);
      cont = true;
    }
  }
  
  //void go(cv::Mat color, cv::Mat depth, cv::Mat blobMat) {
  void go(cv::Mat color, cv::Mat depth) {    

    const cv::Point* ppt[1] = { cutoff[0] };
    int npt[] = { 4 };

    cv::fillPoly(color, ppt, npt, 1, cv::Scalar( 255, 255, 255 ), 8);
      
    //    cv::imshow("affe", color);
    
    //    cv::imshow("depth", depth);    
    depth.convertTo(depth, CV_16UC1, 1000); // convert the image data to uint 16
        
    /// @todo Keys for changing these?
    cv::Size roi_size(roi_length, roi_length);  
    /// @todo readout..

    std::vector<cv::Mat> sources;
    sources.push_back(color);
    sources.push_back(depth);
    cv::Mat display = color.clone();

    if (!learn_online)
      {
	cv::Point mouse(Mouse::x(), Mouse::y());
	int event = Mouse::event();

	// Compute ROI centered on current mouse location
	cv::Point roi_offset(roi_size.width / 2, roi_size.height / 2);
	cv::Point pt1 = mouse - roi_offset; // top left
	cv::Point pt2 = mouse + roi_offset; // bottom right

	if (event == CV_EVENT_RBUTTONDOWN || detect_all)
	  {
	    // Compute object mask by subtracting the plane within the ROI
	    std::vector<CvPoint> chain(4);
	    if(!detect_all) {
	      chain[0] = pt1;
	      chain[1] = cv::Point(pt2.x, pt1.y);
	      chain[2] = pt2;
	      chain[3] = cv::Point(pt1.x, pt2.y);
	    }
	    else { //@todo magic numbers
	      chain[0] = cv::Point(1, 1);
	      chain[1] = cv::Point(639, 1);
	      chain[2] = cv::Point(639, 479);
	      chain[3] = cv::Point(1, 479);
	      detect_all = false;
	    }
	    cv::Mat mask;
	    subtractPlane(depth, mask, chain, focal_length);

	    if(!server_mode) cv::imshow("mask", mask);

	    // Extract template
	    std::string class_id = cv::format("class%d", num_classes);
	    cv::Rect bb;
	    extract_timer.start();
	    int template_id = detector->addTemplate(sources, class_id, mask, &bb);
	    
	    cv::rectangle(display, cv::Point(bb.x, bb.y), cv::Point(bb.x+bb.width, bb.y+bb.height), CV_RGB(255,0,255), 3);
	    sleep(0.1);

	    extract_timer.stop();
	    if (template_id != -1)
	      {
		printf("*** Added template (id %d) for new object class %d***\n",
		       template_id, num_classes);
		printf("Extracted at (%d, %d) size %dx%d\n", bb.x, bb.y, bb.width, bb.height);
	      }
	    ++num_classes;
	  }

	// Draw ROI for display
	cv::rectangle(display, pt1, pt2, CV_RGB(0,0,0), 3);
	cv::rectangle(display, pt1, pt2, CV_RGB(255,255,0), 1);
      }

    // Perform matching
    //ROS_WARN("matching");
    std::vector<cv::linemod::Match> matches;
    std::vector<std::string> class_ids;
    std::vector<cv::Mat> quantized_images;
    match_timer.start();
    detector->match(sources, (float)matching_threshold, matches, class_ids, quantized_images);
    match_timer.stop();

    //ROS_WARN("detector match done");
    
    int classes_visited = 0;
    std::set<std::string> visited;

    //ROS_WARN("creating tar msg");
    
    TargetArray target_msg;
    target_msg.header.frame_id = pcl_frame_id;
    target_msg.header.stamp = ros::Time::now();    
    target_msg.targets.resize(matches.size());
    
    if (median_filter!=0) {
      median_list[median_counter].targets.resize(matches.size());
      if (matches.size() > median_classes)
	median_classes = matches.size();
    }
    Target targets[matches.size()];

    //ROS_WARN("looking at found matches");
    
    for (int i = 0; (i < (int)matches.size()) && (classes_visited < num_classes); ++i)
      {
	cv::linemod::Match m = matches[i];

	if (visited.insert(m.class_id).second)
	  {
	    ++classes_visited;

	    if (show_match_result)
	      {
		printf("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n",
		       m.similarity, m.x, m.y, m.class_id.c_str(), m.template_id);
	      }
	    
      	    // Draw matching template
	    const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);
	    drawResponse(templates, num_modalities, display, cv::Point(m.x, m.y), detector->getT(0));

	    // Compute masks based on convex hull of matched template
	    cv::Mat color_mask, depth_mask;
	    std::vector<CvPoint> chain = maskFromTemplate(templates, num_modalities,
							  cv::Point(m.x, m.y), color.size(),
							  color_mask, display, targets[i]);

 	    targets[i].confidence = m.similarity;
	    targets[i].identifier = m.class_id.c_str();

	    float sumX=0;
	    float sumY=0;
	    
	    ros::Time now = ros::Time();	    
	    std::vector<cv::Point> contours;

	    //ROS_WARN("got hull");
	    
	    for(int m_t=0; m_t < (int)targets[i].hull.points.size(); m_t++){

	      //contour for 2d display
	      contours.push_back(cv::Point(targets[i].hull.points[m_t].x,targets[i].hull.points[m_t].y));
	      
	      sumX += targets[i].hull.points[m_t].x; 
	      sumY += targets[i].hull.points[m_t].y;
	      
	      //contour for 3d
	      pcl::PointXYZ targetPos = pc.at(targets[i].hull.points[m_t].x, targets[i].hull.points[m_t].y);
	      geometry_msgs::PointStamped pointTar;
	      pointTar.header.frame_id = pcl_frame_id;
	      pointTar.header.stamp = now;
	      pointTar.point.x = targetPos.x;
	      pointTar.point.y = targetPos.y;
	      pointTar.point.z = targetPos.z; 
	      tf_listener.transformPoint(tarFrame, pointTar, pointTar);	    
	      targets[i].hull.points[m_t].x = pointTar.point.x;
	      targets[i].hull.points[m_t].y = pointTar.point.y;
	      targets[i].hull.points[m_t].z = pointTar.point.z;
	    }

	    //ROS_WARN("contour done");
	    
	    //center
	    pcl::PointXYZ targetPos = pc.at(sumX/targets[i].hull.points.size(), sumY/targets[i].hull.points.size());	    
	    geometry_msgs::PointStamped pointTarget;
	    pointTarget.header.frame_id = pcl_frame_id;
	    pointTarget.header.stamp = now;
	    pointTarget.point.x = targetPos.x;
	    pointTarget.point.y = targetPos.y;
	    pointTarget.point.z = targetPos.z;
	    tf_listener.transformPoint(tarFrame, pointTarget, pointTarget);
	    
	    targets[i].pose.position.x = pointTarget.point.x ;
	    targets[i].pose.position.y = pointTarget.point.y;
	    targets[i].pose.position.z = pointTarget.point.z;

	    //ROS_WARN("cebter done");
	    
	    //rotated rectangle 2d
	    cv::RotatedRect outline = cv::fitEllipse(cv::Mat(contours));
	    cv::Point2f rect_points[4];
	    //	    std::vector<cv::Point> newcontours;
	    outline.points(rect_points);
	    for( int j = 0; j < 4; j++ ) {
	      cv::line( display, rect_points[j], rect_points[(j+1)%4], CV_RGB(255,255,0), 1, 8 );
	      //newcontours.push_back(rect_points[j]);
	    }
	   
	    
	    //todo debug avg depth in rect:
	    
	    // int pcl_counter = 0;
	    // float pcl_sum = 0;
	    // for (int breite = 0; breite < 639; breite++) {
	    //   for (int hoehe = 0; hoehe < 479; hoehe++) {
	    // 	int inCont = pointPolygonTest(newcontours, cv::Point2f(breite, hoehe), false);
	    // 	  if (inCont > 0) {
	    // 	    pcl::PointXYZ punkt = pc.at(breite, hoehe);
	    // 	    pcl_counter++;
	    // 	    if (!isnan(punkt.z))
	    // 	      pcl_sum += punkt.z;
	    // 	    //		    if (pcl_counter%100==0)
	    // 	    //		      ROS_ERROR("z %f", punkt.z);
	    // 	}
	    //   }
	    // }
	    // ROS_ERROR("pcl sum %f", pcl_sum);
	    // pcl_sum = pcl_sum/pcl_counter;
	    // ROS_ERROR("pcl count %d, pcl avg height: %f", pcl_counter, pcl_sum);
	    
	    
	    //ROS_WARN("rotated rect done");
	    

	    // float tpx0 = abs(rect_points[0].x-rect_points[1].x);
	    // float tpy0 = abs(rect_points[0].y-rect_points[1].y);
	    // if (tpy0 == 0) {
	    //   tpy0 = 0.0001; //*cough*
	    //   ROS_ERROR("division by zero");
	    // }
	    // cv::line(display, 
	    // 	     cv::Point(sumX/targets[i].hull.points.size(), sumY/targets[i].hull.points.size()), 
	    // 	     cv::Point(sumX/targets[i].hull.points.size()+30, (sumY/targets[i].hull.points.size() + 30*(tpx0) / (tpy0))), CV_RGB(255,0,0), 3);
	    
	    
	    //3d orientation
	    pcl::PointXYZ p0 = pc.at(rect_points[0].x, rect_points[0].y);
	    pcl::PointXYZ p1 = pc.at(rect_points[1].x, rect_points[1].y);
	    pcl::PointXYZ p2 = pc.at(rect_points[2].x, rect_points[2].y);
	    pcl::PointXYZ p3 = pc.at(rect_points[3].x, rect_points[3].y);

	    //ROS_WARN("got pcl");
	    
	    //WORST KOT EVER	    
	    try {
	      targets[i].rectangle.points.resize(4);
	    }
	    catch (const std::out_of_range& oor) {
	      std::cerr << "Out of Range error: " << oor.what() << '\n';
	    }
	    //ROS_WARN("resize done");	    
	    tf::StampedTransform myTf;
	    tf_listener.waitForTransform(pcl_frame_id, tarFrame, now, ros::Duration(0.2));
	    // tf_listener.lookupTransform(pcl_frame_id, tarFrame, now, myTf);
	    
	    geometry_msgs::PointStamped point0;
	    point0.header.frame_id = pcl_frame_id;
	    point0.header.stamp = now;
	    point0.point.x = p0.x;
	    point0.point.y = p0.y;
	    point0.point.z = p0.z; 
	    tf_listener.transformPoint(tarFrame, point0, point0);
	    //point0 = myTf * point0;

	    targets[i].rectangle.points[0].x = point0.point.x ;
	    targets[i].rectangle.points[0].y = point0.point.y ;
	    targets[i].rectangle.points[0].z = point0.point.z ;
	    //ROS_WARN("p0 done");
	    
	    geometry_msgs::PointStamped point1;
	    point1.header.frame_id = pcl_frame_id;
	    point1.header.stamp = now;
	    point1.point.x = p1.x;
	    point1.point.y = p1.y;
	    point1.point.z = p1.z; 
	    tf_listener.transformPoint(tarFrame, point1, point1);
	    targets[i].rectangle.points[1].x = point1.point.x ;
	    targets[i].rectangle.points[1].y = point1.point.y ;
	    targets[i].rectangle.points[1].z = point1.point.z ;

	    geometry_msgs::PointStamped point2;
	    point2.header.frame_id = pcl_frame_id;
	    point2.header.stamp = now;
	    point2.point.x = p2.x;
	    point2.point.y = p2.y;
	    point2.point.z = p2.z; 
	    tf_listener.transformPoint(tarFrame, point2, point2);
	    targets[i].rectangle.points[2].x = point2.point.x ;
	    targets[i].rectangle.points[2].y = point2.point.y ;
	    targets[i].rectangle.points[2].z = point2.point.z ;

	    geometry_msgs::PointStamped point3;
	    point3.header.frame_id = pcl_frame_id;
	    point3.header.stamp = now;
	    point3.point.x = p3.x;
	    point3.point.y = p3.y;
	    point3.point.z = p3.z; 
	    tf_listener.transformPoint(tarFrame, point3, point3);
	    targets[i].rectangle.points[3].x = point3.point.x ;
	    targets[i].rectangle.points[3].y = point3.point.y ;
	    targets[i].rectangle.points[3].z = point3.point.z ;

	    //ROS_WARN("3d rotated rect done");
		    
	    target_msg.header.frame_id = tarFrame;

	    // float testX = point0.point.x - point1.point.x;
	    // float testY = point0.point.y - point1.point.y;
	    // float testZ = point0.point.z - point1.point.z;

	    // Eigen::Vector3d a(point0.point.x, point0.point.y, point0.point.z);
	    // Eigen::Vector3d b(point1.point.x, point1.point.y, point1.point.z);
	    // Eigen::Quaternion<double> affe;
	    // affe = affe.setFromTwoVectors(a,b);
	   
	    // targets[i].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(tan(-testX/testY), tan(testZ/testX), tan(testY/testX));
	    // targets[i].pose.orientation.x = affe.x();
	    // targets[i].pose.orientation.y = affe.y();
	    // targets[i].pose.orientation.z = affe.z();
	    // targets[i].pose.orientation.w = affe.w();

	    if(treshold_filter>0 && treshold_counter < (int) treshold_list.size()) {
	      int ident = atoi( targets[i].identifier.substr(5, targets[i].identifier.length()-5).c_str());
	      if (ident > treshold_classes) {
		treshold_classes = ident;
		treshold_hit.resize(ident+1);
		treshold_hit[ident] = 0;
		for(int t_c = 0; t_c < (int)treshold_list.size(); t_c++) {
		  treshold_list[t_c].targets.resize(ident+1);
		  //todo link to dummy
		}
	      }
	      treshold_hit[ident] = treshold_hit[ident] +1;
	      treshold_list[treshold_counter].targets[ident] = targets[i];
	    }
	    
	    if (median_filter == 0 && treshold_filter == 0)
	      target_msg.targets[i] = targets[i];
	    else {
	      if (median_counter != median_filter) { 
		ROS_ERROR("num %d t %d", median_counter,i);
		median_list[median_counter].targets[i] = targets[i];
		if (i == (int)matches.size()-1)
		  median_counter++;
	      }
	      
	      if(median_counter == median_filter && median_filter > 0) {
		ROS_ERROR("enough samples, median classes %d", median_classes);
		median_send = true;
		median_counter = 0;
		//find x y z for num median for all objects
		//identifier
		//bool cont = true;
		//int currentId = 48; //'0'
		target_msg.targets.resize(median_classes);
		float median_x[median_classes][median_filter];
		float median_y[median_classes][median_filter];
		float median_z[median_classes][median_filter];

		for (unsigned int m_c=0; m_c<median_classes; m_c++)
		  for (unsigned int m_f=0; m_f<median_filter; m_f++) {
		    median_x[m_c][m_f]=0;
		    median_y[m_c][m_f]=0;
		    median_z[m_c][m_f]=0;
		  }
		
		//go through every element of median_list for every class.. (inefficient runtime, whatever)
		//while (cont) {
		  //std::string m_ident = "class"+(char)currentId;
		for (int m_c=0; m_c<(int) median_filter; m_c++) {
		  //bool m_found_ident = false;
		  for (int m_t=0; m_t<(int)median_list[m_c].targets.size(); m_t++) {
		    //if(median_list[m_c].targets[m_t].identifier == m_ident) {
		    //m_found_ident = true;
		    ROS_ERROR("finding ident");
		    std::string testtest = median_list[m_c].targets[m_t].identifier;
		    ROS_ERROR("got string");
		    int ident = atoi( targets[i].identifier.substr(5, targets[i].identifier.length()-5).c_str());
		    median_x[ident][m_c] = median_list[m_c].targets[m_t].pose.position.x;
		    median_y[ident][m_c] = median_list[m_c].targets[m_t].pose.position.y;
		    median_z[ident][m_c] = median_list[m_c].targets[m_t].pose.position.z;
		      //}
		  }
		  // if (!m_found_ident) {
		    //   median_x[m_c] = -999;
		    //   median_y[m_c] = -999;
		    //   median_z[m_c] = -999;
		    //sucks, take that into account..
		  //}
		}
		ROS_ERROR("collected medians");
		for (int m_b = 0; m_b < (int)median_classes; m_b++)
		  bubblesort(median_x[m_b], median_filter);
		ROS_ERROR("sorted medians");

		int num_invalid[median_classes];

		for (int n_i = 0; n_i < (int)median_filter; n_i++) {
		  for (int affe = 0; affe < (int) median_classes; affe++) {
		    if (median_x[affe][n_i] < 0.000001 && median_x[affe][n_i] > -0.000001) {
		      num_invalid[affe] = num_invalid[affe] + 1;
		      ROS_ERROR("found invalid");
		    }
		  }
		}
		ROS_ERROR("counted invalid");
		
		//bubblesort(median_y, median_filter);
		//bubblesort(median_z, median_filter);
		//find fitting in median list 
		//matching optimizable//
		for (int m_c = 0; m_c<(int)median_filter; m_c++) {
		  for (int m_t=0; m_t<(int)median_list[m_c].targets.size(); m_t++) {
		    //if (median_list[m_c].targets[m_t].identifier == m_ident) {
		    int ident = (int) median_list[m_c].targets[m_t].identifier[5] - '0';
		    ROS_ERROR("ident %d", ident);
		    int index = (median_filter/2-1+num_invalid[ident]);
		    if (index < (int)median_filter && median_x[ident][index] == median_list[m_c].targets[m_t].pose.position.x){
		      target_msg.targets[ident] = median_list[m_c].targets[m_t];
		      if (num_invalid[ident] > (int)median_filter/2-1)
			target_msg.targets[ident].confidence = 0;
		    }
		    else
		      target_msg.targets[ident].confidence = 0;
		    //}
		  }
		}
		// if (currentId-48 == median_classes)
		//   cont = false;
		//currentId++;
	      
		ROS_ERROR("samples processed");
	      }
	    }
	    
	    
	    if (learn_online == true || show_hull == true)
	      {
		/// @todo Online learning possibly broken by new gradient feature extraction,
		/// which assumes an Accurate object outline.

		if (learn_online == true) {
		  
		  outline.size.height = outline.size.height + learning_roi;
		  outline.size.width = outline.size.width + learning_roi;
		  outline.points(rect_points);
		  std::vector<CvPoint> chainT(4);
		  for(int countaer=0; countaer<4; countaer++) {
		    chainT[countaer] = cv::Point(rect_points[countaer].x, rect_points[countaer].y);
		    cv::line( display, rect_points[countaer], rect_points[(countaer+1)%4], CV_RGB(255,255,255), 1, 8 );
		    
		  }
		  
		  subtractPlane(depth, depth_mask, chainT, focal_length); //changed..

		  if (!server_mode) cv::imshow("mask", depth_mask);

		  // If pretty sure (but not TOO sure), add new template
		  if (learning_lower_bound < m.similarity && m.similarity < learning_upper_bound)
		    {
		      extract_timer.start();
		      int template_id = detector->addTemplate(sources, m.class_id, depth_mask);
		      extract_timer.stop();
		      //		      ROS_ERROR("temp id %d", template_id);
		      if (template_id != -1)
			{
			  printf("*** Added template (id %d) for existing object class %s***\n",
				 template_id, m.class_id.c_str());
			}
		    }
		}
	      }
	  }
      }
    //ROS_WARN("tc: %d", treshold_counter);
    treshold_counter++;
    if(treshold_counter == (int)treshold_list.size()) {
      TargetArray tar_msg;
      tar_msg.header.frame_id = tarFrame;
      tar_msg.header.stamp = ros::Time::now();    
      tar_msg.targets.clear();

      //ROS_INFO("filtering");
      
      for(int s=0; s<treshold_classes+1; s++) {
	//ROS_ERROR("s %d hits %d classes %d", s, treshold_hit[s], treshold_classes);
	if (treshold_hit[s] >= treshold_list.size() - treshold_lee) {
	  float distance = 0;
	  for(int t=0; t<treshold_list.size(); t++) {
	    //if dummy dont add
	    distance += pow(treshold_list[t].targets[s].pose.position.x-treshold_list[(t+1)%treshold_list.size()].targets[s].pose.position.x, 2)
			     + pow(treshold_list[t].targets[s].pose.position.y-treshold_list[(t+1)%treshold_list.size()].targets[s].pose.position.y, 2)
			     + pow(treshold_list[t].targets[s].pose.position.z-treshold_list[(t+1)%treshold_list.size()].targets[s].pose.position.z, 2);
	  }
	  distance/=treshold_list.size();
	  distance = sqrt(distance);
	  //ROS_INFO("rms distance: %f", distance);
	  if (treshold_filter>distance) {
	    //todo dont take last but median...
	    //ROS_INFO("tar okay, adding to tar msg");
	    tar_msg.targets.push_back(treshold_list[0].targets[s]);
	    //ROS_INFO("tar pushed back");
	  }
	  else
	    ;
	    //ROS_INFO("tar distance to high: %f", distance);
	}
	else
	  ;
	  //ROS_INFO("dropped because not enough hits: %d/%d", treshold_hit[s], treshold_list.size());
      }
      
      tar_pub_.publish(tar_msg);
      treshold_counter = 0;
      treshold_classes = -1;
      treshold_hit.clear();
    }
    
    if (median_filter == 0 && treshold_filter == 0)
      tar_pub_.publish(target_msg);
    else
      if (median_send) {
	tar_pub_.publish(target_msg);
	median_send = false;
      }
    if (show_match_result && matches.empty())
      printf("No matches found...\n");
    if (show_timings)
      {
	printf("Training: %.2fs\n", extract_timer.time());
	printf("Matching: %.2fs\n", match_timer.time());
      }
    if (show_match_result || show_timings)
      printf("------------------------------------------------------------\n");

    if (!server_mode) cv::imshow("color", display);


    out_msg.image = display.clone(); // Your cv::Mat
    img_pub_.publish(out_msg.toImageMsg());

    if (!server_mode) cv::imshow("normals", quantized_images[1]);

    
    if (blob) {
      // set up the parameters (check the defaults in opencv's code in blobdetector.cpp)
      cv::SimpleBlobDetector::Params params;
      params.filterByInertia = false;
      params.filterByConvexity = false;
      params.filterByColor = false;
      params.filterByCircularity = false;
      params.filterByArea = true;
      params.minArea = 200.0f;
      params.maxArea = 5000.0f;
      params.minDistBetweenBlobs = 10.0f;
      params.minRepeatability = 5;
      params.maxThreshold = 220;
      params.minThreshold = 50;
      params.thresholdStep = 10;
      //        params.blobColor
      // 	params.centroidROIMargin
      // 	params.computeRadius
      // 	params.defaultKeypointSize
      // 	params.filterByArea
      // 	params.filterByCircularity
      // 	params.filterByColor
      // 	params.filterByConvexity
      // 	params.filterByInertia
      // 	params.isGrayscaleCentroid
      // 	params.maxArea
      // 	params.maxCentersDist
      // 	params.maxThreshold
      // 	params.minArea
      // 	params.minCircularity
      // 	params.minConvexity
      // 	params.minInertiaRatio
      // 	params.minRepeatability
      // 	params.minThreshold
      // 	params.thresholdStep
      // 	// ... any other params you don't want default value

      // set up and create the detector using the parameters
      cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);
      blob_detector->create("SimpleBlob");

      //color 2 gray
      cv::Mat gr(color);
      cvtColor(gr, gr, CV_BGR2GRAY);

      cv::Mat newDepth(depth);
      double min;
      double max;
      cv::minMaxIdx(newDepth, &min, &max);
      cv::Mat adjMap;
      cv::convertScaleAbs(newDepth, adjMap, 255 / max);
   
      // cv::Mat depthMat8UC1;
      // cv::Mat depthMat16UC1(640, 480, CV_16UC1);
      // depthMat16UC1.data = ( (uchar*) blobMat.data );
      // //      depthMat16UC1.convertTo(depthMat8UC1, CV_8U, 0.05f); 
      // depthMat16UC1.convertTo(depthMat8UC1, CV_8U, 1.0/256.0); 
      // imshow("Depth Image conv", depthMat8UC1);
      // imshow("Depth Image raw", depth);
      
      // Detect!
      std::vector<cv::KeyPoint> keypoints;
      blob_detector->detect(quantized_images[1], keypoints);

      // extract the x y coordinates of the keypoints: 
      for (unsigned int i=0; i<keypoints.size(); i++){
	float X = keypoints[i].pt.x; 
	float Y = keypoints[i].pt.y;
	float size = keypoints[i].size; 
	//ROS_INFO("num %d x %f y %f", i, X, Y);
	cv::rectangle(gr, cv::Point(X-size, Y-size), cv::Point(X+size, Y+size), CV_RGB(255,0,0), 3);
      }

      cv::imshow("Gray", gr);
      cv::imshow("depth", adjMap);
      //      cvWaitKey(100);
    }


    cv::FileStorage fs;
    char key;
    if (!server_mode) {
      key = (char)cvWaitKey(10);
      if (!menue(key))
	return;
    }
    else
      if (menue_choice != '0') {
	if (!menue(menue_choice))
	  return;
	menue_choice = '0';
      }
    cont = true;

    //ROS_WARN("aux done");
  }

  bool menue(char key){
    std::vector<std::string> new_ids;

    if( key == 'q' )
      return false;

    switch (key)
      {
      case 'h':
	help();
	break;
      case 'm':
	// toggle printing match result
	show_match_result = !show_match_result;
	printf("Show match result %s\n", show_match_result ? "ON" : "OFF");
	break;
      case 't':
	// toggle printing timings
	show_timings = !show_timings;
	printf("Show timings %s\n", show_timings ? "ON" : "OFF");
	break;
      case 'l':
	// toggle online learning
	learn_online = !learn_online;
	printf("Online learning %s\n", learn_online ? "ON" : "OFF");
	break;
      case '[':
	// decrement threshold
	matching_threshold = std::max(matching_threshold - 1, -100);
	printf("New threshold: %d\n", matching_threshold);
	break;
      case ']':
	// increment threshold
	matching_threshold = std::min(matching_threshold + 1, +100);
	printf("New threshold: %d\n", matching_threshold);
	break;
      case 'w':
	// write model to disk
	writeLinemod(detector, filename);
	printf("Wrote detector and template sto %s\n", filename.c_str());
	break;
      case 'd':
	detect_all = !detect_all;
	printf("Detecting plane based %s\n", detect_all ? "ON" : "OFF");
	break;
      case 'u':
	roi_length -= 10; 	
	learning_roi = roi_length/10;
	printf("ROI length %d \n", roi_length);
	break;
      case 'i':
	roi_length += 10;
	learning_roi = roi_length/10;
	printf("ROI length %d \n", roi_length);
	break;
      case 'j':
	learning_lower_bound--;
	printf("learning lower bound %d \n", learning_lower_bound);
	break;
      case 'k':
	learning_lower_bound++;
	printf("learning lower bound %d \n", learning_lower_bound);
	break;
      case '7':
	learning_upper_bound--;
	printf("learning upper bound %d \n", learning_upper_bound);
	break;
      case '8':
	learning_upper_bound++;
	printf("learning upper bound %d \n", learning_upper_bound);
	break;
      case 'r':
	printf("Resetting..\n");
	detector = cv::linemod::getDefaultLINEMOD();
	num_classes = 0;
	num_modalities = (int)detector->getModalities().size();
	break;
      case 'o':
	cutY -= 10;
	printf("Cut off Y: %d\n", cutY);
	cutoff[0][0] = (cv::Point(639,479));    
	cutoff[0][1] = (cv::Point(639,479-cutY));
	cutoff[0][2] = (cv::Point(639-cutX,479));
	cutoff[0][3] = (cv::Point(639,479));
	break;
      case 'p':
	cutY += 10;
	printf("Cut off Y: %d\n", cutY);
	cutoff[0][0] = (cv::Point(639,479));    
	cutoff[0][1] = (cv::Point(639,479-cutY));
	cutoff[0][2] = (cv::Point(639-cutX,479));
	cutoff[0][3] = (cv::Point(639,479));
	break;
      case '9':
	cutX -= 10;
	printf("Cut off X: %d\n", cutX);
	cutoff[0][0] = (cv::Point(639,479));    
	cutoff[0][1] = (cv::Point(639,479-cutY));
	cutoff[0][2] = (cv::Point(639-cutX,479));
	cutoff[0][3] = (cv::Point(639,479));
	break;
      case '0':
	cutX += 10;
	printf("Cut off X: %d\n", cutX);
	cutoff[0][0] = (cv::Point(639,479));    
	cutoff[0][1] = (cv::Point(639,479-cutY));
	cutoff[0][2] = (cv::Point(639-cutX,479));
	cutoff[0][3] = (cv::Point(639,479));
	break;
		
      case 'e':
	detector = readLinemod(filename);
	new_ids = detector->classIds();
	num_classes = detector->numClasses();
	printf("Loaded %s with %d classes and %d templates\n",
	       filename.c_str(), num_classes, detector->numTemplates());
	if (!new_ids.empty())
	  {
	    printf("Class ids:\n");
	    std::copy(new_ids.begin(), new_ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
	  }
	num_modalities = (int)detector->getModalities().size();
	break;
      default:
	;
      }

    return true;
  }
  
  static void reprojectPoints(const std::vector<cv::Point3d>& proj, std::vector<cv::Point3d>& real, double f)
  {
    real.resize(proj.size());
    double f_inv = 1.0 / f;

    for (int i = 0; i < (int)proj.size(); ++i)
      {
	double Z = proj[i].z;
	real[i].x = (proj[i].x - 320.) * (f_inv * Z);
	real[i].y = (proj[i].y - 240.) * (f_inv * Z);
	real[i].z = Z;
      }
  }

  static void filterPlane(IplImage * ap_depth, std::vector<IplImage *> & a_masks, std::vector<CvPoint> & a_chain, double f)
  {
    const int l_num_cost_pts = 200;

    float l_thres = 4;

    IplImage * lp_mask = cvCreateImage(cvGetSize(ap_depth), IPL_DEPTH_8U, 1);
    cvSet(lp_mask, cvRealScalar(0));

    std::vector<CvPoint> l_chain_vector;

    float l_chain_length = 0;
    float * lp_seg_length = new float[a_chain.size()];

    for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i)
      {
	float x_diff = (float)(a_chain[(l_i + 1) % a_chain.size()].x - a_chain[l_i].x);
	float y_diff = (float)(a_chain[(l_i + 1) % a_chain.size()].y - a_chain[l_i].y);
	lp_seg_length[l_i] = sqrt(x_diff*x_diff + y_diff*y_diff);
	l_chain_length += lp_seg_length[l_i];
      }
    for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i)
      {
	if (lp_seg_length[l_i] > 0)
	  {
	    int l_cur_num = cvRound(l_num_cost_pts * lp_seg_length[l_i] / l_chain_length);
	    float l_cur_len = lp_seg_length[l_i] / l_cur_num;

	    for (int l_j = 0; l_j < l_cur_num; ++l_j)
	      {
		float l_ratio = (l_cur_len * l_j / lp_seg_length[l_i]);

		CvPoint l_pts;

		l_pts.x = cvRound(l_ratio * (a_chain[(l_i + 1) % a_chain.size()].x - a_chain[l_i].x) + a_chain[l_i].x);
		l_pts.y = cvRound(l_ratio * (a_chain[(l_i + 1) % a_chain.size()].y - a_chain[l_i].y) + a_chain[l_i].y);

		l_chain_vector.push_back(l_pts);
	      }
	  }
      }
    std::vector<cv::Point3d> lp_src_3Dpts(l_chain_vector.size());

    for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i)
      {
	lp_src_3Dpts[l_i].x = l_chain_vector[l_i].x;
	lp_src_3Dpts[l_i].y = l_chain_vector[l_i].y;
	lp_src_3Dpts[l_i].z = CV_IMAGE_ELEM(ap_depth, unsigned short, cvRound(lp_src_3Dpts[l_i].y), cvRound(lp_src_3Dpts[l_i].x));
	//CV_IMAGE_ELEM(lp_mask,unsigned char,(int)lp_src_3Dpts[l_i].Y,(int)lp_src_3Dpts[l_i].X)=255;
      }
    //cv_show_image(lp_mask,"hallo2");

    reprojectPoints(lp_src_3Dpts, lp_src_3Dpts, f);

    CvMat * lp_pts = cvCreateMat((int)l_chain_vector.size(), 4, CV_32F);
    CvMat * lp_v = cvCreateMat(4, 4, CV_32F);
    CvMat * lp_w = cvCreateMat(4, 1, CV_32F);

    for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i)
      {
	CV_MAT_ELEM(*lp_pts, float, l_i, 0) = (float)lp_src_3Dpts[l_i].x;
	CV_MAT_ELEM(*lp_pts, float, l_i, 1) = (float)lp_src_3Dpts[l_i].y;
	CV_MAT_ELEM(*lp_pts, float, l_i, 2) = (float)lp_src_3Dpts[l_i].z;
	CV_MAT_ELEM(*lp_pts, float, l_i, 3) = 1.0f;
      }
    cvSVD(lp_pts, lp_w, 0, lp_v);

    float l_n[4] = {CV_MAT_ELEM(*lp_v, float, 0, 3),
		    CV_MAT_ELEM(*lp_v, float, 1, 3),
		    CV_MAT_ELEM(*lp_v, float, 2, 3),
		    CV_MAT_ELEM(*lp_v, float, 3, 3)};

    float l_norm = sqrt(l_n[0] * l_n[0] + l_n[1] * l_n[1] + l_n[2] * l_n[2]);

    l_n[0] /= l_norm;
    l_n[1] /= l_norm;
    l_n[2] /= l_norm;
    l_n[3] /= l_norm;

    float l_max_dist = 0;

    for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i)
      {
	float l_dist =  l_n[0] * CV_MAT_ELEM(*lp_pts, float, l_i, 0) +
	  l_n[1] * CV_MAT_ELEM(*lp_pts, float, l_i, 1) +
	  l_n[2] * CV_MAT_ELEM(*lp_pts, float, l_i, 2) +
	  l_n[3] * CV_MAT_ELEM(*lp_pts, float, l_i, 3);

	if (fabs(l_dist) > l_max_dist)
	  l_max_dist = l_dist;
      }
    //std::cerr << "plane: " << l_n[0] << ";" << l_n[1] << ";" << l_n[2] << ";" << l_n[3] << " maxdist: " << l_max_dist << " end" << std::endl;
    int l_minx = ap_depth->width;
    int l_miny = ap_depth->height;
    int l_maxx = 0;
    int l_maxy = 0;

    for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i)
      {
	l_minx = std::min(l_minx, a_chain[l_i].x);
	l_miny = std::min(l_miny, a_chain[l_i].y);
	l_maxx = std::max(l_maxx, a_chain[l_i].x);
	l_maxy = std::max(l_maxy, a_chain[l_i].y);
      }
    int l_w = l_maxx - l_minx + 1;
    int l_h = l_maxy - l_miny + 1;
    int l_nn = (int)a_chain.size();

    CvPoint * lp_chain = new CvPoint[l_nn];

    for (int l_i = 0; l_i < l_nn; ++l_i)
      lp_chain[l_i] = a_chain[l_i];

    cvFillPoly(lp_mask, &lp_chain, &l_nn, 1, cvScalar(255, 255, 255));

    delete[] lp_chain;

    //cv::imshow("hallo1",lp_mask);

    std::vector<cv::Point3d> lp_dst_3Dpts(l_h * l_w);

    int l_ind = 0;

    for (int l_r = 0; l_r < l_h; ++l_r)
      {
	for (int l_c = 0; l_c < l_w; ++l_c)
	  {
	    lp_dst_3Dpts[l_ind].x = l_c + l_minx;
	    lp_dst_3Dpts[l_ind].y = l_r + l_miny;
	    lp_dst_3Dpts[l_ind].z = CV_IMAGE_ELEM(ap_depth, unsigned short, l_r + l_miny, l_c + l_minx);
	    ++l_ind;
	  }
      }
    reprojectPoints(lp_dst_3Dpts, lp_dst_3Dpts, f);

    l_ind = 0;

    for (int l_r = 0; l_r < l_h; ++l_r)
      {
	for (int l_c = 0; l_c < l_w; ++l_c)
	  {
	    float l_dist = (float)(l_n[0] * lp_dst_3Dpts[l_ind].x + l_n[1] * lp_dst_3Dpts[l_ind].y + lp_dst_3Dpts[l_ind].z * l_n[2] + l_n[3]);

	    ++l_ind;

	    if (CV_IMAGE_ELEM(lp_mask, unsigned char, l_r + l_miny, l_c + l_minx) != 0)
	      {
		if (fabs(l_dist) < std::max(l_thres, (l_max_dist * 2.0f)))
		  {
		    for (int l_p = 0; l_p < (int)a_masks.size(); ++l_p)
		      {
			int l_col = cvRound((l_c + l_minx) / (l_p + 1.0));
			int l_row = cvRound((l_r + l_miny) / (l_p + 1.0));

			CV_IMAGE_ELEM(a_masks[l_p], unsigned char, l_row, l_col) = 0;
		      }
		  }
		else
		  {
		    for (int l_p = 0; l_p < (int)a_masks.size(); ++l_p)
		      {
			int l_col = cvRound((l_c + l_minx) / (l_p + 1.0));
			int l_row = cvRound((l_r + l_miny) / (l_p + 1.0));

			CV_IMAGE_ELEM(a_masks[l_p], unsigned char, l_row, l_col) = 255;
		      }
		  }
	      }
	  }
      }
    cvReleaseImage(&lp_mask);
    cvReleaseMat(&lp_pts);
    cvReleaseMat(&lp_w);
    cvReleaseMat(&lp_v);
  }

  void subtractPlane(const cv::Mat& depth, cv::Mat& mask, std::vector<CvPoint>& chain, double f)
  {
    mask = cv::Mat::zeros(depth.size(), CV_8U);
    std::vector<IplImage*> tmp;
    IplImage mask_ipl = mask;
    tmp.push_back(&mask_ipl);
    IplImage depth_ipl = depth;    
    filterPlane(&depth_ipl, tmp, chain, f);
  }

  std::vector<CvPoint> maskFromTemplate(const std::vector<cv::linemod::Template>& templates,
					int num_modalities, cv::Point offset, cv::Size size,
					cv::Mat& mask, cv::Mat& dst, Target& target)
  {
    templateConvexHull(templates, num_modalities, offset, size, mask, target);

    const int OFFSET = 30;
    // cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), OFFSET);

    CvMemStorage * lp_storage = cvCreateMemStorage(0);
    CvTreeNodeIterator l_iterator;
    CvSeqReader l_reader;
    CvSeq * lp_contour = 0;

    cv::Mat mask_copy = mask.clone();
    IplImage mask_copy_ipl = mask_copy;
    cvFindContours(&mask_copy_ipl, lp_storage, &lp_contour, sizeof(CvContour),
		   CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    std::vector<CvPoint> l_pts1; // to use as input to cv_primesensor::filter_plane

    cvInitTreeNodeIterator(&l_iterator, lp_contour, 1);
    while ((lp_contour = (CvSeq *)cvNextTreeNode(&l_iterator)) != 0)
      {
	CvPoint l_pt0;
	cvStartReadSeq(lp_contour, &l_reader, 0);
	CV_READ_SEQ_ELEM(l_pt0, l_reader);
	l_pts1.push_back(l_pt0);

	for (int i = 0; i < lp_contour->total; ++i)
	  {
	    CvPoint l_pt1;
	    CV_READ_SEQ_ELEM(l_pt1, l_reader);
	    /// @todo Really need dst at all? Can just as well do this outside
	    cv::line(dst, l_pt0, l_pt1, CV_RGB(0, 255, 0), 2);

	    l_pt0 = l_pt1;
	    l_pts1.push_back(l_pt0);
	  }
      }
    cvReleaseMemStorage(&lp_storage);

    return l_pts1;
  }

  // Adapted from cv_show_angles
  cv::Mat displayQuantized(const cv::Mat& quantized)
  {
    cv::Mat color(quantized.size(), CV_8UC3);
    for (int r = 0; r < quantized.rows; ++r)
      {
	const uchar* quant_r = quantized.ptr(r);
	cv::Vec3b* color_r = color.ptr<cv::Vec3b>(r);

	for (int c = 0; c < quantized.cols; ++c)
	  {
	    cv::Vec3b& bgr = color_r[c];
	    switch (quant_r[c])
	      {
	      case 0:   bgr[0]=  0; bgr[1]=  0; bgr[2]=  0;    break;
	      case 1:   bgr[0]= 55; bgr[1]= 55; bgr[2]= 55;    break;
	      case 2:   bgr[0]= 80; bgr[1]= 80; bgr[2]= 80;    break;
	      case 4:   bgr[0]=105; bgr[1]=105; bgr[2]=105;    break;
	      case 8:   bgr[0]=130; bgr[1]=130; bgr[2]=130;    break;
	      case 16:  bgr[0]=155; bgr[1]=155; bgr[2]=155;    break;
	      case 32:  bgr[0]=180; bgr[1]=180; bgr[2]=180;    break;
	      case 64:  bgr[0]=205; bgr[1]=205; bgr[2]=205;    break;
	      case 128: bgr[0]=230; bgr[1]=230; bgr[2]=230;    break;
	      case 255: bgr[0]=  0; bgr[1]=  0; bgr[2]=255;    break;
	      default:  bgr[0]=  0; bgr[1]=255; bgr[2]=  0;    break;
	      }
	  }
      }

    return color;
  }

  // Adapted from cv_line_template::convex_hull
  void templateConvexHull(const std::vector<cv::linemod::Template>& templates,
			  int num_modalities, cv::Point offset, cv::Size size,
			  cv::Mat& dst, Target& target)
  {
    std::vector<cv::Point> points;
    for (int m = 0; m < num_modalities; ++m)
      {
	for (int i = 0; i < (int)templates[m].features.size(); ++i)
	  {
	    cv::linemod::Feature f = templates[m].features[i];
	    points.push_back(cv::Point(f.x, f.y) + offset);
	  }
      }
 
    std::vector<cv::Point> hull;
    cv::convexHull(points, hull);

    //target.hull.points.resize(hull.size());
    for (int i=0; i<hull.size(); i++) {
      geometry_msgs::Point32 point; //todo wat
      point.x = hull.at(i).x; // -offeset..;
      point.y = hull.at(i).y;// - offset.y;
      target.hull.points.push_back(point);
    }

    dst = cv::Mat::zeros(size, CV_8U);
    const int hull_count = (int)hull.size();
    const cv::Point* hull_pts = &hull[0];
    cv::fillPoly(dst, &hull_pts, &hull_count, 1, cv::Scalar(255));
  }

  void drawResponse(const std::vector<cv::linemod::Template>& templates,
		    int num_modalities, cv::Mat& dst, cv::Point offset, int T)
  {
    static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255),
					  CV_RGB(0, 255, 0),
					  CV_RGB(255, 255, 0),
					  CV_RGB(255, 140, 0),
					  CV_RGB(255, 0, 0) };

    for (int m = 0; m < num_modalities; ++m)
      {
	// NOTE: Original demo recalculated max response for each feature in the TxT
	// box around it and chose the display color based on that response. Here
	// the display color just depends on the modality.
	cv::Scalar color = COLORS[m];

	for (int i = 0; i < (int)templates[m].features.size(); ++i)
	  {
	    cv::linemod::Feature f = templates[m].features[i];
	    cv::Point pt(f.x + offset.x, f.y + offset.y);
	    cv::circle(dst, pt, T / 2, color);
	  }
      }
  }
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "linemod_server");
  Linemod lm(argc, argv);
  ros::spin();
  return 0;
}
