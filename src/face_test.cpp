#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
 
using namespace cv;
using namespace std;
typedef unsigned short      UINT16, *PUINT16;

vector<Point> src;

int i = 0;
int nWidth = 512;
int nHeight = 424;
UINT16 *pBuffer = NULL;

Mat depth_image(nHeight, nWidth, CV_16UC1);
Mat DepthImage(nHeight, nWidth, CV_16UC1);

Point p;
void onMouse(int event, int x, int y, int flags, void *param)
{
    Mat *img = reinterpret_cast<Mat*>(param);
    if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标  
    {
        i++;//统计点击的次数  
        p.x = x;
        p.y = y;
        src.push_back(p);
        cout << p << static_cast<int>(img->at<unsigned short>(cv::Point(x, y))) << endl;
        cout << i << endl;
        cout << p.x << " " << p.y << " " << static_cast<int>(img->at<unsigned short>(cv::Point(x, y))) << endl;
    }
}
 
CascadeClassifier face_cascade;
 
static const std::string OPENCV_WINDOW = "Raw Image window";
 
 
class Face_Detector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber dep_image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  Face_Detector()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/image_raw", 1, 
      &Face_Detector::imageCb, this);
    dep_image_sub_ = it_.subscribe("/camera/depth/image_raw", 1, 
      &Face_Detector::depImageCb, this);
    image_pub_ = it_.advertise("/face_detector/raw_image", 1);
    cv::namedWindow(OPENCV_WINDOW);
 
  }
 
  ~Face_Detector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
 
  void depImageCb(const sensor_msgs::ImageConstPtr& msg)

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
 
    cv_bridge::CvImagePtr cv_ptr;
    namespace enc = sensor_msgs::image_encodings;
 
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
 
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600){
 
	    detect_faces(cv_ptr->image);
    	image_pub_.publish(cv_ptr->toImageMsg());
	  }
  }
  
  void detect_faces(cv::Mat img)
  {
    RNG rng( 0xFFFFFFFF );
    int lineType = 8;
    Mat frame_gray;
    cvtColor( img, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    //-- Detect faces
    std::vector<Rect> faces;
    face_cascade.detectMultiScale( frame_gray, faces );
    for ( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        ellipse( img, center, Size( faces[i].width/2, faces[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4 );
        //图像，文本，坐标，文字类型，缩放因子，颜色，线宽，线型
        //putText( img, "WARNING:Face Recognation!", org, rng.uniform(0,8),
        //     rng.uniform(0,100)*0.05+0.05, randomColor(rng), rng.uniform(1, 10), lineType);
        //putText( img, "WARNING : Face Recognation!", org, rng.uniform(0,8),
           //  1.5, Scalar(0, 255,0 ) , rng.uniform(1, 10), lineType);
        
    }
    Point org = Point(40,40);
    if (faces.size() >= 1) {
       putText( img, "WARNING : Face Recognation!", org, rng.uniform(0,8), 1.5, Scalar(0, 255,0 ) , rng.uniform(1, 10), lineType);
    }
    imshow(OPENCV_WINDOW,img);
    waitKey(3);
  }	
    static Scalar randomColor( RNG& rng )
  {
    int icolor = (unsigned) rng;
    return Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 ); //移位操作，进而生成不同的颜色
  }
 
 
};
 
 
 
int main(int argc, char** argv)
{
  //从命令行读取必要的信息,注意路径
  CommandLineParser parser(argc, argv,
                             "{help h||}"
                             "{face_cascade|/home/sf/catkin_ws/src/ros_opencv/haarcascade_frontalface_default.xml|Path to face cascade.}");
    parser.about( "\nThis program demonstrates using the cv::CascadeClassifier class to detect objects (Face + eyes) in a video stream.\n"
                  "You can use Haar or LBP features.\n\n" );
    parser.printMessage();
    String face_cascade_name = parser.get<String>("face_cascade");
 
    //-- 1. Load the cascades
    if( !face_cascade.load( face_cascade_name ) )
    {
        cout << "--(!)Error loading face cascade\n";
        return -1;
    };
  ros::init(argc, argv, "Face_Detector");
  Face_Detector ic;
  ros::spin();
  return 0;
}