#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Int32MultiArray.h"
#include "zbar.h"
#include "vector"
#include "algorithm"

//height of region of interest
static int height_roi = 400;
static int width_roi = 640;

static std::string QrRegistered = "test";

typedef struct
{
  std::string type;
  std::string data;
  std::vector <cv::Point> location;
}decodedObject;

class qr_scanner{
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_trans;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    ros::Publisher pub_qrPos;
    // Create zbar scanner
    zbar::ImageScanner scanner;


public:

qr_scanner(ros::NodeHandle node_handle):image_trans(node_handle){
    // Configure scanner
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    image_sub = image_trans.subscribe("camera_module/video_stream", 1, &qr_scanner::qr_scannerCallback,this);
    image_pub = image_trans.advertise("qr_scanner/video_stream", 1);
    pub_qrPos = node_handle.advertise<std_msgs::Int32MultiArray>("qr_scanner/qr_pos", 2);
}

~qr_scanner(){
    cv::destroyAllWindows();
}

// Find and decode barcodes and QR codes
void decode(cv::Mat &im, std::vector<decodedObject>&decodedObjects)
{


  // Convert image to grayscale
  //cv::Mat imGray;
  //cvtColor(im, imGray,CV_BGR2GRAY);

  // Wrap image data in a zbar image
  zbar::Image image(im.cols, im.rows, "Y800", (uchar *)im.data, im.cols * im.rows);

  // Scan the image for barcodes and QRCodes
  int n = scanner.scan(image);

  // Print results
  for(zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
  {
    decodedObject obj;

    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();


    // Obtain location
    for(int i = 0; i< symbol->get_location_size(); i++)
    {
      obj.location.push_back(cv::Point(symbol->get_location_x(i),symbol->get_location_y(i)));
    }

    decodedObjects.push_back(obj);
  }
}

void qr_scannerCallback(const sensor_msgs::ImageConstPtr &msg)
   {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    
    //float scale = float(height)/float(cv_ptr->image.rows);
    cv::Rect roi_rect = cv::Rect((cv_ptr->image.cols - width_roi)/2, (cv_ptr->image.rows - height_roi)/2, width_roi, height_roi);
    cv::Mat roi = cv_ptr->image(roi_rect);

	int scale = 2;
    cv::Mat image;
    cv::resize(roi, roi, cv::Size(0,0),scale, scale, CV_INTER_AREA);

    cv::rectangle(cv_ptr->image, roi_rect, cv::Scalar(0,0,255), 2);
	
	cv::Mat gray_roi;
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
	cv::cvtColor(roi, gray_roi, CV_BGR2GRAY );

	cv::adaptiveThreshold(gray_roi, gray_roi, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 199, 5);
	
	cv::erode(gray_roi, gray_roi, kernel);

    std::vector<decodedObject> decodedObjects;
    decode(gray_roi, decodedObjects);

	cv::Point scalePoint = cv::Point((cv_ptr->image.cols - width_roi)/2, (cv_ptr->image.rows - height_roi)/2);

    size_t size = decodedObjects.size();

        // Loop over all decoded objects
        for(int i = 0; i < size; i++)
        { 
            if(decodedObjects[i].data == QrRegistered){

                std::vector<cv::Point> points = decodedObjects[i].location;
                std::vector<cv::Point> hull;
                
                // If the points do not form a quad, find convex hull
                if(points.size() > 4)
                    cv::convexHull(points, hull);
                else
                    hull = points;
                
                // Number of points in the convex hull
                int n = hull.size();
                
                for(int j = 0; j < n; j++)
                {
					
                    cv::line(cv_ptr->image, hull[j]/scale +scalePoint, hull[ (j+1) % n]/scale + scalePoint, cv::Scalar(255,0,0), 3);
                }

                std_msgs::Int32MultiArray pos = std_msgs::Int32MultiArray();
                pos.data = {points[2].x, points[2].y};
                pub_qrPos.publish(pos);
            }
        }
    cv::imshow("roi", gray_roi);
    cv::waitKey(1);

    image_pub.publish(cv_ptr->toImageMsg());

    
   }

};

int main(int argc,  char  **argv)
{
    ros::init(argc, argv, "qr_scanner");

    ros::param::get("qr_scanner/height_roi", height_roi);
    ros::param::get("qr_scanner/width_roi", width_roi);

    ros::NodeHandle node_handle;

    qr_scanner scan_qr = qr_scanner(node_handle);
    
    //ros::param::get("subscriber/height", height);

    ros::spin();
    

    return 0;
}
