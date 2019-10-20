#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/String.h"
#include "zbar.h"
#include "vector"
#include "algorithm"
#include "mutex"
#include "thread"


static int height = 360;

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
    std::mutex mutex_img;
    std::vector<cv::Mat> buffer_imgs;


public:

qr_scanner():image_trans(node_handle){
    image_sub = image_trans.subscribe("camera_module/video_stream", 1, &qr_scanner::qr_scannerCallback,this);
    image_pub = image_trans.advertise("qr_scanner/video_stream", 1);

}

~qr_scanner(){
    cv::destroyAllWindows();
}

void callback_buffer(const sensor_msgs::ImageConstPtr &msg){
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

    buffer_imgs.push_back(cv_ptr->image);
}

// Find and decode barcodes and QR codes
void decode(cv::Mat &im, std::vector<decodedObject>&decodedObjects)
{

  // Create zbar scanner
  zbar::ImageScanner scanner;

  // Configure scanner
  scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

  // Convert image to grayscale
  cv::Mat imGray;
  cvtColor(im, imGray,CV_BGR2GRAY);

  // Wrap image data in a zbar image
  zbar::Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);

  // Scan the image for barcodes and QRCodes
  int n = scanner.scan(image);

  // Print results
  for(zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
  {
    decodedObject obj;

    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();

    // Print type and data
    std::cout << "Type : " << obj.type << std::endl;
    std::cout << "Data : " << obj.data << std::endl << std::endl;

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
    
    float scale = float(height)/float(cv_ptr->image.rows);
    
    std::vector<decodedObject> decodedObjects;
    decode(cv_ptr->image, decodedObjects);

    size_t size = decodedObjects.size();
    if(size > 0){
        ROS_INFO("\nobjects: %zu", size);
        // Loop over all decoded objects
        for(int i = 0; i < size; i++)
        {
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
                line(cv_ptr->image, hull[j], hull[ (j+1) % n], cv::Scalar(255,0,0), 3);
            }
            
        }
    }

    image_pub.publish(cv_ptr->toImageMsg());
    
   }

};

int main(int argc,  char  **argv)
{
    ros::init(argc, argv, "qr_scanner");

    qr_scanner scan_qr = qr_scanner();
    
    //ros::param::get("subscriber/height", height);

    ros::spin();
    

    return 0;
}
