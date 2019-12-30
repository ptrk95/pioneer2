#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Bool.h"
#include "zbar.h"
#include "vector"
#include "algorithm"


//height of region of interest
int height_roi = 400;
int width_roi = 640;
float scale = 2;

bool unregistered = false;

bool show_roi = false;

std::string QrRegistered = "test";
std::string QrUnregister = "0";
std::string stream_name = "video_stream";

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
	ros::Publisher pub_reset;
    // Create zbar scanner
    zbar::ImageScanner scanner;


public:



qr_scanner(ros::NodeHandle node_handle):image_trans(node_handle){
    // Configure scanner
    std::string source_name = "camera_module/";
    source_name.append(stream_name);
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    image_sub = image_trans.subscribe(source_name, 1, &qr_scanner::qr_scannerCallback,this);
    image_pub = image_trans.advertise("qr_scanner/video_stream", 1);
    pub_qrPos = node_handle.advertise<std_msgs::Int32MultiArray>("qr_scanner/qr_pos", 2);
    pub_reset = node_handle.advertise<std_msgs::Bool>("qr_scanner/qr_reset", 1);
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
	

    cv::rectangle(cv_ptr->image, roi_rect, cv::Scalar(0,0,255), 2);
	
	
	cv::Mat gray_roi;
	//cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
	cv::cvtColor(roi, gray_roi, CV_BGR2GRAY );

	cv::adaptiveThreshold(gray_roi, gray_roi, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 111, 0);

    if(scale!=1){

    cv::resize(gray_roi, gray_roi, cv::Size(0,0),scale, scale, CV_INTER_LINEAR);
    }
	//cv::threshold(gray_roi, gray_roi, 127, 255, cv::THRESH_BINARY);
	//cv::erode(gray_roi, gray_roi, kernel);

	//cv::adaptiveThreshold(gray_roi, gray_roi, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 13, 3);
	//cv::Mat sharpeningKernel = (cv::Mat_<int>(3,3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
	//cv::Mat tmp_roi = gray_roi;
	//cv::filter2D(tmp_roi, gray_roi, -1, sharpeningKernel);
	//cv::GaussianBlur(tmp_roi, gray_roi, cv::Size(0, 0), 3);
	//cv::addWeighted(tmp_roi, 1.5, gray_roi, -0.5, 0, gray_roi);

	

    std::vector<decodedObject> decodedObjects;
    decode(gray_roi, decodedObjects);

	cv::Point scalePoint = cv::Point((cv_ptr->image.cols - width_roi)/2, (cv_ptr->image.rows - height_roi)/2);

    size_t size = decodedObjects.size();

        // Loop over all decoded objects
        for(int i = 0; i < size; i++)
        { 
			
            if(decodedObjects[i].data == QrRegistered && !unregistered){

                std::vector<cv::Point> points = decodedObjects[i].location;
                int n = points.size();
                std::vector<cv::Point> hull(n);
                std::vector<int> hull_x(n);
                std::vector<int> hull_y(n);
                
                
                // Number of points in the convex hull
                
                
                for(int j = 0; j < n; j++)
                {
		            hull[j] = points[j]/scale + scalePoint;
                    hull_x[j] = hull[j].x;
                    hull_y[j] = hull[j].y;
                    cv::line(cv_ptr->image, points[j]/scale + scalePoint, points[ (j+1) % n]/scale + scalePoint, cv::Scalar(255,0,0), 3);
                }
                
		
		//cv::Point middle_pos( hull[1].x+ ((hull[2].x-hull[1].x)/2), hull[0].y +((hull[1].y-hull[0].y)/2));
                std_msgs::Int32MultiArray pos = std_msgs::Int32MultiArray();
                pos.data = { int(*std::min_element(hull_x.begin(), hull_x.end())), int(*std::max_element(hull_x.begin(), hull_x.end())), int(*std::min_element(hull_y.begin(), hull_y.end())), int(*std::max_element(hull_y.begin(), hull_y.end()))};
                pub_qrPos.publish(pos);
            }else if(decodedObjects[i].data == QrUnregister && !unregistered){
                QrRegistered = "";
				std_msgs::Bool reset = std_msgs::Bool();
				reset.data = true;
				unregistered = true;
				pub_reset.publish(reset);
            }else if(unregistered && decodedObjects[i].data != QrUnregister){
                QrRegistered = decodedObjects[i].data;
				unregistered = false;
				std::cout << QrRegistered << std::endl;
                std_msgs::Bool reset = std_msgs::Bool();
				reset.data = false;
				pub_reset.publish(reset);
            }
        }
if(show_roi){
cv::imshow("roi", gray_roi);
    cv::waitKey(1);
}
    

    image_pub.publish(cv_ptr->toImageMsg());

    
   }

};





int main(int argc,  char  **argv)
{
    ros::init(argc, argv, "qr_scanner");

    ros::param::get("qr_scanner/height_roi", height_roi);
    ros::param::get("qr_scanner/width_roi", width_roi);
	ros::param::get("qr_scanner/scale", scale);
    ros::param::get("qr_scanner/qr_code", QrRegistered);
    ros::param::get("qr_scanner/qr_code_unregister", QrUnregister);
    ros::param::get("~show_roi", show_roi);
    ros::param::get("~stream_name", stream_name); // for multithreading
	std::cout << stream_name << std::endl;
    ros::NodeHandle node_handle;
    

    qr_scanner scan_qr = qr_scanner(node_handle);
    
    //ros::param::get("subscriber/height", height);

    ros::spin();
    

    return 0;
}
