#include <csignal>
#include <cstdio>
#include <math.h>
#include <limits>
#include <thread>
#include <chrono>
#include <memory>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>
#include <leadsense_ros/LeadSenseConfig.h>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>

#include <evo_depthcamera.h>
#include <evo_mat.h>
#include <evo_global_define.h>
#include <evo_matconverter.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace evo;
using namespace bino;

namespace leadsense_ros {

    class LeadSenseNodelet : public nodelet::Nodelet {
      
        std::unique_ptr<DepthCamera> camera;
        GrabParameters grab_param;        
        
        ros::NodeHandle nh;
        ros::NodeHandle nh_ns;
        boost::shared_ptr<boost::thread> device_poll_thread;

        image_transport::Publisher pub_left;
        image_transport::Publisher pub_right;
        ros::Publisher pub_left_cam_info;
        ros::Publisher pub_right_cam_info;
        image_transport::Publisher pub_depth;
        ros::Publisher pub_depth_cam_info;
        ros::Publisher pub_cloud;

        // tf
        std::string left_frame_id;
        std::string right_frame_id;
        std::string depth_frame_id;
        std::string cloud_frame_id;
        
        Mat<float> evo_pointcloud;

        // Launch usb_stereoparameters
        int resolution_fps;//RESOLUTION_MODE_HD720_HALF
        int ros_rate;
        int camera_index = 0;
        int workmode; 
        
        //flag
        int confidence;
        bool auto_exposure=true;
        double exposure_time;
        int openniDepthMode=0;// 16 bit UC data in mm else 32F in m, for more info http://www.ros.org/reps/rep-0118.html
        
        float depth_cols_ratio;
        float depth_raws_ratio;


        /* \brief Image to ros message conversion
         * \param img : the image to publish
         * \param encodingType : the sensor_msgs::image_encodings encoding type
         * \param frameId : the id of the reference frame of the image
         * \param t : the ros::Time to stamp the image
         */
        sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t) {
            sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
            sensor_msgs::Image& imgMessage = *ptr;
            imgMessage.header.stamp = t;
            imgMessage.header.frame_id = frameId;
            imgMessage.height = img.rows;
            imgMessage.width = img.cols;
            imgMessage.encoding = encodingType;
            int num = 1; //for endianness detection
            imgMessage.is_bigendian = !(*(char *) &num == 1);
            imgMessage.step = img.cols * img.elemSize();
            size_t size = imgMessage.step * img.rows;
            imgMessage.data.resize(size);

            if (img.isContinuous())
                memcpy((char*) (&imgMessage.data[0]), img.data, size);
            else {
                uchar* opencvData = img.data;
                uchar* rosData = (uchar*) (&imgMessage.data[0]);
                for (unsigned int i = 0; i < img.rows; i++) {
                    memcpy(rosData, opencvData, imgMessage.step);
                    rosData += imgMessage.step;
                    opencvData += img.step;
                }
            }
            return ptr;
        }

        /* \brief Publish a cv::Mat image with a ros Publisher
         * \param img : the image to publish
         * \param pub_img : the publisher object to use
         * \param img_frame_id : the id of the reference frame of the image
         * \param t : the ros::Time to stamp the image
         */
        void publishImage(cv::Mat img, image_transport::Publisher &pub_img, string img_frame_id, ros::Time t) {
            pub_img.publish(imageToROSmsg(img, sensor_msgs::image_encodings::BGR8, img_frame_id, t));
        }


        /* \brief Publish the informations of a camera with a ros Publisher
         * \param cam_info_msg : the information message to publish
         * \param pub_cam_info : the publisher object to use
         * \param t : the ros::Time to stamp the message
         */
        void publishCamInfo(sensor_msgs::CameraInfoPtr cam_info_msg, ros::Publisher pub_cam_info, ros::Time t) {
            static int seq = 0;
            cam_info_msg->header.stamp = t;
            cam_info_msg->header.seq = seq;
            pub_cam_info.publish(cam_info_msg);
            seq++;
        }

        /* \brief Get the information of the usb cameras and store them in an information message
         * \param zed : the sl::zed::Camera* pointer to an instance
         * \param left_cam_info_msg : the information message to fill with the left camera informations
         * \param right_cam_info_msg : the information message to fill with the right camera informations
         * \param left_frame_id : the id of the reference frame of the left camera
         * \param right_frame_id : the id of the reference frame of the right camera
         */
        void fillCamInfo(DepthCamera* camera,sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr right_cam_info_msg,string left_frame_id, string right_frame_id) {
            cv::Mat M1, D1, M2, D2;
            cv::Mat R1, P1, R2, P2;
			bool rectified = true;        
            StereoParameters stereo_param=camera->getStereoParameters(rectified);

            int width=camera->getImageSizeFPS().width;
            int height=camera->getImageSizeFPS().height;

            NODELET_INFO("camera resolution:%d x %d",width,height);
          
            float baseline = stereo_param.T.value[0]*0.001; // baseline converted in meters
            NODELET_INFO("base_line: %f m",baseline);
            float fx = stereo_param.leftCam.focal.x;
            float fy = stereo_param.leftCam.focal.y;
            float cx = stereo_param.leftCam.center.x;
            float cy =  stereo_param.leftCam.center.y;
            
             // There is no distorsions since the images are rectified
            double k1 = 0;
            double k2 = 0;
            double k3 = 0;
            double p1 = 0;
            double p2 = 0;
            NODELET_INFO("fx: %f",fx);
            NODELET_INFO("fy: %f",fy);
            NODELET_INFO("cx: %f",cx);
            NODELET_INFO("cy: %f",cy);
            NODELET_INFO("k1: %f",k1);
            NODELET_INFO("k2: %f",k2);
            NODELET_INFO("k3: %f",k3);
            NODELET_INFO("p1: %f",p1);
            NODELET_INFO("p2: %f",p2);


            left_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            right_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

            left_cam_info_msg->D.resize(5);
            right_cam_info_msg->D.resize(5);
            left_cam_info_msg->D[0] = right_cam_info_msg->D[0] = k1;
            left_cam_info_msg->D[1] = right_cam_info_msg->D[1] = k2;
            left_cam_info_msg->D[2] = right_cam_info_msg->D[2] = k3;
            left_cam_info_msg->D[3] = right_cam_info_msg->D[3] = p1;
            left_cam_info_msg->D[4] = right_cam_info_msg->D[4] = p2;

            left_cam_info_msg->K.fill(0.0);
            right_cam_info_msg->K.fill(0.0);
            left_cam_info_msg->K[0] = right_cam_info_msg->K[0] = fx;
            left_cam_info_msg->K[2] = right_cam_info_msg->K[2] = cx;
            left_cam_info_msg->K[4] = right_cam_info_msg->K[4] = fy;
            left_cam_info_msg->K[5] = right_cam_info_msg->K[5] = cy;
            left_cam_info_msg->K[8] = right_cam_info_msg->K[8] = 1.0;

            left_cam_info_msg->R.fill(0.0);
            right_cam_info_msg->R.fill(0.0);

            left_cam_info_msg->P.fill(0.0);
            right_cam_info_msg->P.fill(0.0);
            left_cam_info_msg->P[0] = right_cam_info_msg->P[0] = fx;
            left_cam_info_msg->P[2] = right_cam_info_msg->P[2] = cx;
            left_cam_info_msg->P[5] = right_cam_info_msg->P[5] = fy;
            left_cam_info_msg->P[6] = right_cam_info_msg->P[6] = cy;
            left_cam_info_msg->P[10] = right_cam_info_msg->P[10] = 1.0;
            right_cam_info_msg->P[3] = (-1 * fx * baseline);

            left_cam_info_msg->width = right_cam_info_msg->width = width;
            left_cam_info_msg->height = right_cam_info_msg->height = height;

            left_cam_info_msg->header.frame_id = left_frame_id;
            right_cam_info_msg->header.frame_id = right_frame_id;
        }
        
        
        /* \brief Publish a cv::Mat depth image with a ros Publisher
         * \param depth : the depth image to publish
         * \param pub_depth : the publisher object to use
         * \param depth_frame_id : the id of the reference frame of the depth image
         * \param t : the ros::Time to stamp the depth image
         */
        void publishDepth(cv::Mat depth, image_transport::Publisher &pub_depth, string depth_frame_id, ros::Time t) {
            string encoding;
            if (openniDepthMode) {
                depth *= 1000.0f;
                depth.convertTo(depth, CV_16UC1); // in mm, rounded
                encoding = sensor_msgs::image_encodings::TYPE_16UC1;
            } else {
                encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            }
            pub_depth.publish(imageToROSmsg(depth, encoding, depth_frame_id, t));
        }

        
        /* \brief Publish a pcl::PointXYZRGB point cloud with a ros Publisher
         * \param pub_cloud : the publisher object to use
         * \param pointcloud_frame_id : the id of the reference frame of the point cloud
         * \param t : the ros::Time to stamp the point cloud
         */
        void publishPointCloud(ros::Publisher &pub_cloud, string pointcloud_frame_id, ros::Time t) {

            pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
            point_cloud.width=evo_pointcloud.getWidth();
            point_cloud.height=evo_pointcloud.getHeight();
            point_cloud.points.resize(evo_pointcloud.getWidth()*evo_pointcloud.getHeight());
           
           for (int m = 0; m < evo_pointcloud.getWidth() * evo_pointcloud.getHeight(); m++){
              
                point_cloud.points[m].x =  evo_pointcloud.data[m * 4 + 2 ];		//X
                point_cloud.points[m].y = -evo_pointcloud.data[m * 4     ];		//Y
                point_cloud.points[m].z = -evo_pointcloud.data[m * 4 + 1 ];		//Z
                float temp = evo_pointcloud.data[m * 4 + 3];
                uint32_t rgba = *reinterpret_cast<int*>(&temp);
                //order of BGRA in memory should be ARGB
				//uint8_t r=(rgba  )& 0x0000ff;
				//uint8_t g=(rgba >>8 )& 0x0000ff;
				//uint8_t b=(rgba >>16  )& 0x0000ff;
				//uint32_t rgb= ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                 uint32_t rgb= rgba & 0x00ffffff;
                 point_cloud.points[m].rgb = *reinterpret_cast<float*>(&rgb);
       
            }
            
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(point_cloud, output); // Convert the point cloud to a ROS message
             
            output.header.frame_id = pointcloud_frame_id; // Set the header values of the ROS message
            output.header.stamp = t;
            output.height = evo_pointcloud.getHeight();
            output.width = evo_pointcloud.getWidth();
            int num = 1; //for endianness detection
            output.is_bigendian = !(*(char *) &num == 1);
            output.is_dense = false;
            pub_cloud.publish(output);
           
        }
        void callback(leadsense_ros::LeadSenseConfig &config, uint32_t level) {
            NODELET_INFO("Reconfigure confidence : %d", config.confidence);
            confidence = config.confidence;
            auto_exposure=config.auto_exposure;
            if(!auto_exposure){
				NODELET_INFO("Reconfigure exposure time : %f", config.exposure_time);
				exposure_time=config.exposure_time;
			}
             
        }

        void device_poll() {
            ros::Rate loop_rate(ros_rate);
            ros::Time old_t = ros::Time::now();
            int width=camera->getImageSizeFPS().width;
            int height=camera->getImageSizeFPS().height;
           
            cv::Size cvSize(width, height);
          
            cv::Mat leftImage(cvSize, CV_8UC3);
            cv::Mat rightImage(cvSize, CV_8UC3);
            
            // Create and fill the camera information messages
            sensor_msgs::CameraInfoPtr left_cam_info_msg(new sensor_msgs::CameraInfo());
            sensor_msgs::CameraInfoPtr right_cam_info_msg(new sensor_msgs::CameraInfo());
            sensor_msgs::CameraInfoPtr depth_cam_info_msg(new sensor_msgs::CameraInfo());
            fillCamInfo(camera.get(),left_cam_info_msg, right_cam_info_msg, left_frame_id, right_frame_id);
            depth_cam_info_msg=left_cam_info_msg;

            Mat<unsigned char> left, right;
            Mat<float> depth_map;
          
            // Main loop
            while (nh_ns.ok()) {
                int left_SubNum=pub_left.getNumSubscribers();
                int right_SubNum=pub_right.getNumSubscribers();
                int depth_SubNum=pub_depth.getNumSubscribers();
                int cloud_SubNum=pub_cloud.getNumSubscribers();
                bool runloop=(left_SubNum+right_SubNum+depth_SubNum+cloud_SubNum)>0;
                
                if(runloop)
                {
                    int actual_confidence = (int)(camera->getConfidenceThreshold()*100);
                    if(actual_confidence!=confidence)
                    {
                    	camera->setConfidenceThreshold((float)confidence/100);
                    }
					
					camera->useAutoExposure(auto_exposure);
					float actual_exposure_time = camera->getExposureTime();
					//NODELET_INFO("actual exp:%f, set exp: %f, auto exp %d", actual_exposure_time, exposure_time, auto_exposure);
				    if((auto_exposure == false) && (abs(actual_exposure_time - exposure_time) > 0.01))
				    {
				    	camera->setExposureTime(exposure_time); 
				    }
					
					//set unit of measurement
					camera->setMeasureUnit(MEASURE_UNIT_METER);

                    ros::Time t = ros::Time::now(); // Get current time
                    RESULT_CODE err=camera->grab(grab_param);
                    if(err!=RESULT_CODE_OK)
                    {
                        NODELET_DEBUG("Wait for a new image to proceed");
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                        if ((t - old_t).toSec() > 5) 
                        {// delete the old object before constructing a new one
                            camera.reset();
                            camera.reset(new DepthCamera());
                            NODELET_INFO("Re-openning the leadsense_camera");
                            RESULT_CODE res=RESULT_CODE_INVALID_CAMERA;
                            while(res!=RESULT_CODE_OK)
                            {
                              res=camera->open((RESOLUTION_FPS_MODE)resolution_fps,camera_index,(WORK_MODE)workmode);
                                NODELET_INFO_STREAM("depth camera reopen (live): "<<result_code2str(res));
                                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                            }
        
                        }
                        continue;
                    }
                    old_t =ros::Time::now();
                    
                    if(left_SubNum>0)
                    {
                       
                        left=camera->retrieveView(VIEW_TYPE_LEFT, MAT_TYPE_CPU);
                        leftImage=evoMat2cvMat(left);
                        cv::cvtColor(leftImage, leftImage, CV_RGBA2BGR);
                        publishCamInfo(left_cam_info_msg, pub_left_cam_info, t);
                        publishImage(leftImage, pub_left, left_frame_id, t);
                      
                    }
                    if(right_SubNum>0)
                    {
                        right = camera->retrieveView(VIEW_TYPE_RIGHT, MAT_TYPE_CPU);
                        rightImage=evoMat2cvMat(right);
                        cv::cvtColor(rightImage,rightImage,CV_RGBA2BGR);
                        publishCamInfo(right_cam_info_msg, pub_right_cam_info, t);
                        publishImage(rightImage, pub_right, right_frame_id, t);
                    }
                    if(depth_SubNum>0)
                    {
                        depth_map=camera->retrieveDepth(DEPTH_TYPE_DISTANCE_Z, MAT_TYPE_CPU);
                        publishCamInfo(depth_cam_info_msg,pub_depth_cam_info,t);
                        cv::Mat cv_depth_map=evoMat2cvMat(depth_map);
                       
                        int depth_cols=cv_depth_map.cols;
                        int depth_rows=cv_depth_map.rows;
                       
                        cv::Mat depth_map_roi=cv::Mat::zeros(depth_rows,depth_cols,CV_32FC1);
                        cv_depth_map.rowRange((int)(depth_rows*depth_raws_ratio), depth_rows).colRange((int)(depth_cols*depth_cols_ratio), depth_cols-(int)(depth_cols*depth_cols_ratio)).copyTo(depth_map_roi.rowRange((int)(depth_rows*depth_raws_ratio), depth_rows).colRange((int)(depth_cols*depth_cols_ratio), depth_cols-(int)(depth_cols*depth_cols_ratio))); 
               
                        publishDepth(depth_map_roi,pub_depth,depth_frame_id,t);
                      
                    }
                    if(cloud_SubNum>0)
                    {
                        evo_pointcloud = camera->retrieveDepth(DEPTH_TYPE_POINT_CLOUD_XYZBGRA, MAT_TYPE_CPU);
                        publishPointCloud(pub_cloud,cloud_frame_id,t);                        
                    }
                    loop_rate.sleep();
                }
            } // while loop
            camera.reset();
    }

        boost::shared_ptr<dynamic_reconfigure::Server<leadsense_ros::LeadSenseConfig>> server;
        void onInit() {
            // Launch file parameters
            resolution_fps=RESOLUTION_FPS_MODE_HD720_30;
            ros_rate = 30;
            camera_index = 0;
            workmode=WORK_MODE_FAST;
          
            depth_raws_ratio=0;
            depth_cols_ratio=0;
            
            std::string img_topic = "image_rect_gray";
            // Set the default topic names
            string left_image_topic = "left/" + img_topic;
            string left_cam_info_topic = "left/camera_info";
            left_frame_id = "/leadsense_current_frame";

            string right_image_topic = "right/" + img_topic;
            string right_cam_info_topic = "right/camera_info";
            right_frame_id = "/leadsense_current_frame";
            
            string depth_image_topic = "depth/depth_registered";
            string depth_cam_info_topic = "depth/camera_info";
            depth_frame_id = "/leadsense_depth_frame";
            
            string point_cloud_topic = "point_cloud/cloud_registered";
            cloud_frame_id = "/leadsense_current_frame";


            nh = getMTNodeHandle();
            nh_ns = getMTPrivateNodeHandle();
            
            nh_ns.getParam("depth_raws_ratio",depth_raws_ratio);
            nh_ns.getParam("depth_cols_ratio",depth_cols_ratio);

            // Get parameters from launch file
            nh_ns.getParam("deviceId", camera_index );
            nh_ns.getParam("ros_rate", ros_rate);
	        nh_ns.getParam("resolution_fps",resolution_fps);
            nh_ns.getParam("work_mode",workmode);


            nh_ns.getParam("left_image_topic", left_image_topic);
            nh_ns.getParam("left_cam_info_topic", left_cam_info_topic);
            nh_ns.getParam("left_frame_id",left_frame_id);

            nh_ns.getParam("right_image_topic", right_image_topic);
            nh_ns.getParam("right_cam_info_topic", right_cam_info_topic);
            nh_ns.getParam("right_frame_id",right_frame_id);
            
            nh_ns.getParam("depth_image_topic",depth_image_topic);
            nh_ns.getParam("depth_cam_info_topic",depth_cam_info_topic);
            nh_ns.getParam("depth_frame_id",depth_frame_id);
            
            nh_ns.getParam("point_cloud_topic",point_cloud_topic);
            nh_ns.getParam("cloud_frame_id",cloud_frame_id);
            nh_ns.getParam("auto_exposure",auto_exposure);
                      
            camera.reset(new DepthCamera());
            
            RESULT_CODE res=RESULT_CODE_INVALID_CAMERA;
            while(res!=RESULT_CODE_OK)
            {
                res=camera->open( (RESOLUTION_FPS_MODE)resolution_fps,camera_index,(WORK_MODE)workmode);
                NODELET_INFO_STREAM("depth camera open (live): "<<result_code2str(res));
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            }
            if(nh_ns.getParam("confidence", confidence))
                camera->setConfidenceThreshold((float)confidence/100);
            if(nh_ns.getParam("exposure_time",exposure_time))
               camera->setExposureTime(exposure_time);
            
             //Reconfigure confidence
            server=boost::make_shared<dynamic_reconfigure::Server<leadsense_ros::LeadSenseConfig>>();
            dynamic_reconfigure::Server<leadsense_ros::LeadSenseConfig>::CallbackType f;
            f=boost::bind(&LeadSenseNodelet::callback,this,_1,_2);
            server->setCallback(f);
            if(!auto_exposure)
                camera->setExposureTime(exposure_time);
            
            // Create all the publishers
            // Image publishers
            image_transport::ImageTransport it_stereo(nh);
            pub_left = it_stereo.advertise(left_image_topic, 1); //left
            NODELET_INFO_STREAM("Advertized on topic " << left_image_topic); 
            pub_right = it_stereo.advertise(right_image_topic, 1); //right
            NODELET_INFO_STREAM("Advertized on topic " << right_image_topic);
            pub_depth=it_stereo.advertise(depth_image_topic,1);
            NODELET_INFO_STREAM("Advertized on topic "<<depth_image_topic);
            
            // Camera info publishers
            pub_left_cam_info = nh.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1); //left
            NODELET_INFO_STREAM("Advertized on topic " << left_cam_info_topic);
            pub_right_cam_info = nh.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic, 1); //right
            NODELET_INFO_STREAM("Advertized on topic " << right_cam_info_topic);
            pub_depth_cam_info = nh.advertise<sensor_msgs::CameraInfo>(depth_cam_info_topic, 1); //right
            NODELET_INFO_STREAM("Advertized on topic " << depth_cam_info_topic);
            
            //pointCloud publisher
            pub_cloud=nh.advertise<sensor_msgs::PointCloud2>(point_cloud_topic,1);
            NODELET_INFO_STREAM("Advertized on topic "<<point_cloud_topic);

            device_poll_thread = boost::shared_ptr<boost::thread>
                    (new boost::thread(boost::bind(&LeadSenseNodelet::device_poll, this)));
        }
    }; // class
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(leadsense_ros::LeadSenseNodelet, nodelet::Nodelet);
