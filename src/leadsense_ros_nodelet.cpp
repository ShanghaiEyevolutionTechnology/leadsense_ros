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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32.h>
#include <stereo_msgs/DisparityImage.h>
#include <image_transport/image_transport.h>

#include <dynamic_reconfigure/server.h>
#include <leadsense_ros/LeadSenseConfig.h>

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>

#include <evo_depthcamera.h>

using namespace std;

namespace leadsense_ros {

	class LeadSenseNodelet : public nodelet::Nodelet {

		evo::bino::DepthCamera camera;
		evo::bino::GrabParameters grab_param;

		ros::NodeHandle nh;
		ros::NodeHandle nh_ns;
		boost::shared_ptr<boost::thread> device_poll_thread;
		boost::shared_ptr<dynamic_reconfigure::Server<leadsense_ros::LeadSenseConfig>> server;

		image_transport::Publisher pub_left;
		image_transport::Publisher pub_right;
		ros::Publisher pub_left_cam_info;
		ros::Publisher pub_right_cam_info;
		image_transport::Publisher pub_depth;
		ros::Publisher pub_depth_cam_info;
		ros::Publisher pub_disparity;
		ros::Publisher pub_cloud;
		ros::Publisher pub_imu;
		ros::Publisher pub_imu_raw;
		ros::Publisher pub_mag;
		ros::Publisher pub_mag_raw;
		ros::Publisher pub_laser_scan;
		ros::Publisher pub_camera_height;
		ros::Publisher pub_camera_pitch;
		ros::Publisher pub_camera_roll;
		image_transport::Publisher pub_image_with_obstacle;
		image_transport::Publisher pub_image_with_ground;

		// tf
		std::string left_frame_id;
		std::string right_frame_id;
		std::string depth_frame_id;
		std::string disparity_frame_id;
		std::string cloud_frame_id;
		std::string imu_frame_id;
		std::string obstacle_detection_frame_id;

		//parameters
		int resolution_fps;
		int frame_rate;
		int camera_index = 0;
		int workmode;
		bool has_imu = false;
		float gravity = 9.8f;
		bool imu_using_9_axes = false;
		bool calc_distance = true;
		evo::imu::IMU_DATA_TYPE imu_data_type = evo::imu::IMU_DATA_TYPE_POSITION_6_AXES;
		int confidence;
		bool auto_exposure = true;
		double exposure_time;
		float max_distance;
		bool openniDepthMode = false;// 16 bit UC data in mm else 32F in m, for more info http://www.ros.org/reps/rep-0118.html
		std::string evo_file_path;
		bool obstacle_detection = false;
		int img_width, img_height;
		evo::bino::StereoParameters stereo_param;
		float view_angle_h;//view angle horizontal
		bool first_frame = true;
		ros::Time start_time_ros;
		float start_time_camera;

		template <class T>
		sensor_msgs::ImagePtr imageToROSmsg(evo::Mat<T> img, std::string frameId, ros::Time t)
		{
			sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
			sensor_msgs::Image& imgMessage = *ptr;

			imgMessage.header.stamp = t;
			imgMessage.header.frame_id = frameId;
			imgMessage.height = img.getHeight();
			imgMessage.width = img.getWidth();

			int num = 1; // for endianness detection
			imgMessage.is_bigendian = !(*(char*)&num == 1);
			
			int channels = img.getChannels();
			imgMessage.step = img.getWidth() * sizeof(T) * channels;

			size_t size = imgMessage.step * imgMessage.height;
			imgMessage.data.resize(size);

			if (sizeof(T) == sizeof(unsigned char))//unsigned char
			{
				switch (channels)
				{
				case 1:
					imgMessage.encoding = sensor_msgs::image_encodings::MONO8;
					break;
				case 3:
					imgMessage.encoding = sensor_msgs::image_encodings::RGB8;
					break;
				case 4:
					imgMessage.encoding = sensor_msgs::image_encodings::RGBA8;
					break;
				default:
					NODELET_ERROR("imageToROSmsg: mat type currently only support uchar and float, 1/3/4 channels");
					break;
				}
			}
			else if (sizeof(T) == sizeof(float))//float
			{
				switch (channels)
				{
				case 1:
					imgMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
					break;
				case 3:
					imgMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
					break;
				case 4:
					imgMessage.encoding = sensor_msgs::image_encodings::TYPE_32FC4;
					break;
				default:
					NODELET_ERROR("imageToROSmsg: mat type currently only support uchar and float, 1/3/4 channels");
					break;
				}
			}
			memcpy((char*)(&imgMessage.data[0]), img.data, size);

			return ptr;
		}


		template <class T>
		void publishImage(evo::Mat<T> img, image_transport::Publisher &pub_img, string img_frame_id, ros::Time t)
		{
			pub_img.publish(imageToROSmsg(img, img_frame_id, t));
		}

		void publishIMU(const evo::imu::IMUData imudata, const ros::Time t)
		{
			sensor_msgs::Imu msg;

			msg.header.stamp = t;
			msg.header.frame_id = imu_frame_id;

			if (imu_data_type == evo::imu::IMU_DATA_TYPE_POSITION_6_AXES)
			{
				msg.orientation.x = imudata.quaternion_6[0];
				msg.orientation.y = imudata.quaternion_6[1];
				msg.orientation.z = imudata.quaternion_6[2];
				msg.orientation.w = imudata.quaternion_6[3];
			}
			else if (imu_data_type == evo::imu::IMU_DATA_TYPE_POSITION_9_AXES)
			{
				msg.orientation.x = imudata.quaternion_9[0];
				msg.orientation.y = imudata.quaternion_9[1];
				msg.orientation.z = imudata.quaternion_9[2];
				msg.orientation.w = imudata.quaternion_9[3];
			}

			msg.orientation_covariance[0] = 0;
			msg.orientation_covariance[1] = 0;
			msg.orientation_covariance[2] = 0;

			msg.orientation_covariance[3] = 0;
			msg.orientation_covariance[4] = 0;
			msg.orientation_covariance[5] = 0;

			msg.orientation_covariance[6] = 0;
			msg.orientation_covariance[7] = 0;
			msg.orientation_covariance[8] = 0;

			msg.linear_acceleration.x = imudata.accel_calibrated[0] * gravity;
			msg.linear_acceleration.y = imudata.accel_calibrated[1] * gravity;
			msg.linear_acceleration.z = imudata.accel_calibrated[2] * gravity;

			msg.linear_acceleration_covariance[0] = 0;
			msg.linear_acceleration_covariance[1] = 0;
			msg.linear_acceleration_covariance[2] = 0;

			msg.linear_acceleration_covariance[3] = 0;
			msg.linear_acceleration_covariance[4] = 0;
			msg.linear_acceleration_covariance[5] = 0;

			msg.linear_acceleration_covariance[6] = 0;
			msg.linear_acceleration_covariance[7] = 0;
			msg.linear_acceleration_covariance[8] = 0;

			msg.angular_velocity.x = imudata.gyro_calibrated[0];
			msg.angular_velocity.y = imudata.gyro_calibrated[1];
			msg.angular_velocity.z = imudata.gyro_calibrated[2];

			msg.angular_velocity_covariance[0] = 0;
			msg.angular_velocity_covariance[1] = 0;
			msg.angular_velocity_covariance[2] = 0;

			msg.angular_velocity_covariance[3] = 0;
			msg.angular_velocity_covariance[4] = 0;
			msg.angular_velocity_covariance[5] = 0;

			msg.angular_velocity_covariance[6] = 0;
			msg.angular_velocity_covariance[7] = 0;
			msg.angular_velocity_covariance[8] = 0;

			pub_imu.publish(msg);			
		}

		void publishIMURaw(const evo::imu::IMUData imudata, const ros::Time t)
		{
			sensor_msgs::Imu msg;

			msg.header.stamp = t;
			msg.header.frame_id = imu_frame_id;

			msg.orientation_covariance[0] = -1;// http://www.ros.org/reps/rep-0145.html#topics

			msg.linear_acceleration.x = imudata.accel[0] * gravity;
			msg.linear_acceleration.y = imudata.accel[1] * gravity;
			msg.linear_acceleration.z = imudata.accel[2] * gravity;

			msg.linear_acceleration_covariance[0] = 0;
			msg.linear_acceleration_covariance[1] = 0;
			msg.linear_acceleration_covariance[2] = 0;

			msg.linear_acceleration_covariance[3] = 0;
			msg.linear_acceleration_covariance[4] = 0;
			msg.linear_acceleration_covariance[5] = 0;

			msg.linear_acceleration_covariance[6] = 0;
			msg.linear_acceleration_covariance[7] = 0;
			msg.linear_acceleration_covariance[8] = 0;

			msg.angular_velocity.x = imudata.gyro[0];
			msg.angular_velocity.y = imudata.gyro[1];
			msg.angular_velocity.z = imudata.gyro[2];

			msg.angular_velocity_covariance[0] = 0;
			msg.angular_velocity_covariance[1] = 0;
			msg.angular_velocity_covariance[2] = 0;

			msg.angular_velocity_covariance[3] = 0;
			msg.angular_velocity_covariance[4] = 0;
			msg.angular_velocity_covariance[5] = 0;

			msg.angular_velocity_covariance[6] = 0;
			msg.angular_velocity_covariance[7] = 0;
			msg.angular_velocity_covariance[8] = 0;

			pub_imu_raw.publish(msg);
		}

		void publishMag(const evo::imu::IMUData imudata, const ros::Time t)
		{
			sensor_msgs::MagneticField msg;

			msg.header.stamp = t;
			msg.header.frame_id = imu_frame_id;

			msg.magnetic_field.x = imudata.mag_calibrated[0] * 0.000001;
			msg.magnetic_field.y = imudata.mag_calibrated[1] * 0.000001;
			msg.magnetic_field.z = imudata.mag_calibrated[2] * 0.000001;

			msg.magnetic_field_covariance[0] = 0;
			msg.magnetic_field_covariance[1] = 0;
			msg.magnetic_field_covariance[2] = 0;

			msg.magnetic_field_covariance[3] = 0;
			msg.magnetic_field_covariance[4] = 0;
			msg.magnetic_field_covariance[5] = 0;

			msg.magnetic_field_covariance[6] = 0;
			msg.magnetic_field_covariance[7] = 0;
			msg.magnetic_field_covariance[8] = 0;

			pub_mag.publish(msg);
		}
		
		void publishMagRaw(const evo::imu::IMUData imudata, const ros::Time t)
		{
			sensor_msgs::MagneticField msg;

			msg.header.stamp = t;
			msg.header.frame_id = imu_frame_id;

			msg.magnetic_field.x = imudata.mag[0] * 0.000001;
			msg.magnetic_field.y = imudata.mag[1] * 0.000001;
			msg.magnetic_field.z = imudata.mag[2] * 0.000001;

			msg.magnetic_field_covariance[0] = 0;
			msg.magnetic_field_covariance[1] = 0;
			msg.magnetic_field_covariance[2] = 0;

			msg.magnetic_field_covariance[3] = 0;
			msg.magnetic_field_covariance[4] = 0;
			msg.magnetic_field_covariance[5] = 0;

			msg.magnetic_field_covariance[6] = 0;
			msg.magnetic_field_covariance[7] = 0;
			msg.magnetic_field_covariance[8] = 0;

			pub_mag_raw.publish(msg);
		}
		
		void publishLaserScan(const evo::bino::ObstacleDetectionResult od_result, const ros::Time t)
		{
			sensor_msgs::LaserScan msg;

			msg.header.stamp = t;
			msg.header.frame_id = obstacle_detection_frame_id;
			msg.angle_min = -view_angle_h / 2;
			msg.angle_max = view_angle_h / 2;
			msg.angle_increment = view_angle_h / img_width;
			msg.time_increment = 0.0;
			msg.scan_time = 1.0f / camera.getImageSizeFPS().fps;
			msg.range_min = camera.getDistanceMinValue();
			msg.range_max = camera.getDistanceMaxValue();
			msg.ranges.assign(img_width, std::numeric_limits<float>::quiet_NaN());
			//using raw result, do not filter
			for (int i = 0; i < img_width; i++)
			{
				int index = (std::atan2(i - img_width / 2, stereo_param.leftCam.focal.x) - msg.angle_min) / msg.angle_increment;
				float dist = sqrtf(od_result.obstacle3d_raw[i].x * od_result.obstacle3d_raw[i].x + od_result.obstacle3d_raw[i].z * od_result.obstacle3d_raw[i].z);
				if ((dist >= msg.range_min) && (dist <= msg.range_max))
				{
					msg.ranges.at(index) = dist;
				}
			}

			pub_laser_scan.publish(msg);
		}
		
		void publishFloat32(const float value, const ros::Publisher pub)
		{
			std_msgs::Float32 msg;

			msg.data = value;

			pub.publish(msg);
		}
		
		void publishCamInfo(sensor_msgs::CameraInfoPtr cam_info_msg, ros::Publisher pub_cam_info, ros::Time t)
		{
			static int seq = 0;
			cam_info_msg->header.stamp = t;
			cam_info_msg->header.seq = seq;
			pub_cam_info.publish(cam_info_msg);
			seq++;
		}

		void fillCamInfo(sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr right_cam_info_msg, string left_frame_id, string right_frame_id)
		{
			float baseline = stereo_param.baseline() * 0.001f; // baseline converted in meters
			NODELET_INFO("baseline: %f m", baseline);
			float fx = stereo_param.leftCam.focal.x;
			float fy = stereo_param.leftCam.focal.y;
			float cx = stereo_param.leftCam.center.x;
			float cy = stereo_param.leftCam.center.y;

			// There is no distorsions since the images are rectified
			float k1 = 0;
			float k2 = 0;
			float k3 = 0;
			float p1 = 0;
			float p2 = 0;
			NODELET_INFO("fx: %f", fx);
			NODELET_INFO("fy: %f", fy);
			NODELET_INFO("cx: %f", cx);
			NODELET_INFO("cy: %f", cy);
			NODELET_INFO("k1: %f", k1);
			NODELET_INFO("k2: %f", k2);
			NODELET_INFO("k3: %f", k3);
			NODELET_INFO("p1: %f", p1);
			NODELET_INFO("p2: %f", p2);


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

			left_cam_info_msg->width = right_cam_info_msg->width = img_width;
			left_cam_info_msg->height = right_cam_info_msg->height = img_height;

			left_cam_info_msg->header.frame_id = left_frame_id;
			right_cam_info_msg->header.frame_id = right_frame_id;
		}

		void publishDepth(evo::Mat<float> depth, image_transport::Publisher &pub_depth, string depth_frame_id, ros::Time t)
		{
			if (!openniDepthMode)
			{
				pub_depth.publish(imageToROSmsg(depth, depth_frame_id, t));
			}
			else
			{
				// OPENNI CONVERSION (meter -> millimeters - float32 -> uint16)
				sensor_msgs::ImagePtr depthMessage = boost::make_shared<sensor_msgs::Image>();

				depthMessage->header.stamp = t;
				depthMessage->header.frame_id = depth_frame_id;
				depthMessage->height = depth.getHeight();
				depthMessage->width = depth.getWidth();

				int num = 1; // for endianness detection
				depthMessage->is_bigendian = !(*(char*)&num == 1);

				depthMessage->step = depthMessage->width * sizeof(uint16_t);
				depthMessage->encoding = sensor_msgs::image_encodings::MONO16;

				size_t size = depthMessage->step * depthMessage->height;
				depthMessage->data.resize(size);

				uint16_t* data = (uint16_t*)(&depthMessage->data[0]);

				int dataSize = depthMessage->width * depthMessage->height;
				float* depthDataPtr = depth.data;

				for (int i = 0; i < dataSize; i++) {
					*(data++) = static_cast<uint16_t>(std::round(*(depthDataPtr++) * 1000));    // in mm, rounded
				}

				pub_depth.publish(depthMessage);
			}
		}

		void publishDisparity(evo::Mat<float> disparity, ros::Publisher &pub_disparity, string disparity_frame_id, ros::Time t)
		{
			sensor_msgs::ImagePtr disparity_image = imageToROSmsg(disparity, disparity_frame_id, t);

			stereo_msgs::DisparityImage msg;
			msg.image = *disparity_image;
			msg.header = msg.image.header;
			msg.f = stereo_param.leftCam.focal.x;
			msg.T = stereo_param.baseline() * 0.001f; // baseline converted in meters
			msg.min_disparity = msg.f * msg.T / camera.getDistanceMaxValue();
			msg.max_disparity = msg.f * msg.T / camera.getDistanceMinValue();

			pub_disparity.publish(msg);
		}


		void publishPointCloud(evo::Mat<float> evo_pointcloud, ros::Publisher &pub_cloud, string pointcloud_frame_id, ros::Time t)
		{
			sensor_msgs::PointCloud2 output;
			
			output.header.stamp = t;
			output.header.frame_id = pointcloud_frame_id; // Set the header values of the ROS message

			output.is_bigendian = false;
			output.is_dense = false;

			output.width = evo_pointcloud.getWidth();
			output.height = evo_pointcloud.getHeight();

			sensor_msgs::PointCloud2Modifier modifier(output);
			modifier.setPointCloud2Fields(4,
				"x", 1, sensor_msgs::PointField::FLOAT32,
				"y", 1, sensor_msgs::PointField::FLOAT32,
				"z", 1, sensor_msgs::PointField::FLOAT32,
				"rgb", 1, sensor_msgs::PointField::FLOAT32);

			// Data copy
			memcpy(&output.data[0], evo_pointcloud.data, 4 * evo_pointcloud.getWidth() * evo_pointcloud.getHeight() * sizeof(float)); // We can do a direct memcpy since data organization is the same

			// Pointcloud publishing
			pub_cloud.publish(output);
		}

		void dynamicReconfCallback(leadsense_ros::LeadSenseConfig &config, uint32_t level)
		{
			switch (level)
			{
				case 0:
					NODELET_INFO("Reconfigure confidence: %d", config.confidence);
					confidence = config.confidence;
					camera.setConfidenceThreshold((float)confidence / 100);
					break;
				case 1:
					NODELET_INFO("Reconfigure auto exposure time : %d", config.auto_exposure);
					auto_exposure = config.auto_exposure;
					camera.useAutoExposure(auto_exposure);
					break;
				case 2:
					NODELET_INFO("Old exposure time: %f, reconfigure: %f", camera.getExposureTime(), exposure_time);					
					config.auto_exposure = false;
					exposure_time = config.exposure_time;
					camera.setExposureTime(exposure_time);
					break;
				case 3:
					NODELET_INFO("Reconfigure max distance : %f", config.max_distance);
					max_distance = config.max_distance;
					camera.setDistanceMaxValue(max_distance);
					break;
				default:
					NODELET_ERROR("dynamicReconfCallback: no such level: %u", level);
					break;
				}			
		}

		ros::Time evoTimestamp2ROSTimestamp(float t)
		{
			if (first_frame)
			{
				start_time_ros = ros::Time::now();
				start_time_camera = t;
				first_frame = false;
				return start_time_ros;
			}
			else
			{
				ros::Time n = ros::Time(start_time_ros.toSec() + t - start_time_camera);
				return n;
			}
		}
		
		void open_camera()
		{
			camera.close();

			evo::RESULT_CODE res = evo::RESULT_CODE_INVALID_CAMERA;
			while (true)
			{
				if (evo_file_path.empty())
				{
					// rescan camera list, to check if camera is connected
					camera.close();
					camera.rescanAvailableCameraNames();
					res = camera.open((evo::bino::RESOLUTION_FPS_MODE)resolution_fps, camera_index, (evo::bino::WORK_MODE)workmode);
					NODELET_INFO_STREAM("camera open (live): " << result_code2str(res));
				}
				else
				{
					// evo file mode do not need rescan
					res = camera.open(evo_file_path.c_str(), (evo::bino::WORK_MODE)workmode);
					NODELET_INFO_STREAM("camera open (evo file): " << result_code2str(res));
				}
				//auto re-open
				if (res != evo::RESULT_CODE_OK)
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				}
				else
				{
					break;
				}
			}

			//check resolution & fps again
			img_width = camera.getImageSizeFPS().width;
			img_height = camera.getImageSizeFPS().height;
			frame_rate = camera.getImageSizeFPS().fps;
			NODELET_INFO("camera resolution: %d x %d, FPS: %d", img_width, img_height, frame_rate);

			//get stereo parameters
			stereo_param = camera.getStereoParameters(true);

			//set unit of measurement
			camera.setMeasureUnit(evo::bino::MEASURE_UNIT_METER);
			
			if (evo_file_path.empty())
			{
				//set auto exposure
				camera.useAutoExposure(auto_exposure);

				if (!auto_exposure)
					camera.setExposureTime(exposure_time);
			}
			
			if (calc_distance)
			{
				if (nh_ns.getParam("confidence", confidence))
						camera.setConfidenceThreshold((float)confidence / 100.0f);
				
				if (nh_ns.getParam("max_distance", max_distance))
						camera.setDistanceMaxValue(max_distance);
			}

			// Set IMU data type
			camera.setIMUDataType(imu_data_type);
			// Set IMU retrieve mode
			camera.setIMUDataRetrieveMode(evo::imu::IMU_DATA_RETRIEVE_MODE_NEWEST_IMAGE);
			// Start retrieve IMU data
			res = camera.startRetrieveIMU();
			if (res == evo::RESULT_CODE_OK)
			{
				has_imu = true;
				NODELET_INFO("IMU start OK");
			}
			else
			{
				has_imu = false;
			}
			
			// Start obstacle detection ('calc_distance' is needed)
			if (calc_distance && obstacle_detection)
			{
				res = camera.startObstacleDetection(has_imu, imu_using_9_axes);
				if (res == evo::RESULT_CODE_OK)
				{
					// Calculate view angle					
					view_angle_h = 2 * atan2(img_width * 0.5f, stereo_param.leftCam.focal.x);
					NODELET_INFO("view angle (horizontal) (rad): %f", view_angle_h);
					// Since unit is changed, reset parameter of obstacle detection
					evo::bino::ObstacleDetectionParameters od_para;
					od_para.delta = 0.0036f;
					od_para.beta = 0.0014f;
					od_para.threshold1 = 25;
					od_para.threshold2 = 37;
					od_para.obstacleMaxHeight = INFINITY;
					od_para.obstacleMinSize.x = 0.02f;//since we use unfiltered result, no use here
					od_para.obstacleMinSize.y = 0.05f;//since we use unfiltered result, no use here
					camera.setObstacleDetectionParameters(od_para);
				}
				else
				{
					view_angle_h = 0;
					NODELET_ERROR_STREAM("start obstalce detection failed, " << result_code2str(res));
				}
			}
			
			first_frame = true;
		}		

		void device_poll()
		{
			ros::Rate loop_rate(frame_rate);
			ros::Time old_t = ros::Time::now();

			// Create and fill the camera information messages
			sensor_msgs::CameraInfoPtr left_cam_info_msg(new sensor_msgs::CameraInfo());
			sensor_msgs::CameraInfoPtr right_cam_info_msg(new sensor_msgs::CameraInfo());
			sensor_msgs::CameraInfoPtr depth_cam_info_msg(new sensor_msgs::CameraInfo());
			fillCamInfo(left_cam_info_msg, right_cam_info_msg, left_frame_id, right_frame_id);
			depth_cam_info_msg = left_cam_info_msg;

			evo::Mat<unsigned char> left, right;
			evo::Mat<float> depth_map, disparity;
			evo::Mat<float> evo_pointcloud;
			std::vector<evo::imu::IMUData> vector_imu_data;
			evo::bino::ObstacleDetectionResult od_result;
			evo::Mat<unsigned char> image_with_obstacle, image_with_ground;

			// Main loop
			while (nh_ns.ok())
			{
				int left_SubNum = pub_left.getNumSubscribers();
				int right_SubNum = pub_right.getNumSubscribers();
				int depth_SubNum = pub_depth.getNumSubscribers();
				int disparity_SubNum = pub_disparity.getNumSubscribers();
				int cloud_SubNum = pub_cloud.getNumSubscribers();
				int imu_SubNum = pub_imu.getNumSubscribers();
				int imu_raw_SubNum = pub_imu_raw.getNumSubscribers();
				int mag_SubNum = pub_mag.getNumSubscribers();
				int mag_raw_SubNum = pub_mag_raw.getNumSubscribers();
				int laser_scan_SubNum = pub_laser_scan.getNumSubscribers();
				int camera_height_SubNum = pub_camera_height.getNumSubscribers();
				int camera_pitch_SubNum = pub_camera_pitch.getNumSubscribers();
				int camera_roll_SubNum = pub_camera_roll.getNumSubscribers();
				int image_with_obstalce_SubNum = pub_image_with_obstacle.getNumSubscribers();
				int image_with_ground_SubNum = pub_image_with_ground.getNumSubscribers();
				bool runloop = (left_SubNum + right_SubNum + depth_SubNum + disparity_SubNum + cloud_SubNum + imu_SubNum + imu_raw_SubNum + mag_SubNum + mag_raw_SubNum + laser_scan_SubNum + camera_height_SubNum + camera_pitch_SubNum + camera_roll_SubNum + image_with_obstalce_SubNum + image_with_ground_SubNum) > 0;

				if (runloop)
				{
					ros::Time t = ros::Time::now(); // Get current time
					evo::RESULT_CODE res = camera.grab(grab_param);
					
					if (res == evo::RESULT_CODE_END_OF_EVO_FILE)
					{
						NODELET_INFO("evo file reach end, press Ctrl+C exit...");
						camera.close();	
					}
					else if (res == evo::RESULT_CODE_INVALID_CAMERA)
					{
						NODELET_INFO("Camera disconnected? Re-openning the leadsense camera");
						open_camera();
						continue;
					}
					else if (res != evo::RESULT_CODE_OK)
					{
						NODELET_DEBUG("Wait for a new image to proceed");
						std::this_thread::sleep_for(std::chrono::milliseconds(2));
						if ((t - old_t).toSec() > 2)
						{
							NODELET_INFO("Re-openning the leadsense camera");
							open_camera();
						}
						continue;
					}
					old_t = ros::Time::now();

					ros::Time cur_image_time = evoTimestamp2ROSTimestamp(camera.getCurrentFrameTimeCode());
					if (left_SubNum > 0)
					{
						left = camera.retrieveImage(evo::bino::SIDE_LEFT, evo::MAT_TYPE_CPU);
						publishCamInfo(left_cam_info_msg, pub_left_cam_info, cur_image_time);
						publishImage(left, pub_left, left_frame_id, cur_image_time);
					}
					if (right_SubNum > 0)
					{
						right = camera.retrieveImage(evo::bino::SIDE_RIGHT, evo::MAT_TYPE_CPU);
						publishCamInfo(right_cam_info_msg, pub_right_cam_info, cur_image_time);
						publishImage(right, pub_right, right_frame_id, cur_image_time);
					}
					if (depth_SubNum > 0)
					{
						depth_map = camera.retrieveDepth(evo::bino::DEPTH_TYPE_DISTANCE_Z, evo::MAT_TYPE_CPU);
						publishCamInfo(depth_cam_info_msg, pub_depth_cam_info, cur_image_time);
						publishDepth(depth_map, pub_depth, depth_frame_id, cur_image_time);
					}
					if (disparity_SubNum > 0)
					{
						disparity = camera.retrieveDepth(evo::bino::DEPTH_TYPE_DISPARITY, evo::MAT_TYPE_CPU);
						publishDisparity(disparity, pub_disparity, disparity_frame_id, cur_image_time);
					}
					if (cloud_SubNum > 0)
					{
						evo_pointcloud = camera.retrieveDepth(evo::bino::DEPTH_TYPE_POINT_CLOUD_ORGANIZED_XYZBGRA, evo::MAT_TYPE_CPU);
						publishPointCloud(evo_pointcloud, pub_cloud, cloud_frame_id, cur_image_time);
					}
					if (imu_SubNum + imu_raw_SubNum + mag_SubNum + mag_raw_SubNum > 0)
					{
						vector_imu_data = camera.retrieveIMUData();
						for (int i = 0; i < vector_imu_data.size(); i++)
						{
							evo::imu::IMUData imu_data = vector_imu_data.at(i);
							if (imu_SubNum > 0)
							{
								publishIMU(imu_data, evoTimestamp2ROSTimestamp(imu_data.timestamp));
								ros::Duration(0.001).sleep();
							}
							if (imu_raw_SubNum > 0)
							{
								publishIMURaw(imu_data, evoTimestamp2ROSTimestamp(imu_data.timestamp));
								ros::Duration(0.001).sleep();
							}
							if (mag_SubNum > 0)
							{
								publishMag(imu_data, evoTimestamp2ROSTimestamp(imu_data.timestamp));
								ros::Duration(0.001).sleep();
							}
							if (mag_raw_SubNum > 0)
							{
								publishMagRaw(imu_data, evoTimestamp2ROSTimestamp(imu_data.timestamp));
								ros::Duration(0.001).sleep();
							}
						}
					}
					if (laser_scan_SubNum + camera_height_SubNum + camera_pitch_SubNum + camera_roll_SubNum > 0)
					{
						od_result = camera.retrieveObstacleDetectionResult();
						if (laser_scan_SubNum > 0)
						{
							publishLaserScan(od_result, cur_image_time);
						}
						if (camera_height_SubNum > 0)
						{
							publishFloat32(od_result.cameraHeight, pub_camera_height);
						}
						if (camera_pitch_SubNum > 0)
						{
							publishFloat32(od_result.cameraPitch, pub_camera_pitch);
						}
						if (camera_roll_SubNum > 0)
						{
							publishFloat32(od_result.cameraRoll, pub_camera_roll);
						}
					}
					if (image_with_obstalce_SubNum > 0)
					{
						image_with_obstacle = camera.retrieveObstacleDetectionView(evo::bino::OBSTACLE_VIEW_TYPE_IMAGE_WITH_OBSTACLE, evo::MAT_TYPE_CPU);
						publishImage(image_with_obstacle, pub_image_with_obstacle, obstacle_detection_frame_id, cur_image_time);
					}
					if (image_with_ground_SubNum > 0)
					{
						image_with_ground = camera.retrieveObstacleDetectionView(evo::bino::OBSTACLE_VIEW_TYPE_IMAGE_WITH_GROUND, evo::MAT_TYPE_CPU);
						publishImage(image_with_ground, pub_image_with_ground, obstacle_detection_frame_id, cur_image_time);
					}
										
					if (!loop_rate.sleep())
					{
						NODELET_WARN_STREAM_THROTTLE(10.0, "Actual loop time " << loop_rate.cycleTime() << " is longer than target loop time " << loop_rate.expectedCycleTime() << ". Consider to lower the 'resolution_fps' setting?");
					}
				}
				else
				{
					NODELET_WARN_THROTTLE(10.0, "No topics subscribed by user");
				}
			} // while loop
			camera.close();
		}

		void checkResolutionFPSMode()
		{
			switch (resolution_fps)
			{
				//HD800
				case evo::bino::RESOLUTION_FPS_MODE_HD800_120:
					frame_rate = 120;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_HD800_100:
					frame_rate = 100;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_HD800_60:
					frame_rate = 60;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_HD800_50:
					frame_rate = 50;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_HD800_30:
					frame_rate = 30;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_HD800_15:
					frame_rate = 15;
					break;
				//HD720
				case evo::bino::RESOLUTION_FPS_MODE_HD720_120:
					frame_rate = 120;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_HD720_100:
					frame_rate = 100;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_HD720_60:
					frame_rate = 60;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_HD720_50:
					frame_rate = 50;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_HD720_30:
					frame_rate = 30;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_HD720_15:
					frame_rate = 15;
					break;
				//SD400
				case evo::bino::RESOLUTION_FPS_MODE_SD400_120:
					frame_rate = 120;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_SD400_100:
					frame_rate = 100;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_SD400_90:
					frame_rate = 90;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_SD400_60:
					frame_rate = 60;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_SD400_30:
					frame_rate = 30;
					break;
				case evo::bino::RESOLUTION_FPS_MODE_SD400_15:
					frame_rate = 15;
					break;
				//else
				default:
					NODELET_WARN_STREAM("Wrong 'resolution_fps': " << resolution_fps << ", set to RESOLUTION_FPS_MODE_HD720_30");
					resolution_fps = evo::bino::RESOLUTION_FPS_MODE_HD720_30;
					frame_rate = 30;
					break;
			}
		}
		
		void onInit()
		{
			// Launch file parameters
			resolution_fps = evo::bino::RESOLUTION_FPS_MODE_HD720_30;
			frame_rate = 30;
			camera_index = 0;
			workmode = evo::bino::WORK_MODE_FAST;

			std::string img_topic = "image_rect_gray";
			// Set the default topic names
			string left_image_topic = "left/" + img_topic;
			string left_cam_info_topic = "left/camera_info";
			left_frame_id = "/leadsense_left_camera";

			string right_image_topic = "right/" + img_topic;
			string right_cam_info_topic = "right/camera_info";
			right_frame_id = "/leadsense_right_camera";

			// Note: Depth image frame id must match image frame id
			string depth_image_topic = "depth/";
			if (openniDepthMode)
			{
				NODELET_INFO_STREAM("Openni depth mode activated");
				depth_image_topic += "depth_raw_registered";
			}
			else
			{
				depth_image_topic += "depth_registered";
			}
			string depth_cam_info_topic = "depth/camera_info";
			depth_frame_id = left_frame_id;

			string disparity_topic = "disparity/disparity";
			disparity_frame_id = left_frame_id;

			string point_cloud_topic = "point_cloud/cloud_registered";
			cloud_frame_id = left_frame_id;

			string imu_topic = "imu/data";
			string imu_raw_topic = "imu/data_raw";
			string mag_topic = "imu/mag";
			string mag_raw_topic = "imu/mag_raw";
			imu_frame_id = "/leadsense_imu_frame";
			
			string laser_scan_topic = "left/laser_scan";
			string camera_height_topic = "left/camera_height";
			string camera_pitch_topic = "left/camera_pitch";
			string camera_roll_topic = "left/camera_roll";
			string image_with_obstacle_topic = "left/image_with_obstacle";
			string image_with_ground_topic = "left/image_with_ground";
			obstacle_detection_frame_id = "leadsense_left_laser_scan_frame";


			nh = getMTNodeHandle();
			nh_ns = getMTPrivateNodeHandle();

			// Get parameters from launch file
			nh_ns.param<std::string>("evo_filepath", evo_file_path, std::string());

			nh_ns.getParam("deviceId", camera_index);
			nh_ns.getParam("resolution_fps", resolution_fps);
			nh_ns.getParam("work_mode", workmode);

			nh_ns.getParam("left_image_topic", left_image_topic);
			nh_ns.getParam("left_cam_info_topic", left_cam_info_topic);
			nh_ns.getParam("left_frame_id", left_frame_id);

			nh_ns.getParam("right_image_topic", right_image_topic);
			nh_ns.getParam("right_cam_info_topic", right_cam_info_topic);
			nh_ns.getParam("right_frame_id", right_frame_id);

			nh_ns.getParam("depth_image_topic", depth_image_topic);
			nh_ns.getParam("depth_cam_info_topic", depth_cam_info_topic);
			nh_ns.getParam("depth_frame_id", depth_frame_id);

			nh_ns.getParam("disparity_topic", disparity_topic);
			nh_ns.getParam("disparity_frame_id", disparity_frame_id);

			nh_ns.getParam("point_cloud_topic", point_cloud_topic);
			nh_ns.getParam("cloud_frame_id", cloud_frame_id);

			nh_ns.getParam("imu_topic", imu_topic);
			nh_ns.getParam("imu_raw_topic", imu_raw_topic);
			nh_ns.getParam("mag_topic", mag_topic);
			nh_ns.getParam("mag_raw_topic", mag_raw_topic);

			nh_ns.getParam("auto_exposure", auto_exposure);
			nh_ns.getParam("exposure_time", exposure_time);
			nh_ns.getParam("gravity", gravity);
			nh_ns.getParam("imu_using_9_axes", imu_using_9_axes);
			nh_ns.getParam("calc_distance", calc_distance);
			
			nh_ns.getParam("obstacle_detection", obstacle_detection);
			nh_ns.getParam("laser_scan_topic", laser_scan_topic);
			nh_ns.getParam("camera_height_topic", camera_height_topic);
			nh_ns.getParam("camera_pitch_topic", camera_pitch_topic);
			nh_ns.getParam("camera_roll_topic", camera_roll_topic);
			nh_ns.getParam("image_with_obstacle_topic", image_with_obstacle_topic);
			nh_ns.getParam("image_with_ground_topic", image_with_ground_topic);

			checkResolutionFPSMode();
			if (imu_using_9_axes)
			{
				imu_data_type = evo::imu::IMU_DATA_TYPE_POSITION_9_AXES;
			}
			else
			{
				imu_data_type = evo::imu::IMU_DATA_TYPE_POSITION_6_AXES;
			}
			NODELET_INFO_STREAM("IMU using: " << imu_data_type2str(imu_data_type));			
			
			// Open camera
			open_camera();

			// Update grab parameters
			grab_param.do_rectify = true;
			grab_param.depth_mode = evo::bino::DEPTH_MODE_STANDARD;
			grab_param.calc_disparity = calc_distance;
			grab_param.calc_distance = calc_distance;
			NODELET_INFO_STREAM("calculate distance: " << calc_distance);

			// Reconfigure
			server = boost::make_shared<dynamic_reconfigure::Server<leadsense_ros::LeadSenseConfig>>();
			dynamic_reconfigure::Server<leadsense_ros::LeadSenseConfig>::CallbackType f;
			f = boost::bind(&LeadSenseNodelet::dynamicReconfCallback, this, _1, _2);
			server->setCallback(f);

			// Create all the publishers
			// left/right
			image_transport::ImageTransport it(nh);
			pub_left = it.advertise(left_image_topic, 1); //left
			NODELET_INFO_STREAM("Advertized on topic " << left_image_topic);
			pub_right = it.advertise(right_image_topic, 1); //right
			NODELET_INFO_STREAM("Advertized on topic " << right_image_topic);
			pub_left_cam_info = nh.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1); //left
			NODELET_INFO_STREAM("Advertized on topic " << left_cam_info_topic);
			pub_right_cam_info = nh.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic, 1); //right
			NODELET_INFO_STREAM("Advertized on topic " << right_cam_info_topic);

			// depth
			if (calc_distance)
			{
				//depth
				pub_depth = it.advertise(depth_image_topic, 1);
				NODELET_INFO_STREAM("Advertized on topic " << depth_image_topic);
				pub_depth_cam_info = nh.advertise<sensor_msgs::CameraInfo>(depth_cam_info_topic, 1);
				NODELET_INFO_STREAM("Advertized on topic " << depth_cam_info_topic);

				//disparity      
				pub_disparity = nh.advertise<stereo_msgs::DisparityImage>(disparity_topic, 1);
				NODELET_INFO_STREAM("Advertized on topic " << disparity_topic);

				//pointCloud
				pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(point_cloud_topic, 1);
				NODELET_INFO_STREAM("Advertized on topic " << point_cloud_topic);
			}

			//IMU publisher
			if (has_imu)
			{
				pub_imu = nh.advertise<sensor_msgs::Imu>(imu_topic, 1);
				NODELET_INFO_STREAM("Advertized on topic " << imu_topic);
				pub_imu_raw = nh.advertise<sensor_msgs::Imu>(imu_raw_topic, 1);
				NODELET_INFO_STREAM("Advertized on topic " << imu_raw_topic);
				if (imu_using_9_axes)
				{
					pub_mag = nh.advertise<sensor_msgs::MagneticField>(mag_topic, 1);
					NODELET_INFO_STREAM("Advertized on topic " << mag_topic);
					pub_mag_raw = nh.advertise<sensor_msgs::MagneticField>(mag_raw_topic, 1);
					NODELET_INFO_STREAM("Advertized on topic " << mag_raw_topic);
				}
			}
			
			//Obstacle detection publisher
			if (calc_distance && obstacle_detection)
			{
				//laser scan
				pub_laser_scan = nh.advertise<sensor_msgs::LaserScan>(laser_scan_topic, 1);
				NODELET_INFO_STREAM("Advertized on topic " << laser_scan_topic);
				//camera height
				pub_camera_height = nh.advertise<std_msgs::Float32>(camera_height_topic, 1);
				NODELET_INFO_STREAM("Advertized on topic " << camera_height_topic);
				//camera pitch
				pub_camera_pitch = nh.advertise<std_msgs::Float32>(camera_pitch_topic, 1);
				NODELET_INFO_STREAM("Advertized on topic " << camera_pitch_topic);
				//camera roll
				pub_camera_roll = nh.advertise<std_msgs::Float32>(camera_roll_topic, 1);
				NODELET_INFO_STREAM("Advertized on topic " << camera_roll_topic);
				//image with obstacle
				pub_image_with_obstacle = it.advertise(image_with_obstacle_topic, 1);
				NODELET_INFO_STREAM("Advertized on topic " << image_with_obstacle_topic);
				//image with ground
				pub_image_with_ground = it.advertise(image_with_ground_topic, 1);
				NODELET_INFO_STREAM("Advertized on topic " << image_with_ground_topic);
			}

			device_poll_thread = boost::shared_ptr<boost::thread>
				(new boost::thread(boost::bind(&LeadSenseNodelet::device_poll, this)));
		}
	}; // class
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(leadsense_ros::LeadSenseNodelet, nodelet::Nodelet);
