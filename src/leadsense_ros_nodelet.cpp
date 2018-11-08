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
#include <stereo_msgs/DisparityImage.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
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

		// tf
		std::string left_frame_id;
		std::string right_frame_id;
		std::string depth_frame_id;
		std::string disparity_frame_id;
		std::string cloud_frame_id;
		std::string imu_frame_id;

		//parameters
		int resolution_fps;
		int ros_rate;
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
		int openniDepthMode = 0;// 16 bit UC data in mm else 32F in m, for more info http://www.ros.org/reps/rep-0118.html
		std::string evo_file_path;

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

			imgMessage.step = img.getWidth() * sizeof(T);

			size_t size = imgMessage.step * imgMessage.height;
			imgMessage.data.resize(size);

			if (sizeof(T) == sizeof(unsigned char))//unsigned char
			{
				switch (img.getChannels())
				{
				case 1:
					imgMessage.encoding = sensor_msgs::image_encodings::MONO8;
					break;
				case 3:
					imgMessage.encoding = sensor_msgs::image_encodings::BGR8;
					break;
				case 4:
					imgMessage.encoding = sensor_msgs::image_encodings::BGRA8;
					break;
				default:
					std::cerr << "imageToROSmsg: mat type currently only support uchar and float, 1/3/4 channels" << std::endl;
					break;
				}
			}
			else if (sizeof(T) == sizeof(float))//float
			{
				switch (img.getChannels())
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
					std::cerr << "imageToROSmsg: mat type currently only support uchar and float, 1/3/4 channels" << std::endl;
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
		
		void publishCamInfo(sensor_msgs::CameraInfoPtr cam_info_msg, ros::Publisher pub_cam_info, ros::Time t) {
			static int seq = 0;
			cam_info_msg->header.stamp = t;
			cam_info_msg->header.seq = seq;
			pub_cam_info.publish(cam_info_msg);
			seq++;
		}

		void fillCamInfo(sensor_msgs::CameraInfoPtr left_cam_info_msg, sensor_msgs::CameraInfoPtr right_cam_info_msg, string left_frame_id, string right_frame_id) {
			evo::bino::StereoParameters stereo_param = camera.getStereoParameters(true);

			int width = camera.getImageSizeFPS().width;
			int height = camera.getImageSizeFPS().height;

			NODELET_INFO("camera resolution: %d x %d", width, height);

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

			left_cam_info_msg->width = right_cam_info_msg->width = width;
			left_cam_info_msg->height = right_cam_info_msg->height = height;

			left_cam_info_msg->header.frame_id = left_frame_id;
			right_cam_info_msg->header.frame_id = right_frame_id;
		}

		void publishDepth(evo::Mat<float> depth, image_transport::Publisher &pub_depth, string depth_frame_id, ros::Time t) {
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

		void publishDisparity(evo::Mat<float> disparity, ros::Publisher &pub_disparity, string disparity_frame_id, ros::Time t) {
			evo::bino::StereoParameters stereo_param = camera.getStereoParameters(true);

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


		void publishPointCloud(evo::Mat<float> evo_pointcloud, ros::Publisher &pub_cloud, string pointcloud_frame_id, ros::Time t) {
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
					NODELET_INFO("actual exposure time: %f, set: %f", camera.getExposureTime(), exposure_time);					
					config.auto_exposure = false;
					exposure_time = config.exposure_time;
					camera.setExposureTime(exposure_time);
					break;
				case 3:
					NODELET_INFO("set max distance : %f", config.max_distance);
					max_distance = config.max_distance;
					camera.setDistanceMaxValue(max_distance);
					break;
				default:
					std::cerr << "dynamicReconfCallback: no such level: " << level << std::endl;
					break;
				}			
		}

		ros::Time evoTimestamp2ROSTimestamp(float t)
		{
			uint32_t sec = (uint32_t)t;
			uint32_t nsec = (uint32_t)((t - sec) * 1000000000);
			return ros::Time(sec, nsec);
		}

		void device_poll() {
			ros::Rate loop_rate(ros_rate);
			ros::Time old_t = ros::Time::now();
			int width = camera.getImageSizeFPS().width;
			int height = camera.getImageSizeFPS().height;

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

			// Main loop
			while (nh_ns.ok()) {
				int left_SubNum = pub_left.getNumSubscribers();
				int right_SubNum = pub_right.getNumSubscribers();
				int depth_SubNum = pub_depth.getNumSubscribers();
				int disparity_SubNum = pub_disparity.getNumSubscribers();
				int cloud_SubNum = pub_cloud.getNumSubscribers();
				int imu_SubNum = pub_imu.getNumSubscribers();
				int imu_raw_SubNum = pub_imu_raw.getNumSubscribers();
				int mag_SubNum = pub_mag.getNumSubscribers();
				bool runloop = (left_SubNum + right_SubNum + depth_SubNum + disparity_SubNum + cloud_SubNum + imu_SubNum + imu_raw_SubNum + mag_SubNum) > 0;

				if (runloop)
				{
					ros::Time t = ros::Time::now(); // Get current time
					evo::RESULT_CODE res = camera.grab(grab_param);
					
					if (res == evo::RESULT_CODE_END_OF_EVO_FILE)
					{
						NODELET_INFO_STREAM("evo file reach end, press Ctrl+C exit...");
						camera.close();	
					}
					else if (res != evo::RESULT_CODE_OK)
					{
						NODELET_DEBUG("Wait for a new image to proceed");
						std::this_thread::sleep_for(std::chrono::milliseconds(2));
						if ((t - old_t).toSec() > 5)
						{
							camera.close();
							NODELET_INFO("Re-openning the leadsense camera");
							res = evo::RESULT_CODE_INVALID_CAMERA;
							while (res != evo::RESULT_CODE_OK)
							{
								res = camera.open((evo::bino::RESOLUTION_FPS_MODE)resolution_fps, camera_index, (evo::bino::WORK_MODE)workmode);
								NODELET_INFO_STREAM("depth camera reopen (live): " << result_code2str(res));
								std::this_thread::sleep_for(std::chrono::milliseconds(2000));
							}

							//set unit of measurement
							camera.setMeasureUnit(evo::bino::MEASURE_UNIT_METER);
							//set auto exposure
							camera.useAutoExposure(auto_exposure);
							//set exposure time
							if (!auto_exposure)
							{
								camera.setExposureTime(exposure_time);
							}

							//set max distance
							camera.setDistanceMaxValue(max_distance);

							// Set IMU data type
							camera.setIMUDataType(imu_data_type);
							// Set IMU retrieve mode
							camera.setIMUDataRetrieveMode(evo::imu::IMU_DATA_RETRIEVE_MODE_NEWEST_IMAGE);
							// Start retrieve IMU data
							res = camera.startRetrieveIMU();
							if (res == evo::RESULT_CODE_OK)
							{
								has_imu = true;
							}
							else
							{
								has_imu = false;
							}
						}
						continue;
					}
					old_t = ros::Time::now();

					if (left_SubNum > 0)
					{
						left = camera.retrieveImage(evo::bino::SIDE_LEFT, evo::MAT_TYPE_CPU);
						publishCamInfo(left_cam_info_msg, pub_left_cam_info, evoTimestamp2ROSTimestamp(camera.getCurrentFrameTimeCode()));
						publishImage(left, pub_left, left_frame_id, evoTimestamp2ROSTimestamp(camera.getCurrentFrameTimeCode()));

					}
					if (right_SubNum > 0)
					{
						right = camera.retrieveImage(evo::bino::SIDE_RIGHT, evo::MAT_TYPE_CPU);
						publishCamInfo(right_cam_info_msg, pub_right_cam_info, evoTimestamp2ROSTimestamp(camera.getCurrentFrameTimeCode()));
						publishImage(right, pub_right, right_frame_id, evoTimestamp2ROSTimestamp(camera.getCurrentFrameTimeCode()));
					}
					if (depth_SubNum > 0)
					{
						depth_map = camera.retrieveDepth(evo::bino::DEPTH_TYPE_DISTANCE_Z, evo::MAT_TYPE_CPU);
						publishCamInfo(depth_cam_info_msg, pub_depth_cam_info, evoTimestamp2ROSTimestamp(camera.getCurrentFrameTimeCode()));
						publishDepth(depth_map, pub_depth, depth_frame_id, evoTimestamp2ROSTimestamp(camera.getCurrentFrameTimeCode()));
					}
					if (disparity_SubNum > 0)
					{
						disparity = camera.retrieveDepth(evo::bino::DEPTH_TYPE_DISPARITY, evo::MAT_TYPE_CPU);
						publishDisparity(disparity, pub_disparity, disparity_frame_id, evoTimestamp2ROSTimestamp(camera.getCurrentFrameTimeCode()));
					}
					if (cloud_SubNum > 0)
					{
						evo_pointcloud = camera.retrieveDepth(evo::bino::DEPTH_TYPE_POINT_CLOUD_XYZBGRA, evo::MAT_TYPE_CPU);
						publishPointCloud(evo_pointcloud, pub_cloud, cloud_frame_id, evoTimestamp2ROSTimestamp(camera.getCurrentFrameTimeCode()));
					}
					if (imu_SubNum > 0)
					{
						vector_imu_data = camera.retrieveIMUData();
						for (int i = 0; i < vector_imu_data.size(); i++)
						{
							evo::imu::IMUData imu_data = vector_imu_data.at(i);
							publishIMU(imu_data, evoTimestamp2ROSTimestamp(imu_data.timestamp));
							ros::Duration(0.001).sleep();
						}
					}
					if (imu_raw_SubNum > 0)
					{
						vector_imu_data = camera.retrieveIMUData();
						for (int i = 0; i < vector_imu_data.size(); i++)
						{
							evo::imu::IMUData imu_data = vector_imu_data.at(i);
							publishIMURaw(imu_data, evoTimestamp2ROSTimestamp(imu_data.timestamp));
							ros::Duration(0.001).sleep();
						}
					}
					if (mag_SubNum > 0)
					{
						vector_imu_data = camera.retrieveIMUData();
						for (int i = 0; i < vector_imu_data.size(); i++)
						{
							evo::imu::IMUData imu_data = vector_imu_data.at(i);
							publishMag(imu_data, evoTimestamp2ROSTimestamp(imu_data.timestamp));
							ros::Duration(0.001).sleep();
						}
					}
					loop_rate.sleep();
				}
			} // while loop
			camera.close();
		}

		boost::shared_ptr<dynamic_reconfigure::Server<leadsense_ros::LeadSenseConfig>> server;
		void onInit() {
			// Launch file parameters
			resolution_fps = evo::bino::RESOLUTION_FPS_MODE_HD720_30;
			ros_rate = 30;
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
			string depth_image_topic = "depth/depth_registered";
			string depth_cam_info_topic = "depth/camera_info";
			depth_frame_id = left_frame_id;

			string disparity_topic = "disparity/disparity";
			disparity_frame_id = left_frame_id;

			string point_cloud_topic = "point_cloud/cloud_registered";
			cloud_frame_id = left_frame_id;

			string imu_topic = "imu/data";
			string imu_raw_topic = "imu/data_raw";
			string mag_topic = "imu/mag";
			imu_frame_id = "/leadsense_imu_frame";


			nh = getMTNodeHandle();
			nh_ns = getMTPrivateNodeHandle();

			// Get parameters from launch file
			nh_ns.getParam("deviceId", camera_index);
			nh_ns.getParam("ros_rate", ros_rate);
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

			nh_ns.getParam("auto_exposure", auto_exposure);
			nh_ns.getParam("exposure_time", exposure_time);
			nh_ns.getParam("gravity", gravity);
			nh_ns.getParam("imu_using_9_axes", imu_using_9_axes);
			nh_ns.getParam("calc_distance", calc_distance);

			if (imu_using_9_axes)
			{
				imu_data_type = evo::imu::IMU_DATA_TYPE_POSITION_9_AXES;
			}
			else
			{
				imu_data_type = evo::imu::IMU_DATA_TYPE_POSITION_6_AXES;
			}
			NODELET_INFO_STREAM("IMU using: " << imu_data_type2str(imu_data_type));
			
			
			nh_ns.param<std::string>("evo_filepath", evo_file_path, std::string());


			camera.close();

			evo::RESULT_CODE res = evo::RESULT_CODE_INVALID_CAMERA;
			while (res != evo::RESULT_CODE_OK)
			{
				if (evo_file_path.empty())
				{
					res = camera.open((evo::bino::RESOLUTION_FPS_MODE)resolution_fps, camera_index, (evo::bino::WORK_MODE)workmode);
					NODELET_INFO_STREAM("depth camera open (live): " << result_code2str(res));
				}
				else
				{
					res = camera.open(evo_file_path.c_str(), (evo::bino::WORK_MODE)workmode);
					NODELET_INFO_STREAM("depth camera open (evo file): " << result_code2str(res));
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(2000));
			}

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
			}
			else
			{
				has_imu = false;
			}

			// Update grab parameters
			grab_param.do_rectify = true;
			grab_param.depth_mode = evo::bino::DEPTH_MODE_STANDARD;
			grab_param.calc_disparity = calc_distance;
			grab_param.calc_distance = calc_distance;
			NODELET_INFO_STREAM("calculate distance: " << calc_distance);

			//Reconfigure
			server = boost::make_shared<dynamic_reconfigure::Server<leadsense_ros::LeadSenseConfig>>();
			dynamic_reconfigure::Server<leadsense_ros::LeadSenseConfig>::CallbackType f;
			f = boost::bind(&LeadSenseNodelet::dynamicReconfCallback, this, _1, _2);
			server->setCallback(f);

			// Create all the publishers
			// left/right
			image_transport::ImageTransport it_stereo(nh);
			pub_left = it_stereo.advertise(left_image_topic, 1); //left
			NODELET_INFO_STREAM("Advertized on topic " << left_image_topic);
			pub_right = it_stereo.advertise(right_image_topic, 1); //right
			NODELET_INFO_STREAM("Advertized on topic " << right_image_topic);
			pub_left_cam_info = nh.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1); //left
			NODELET_INFO_STREAM("Advertized on topic " << left_cam_info_topic);
			pub_right_cam_info = nh.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic, 1); //right
			NODELET_INFO_STREAM("Advertized on topic " << right_cam_info_topic);

			// depth
			if (calc_distance)
			{
				pub_depth = it_stereo.advertise(depth_image_topic, 1); //depth
				NODELET_INFO_STREAM("Advertized on topic " << depth_image_topic);
				pub_depth_cam_info = nh.advertise<sensor_msgs::CameraInfo>(depth_cam_info_topic, 1); //depth
				NODELET_INFO_STREAM("Advertized on topic " << depth_cam_info_topic);

				//disparity      
				pub_disparity = nh.advertise<stereo_msgs::DisparityImage>(disparity_topic, 1); //disparity
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
				}
			}

			device_poll_thread = boost::shared_ptr<boost::thread>
				(new boost::thread(boost::bind(&LeadSenseNodelet::device_poll, this)));
		}
	}; // class
} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(leadsense_ros::LeadSenseNodelet, nodelet::Nodelet);
