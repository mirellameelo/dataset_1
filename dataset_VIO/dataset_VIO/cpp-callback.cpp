// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp>
#include <thread>
#include <iostream>
#include <iomanip>
#include <typeinfo>
#include <map>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <iomanip>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../third_party/stb_image_write.h"

std::mutex mm;
std::vector<std::string> logs;
std::vector<std::string> fotos;
std::ofstream IMU;
std::ofstream timestamp_frame;
std::vector<double> tempo1, imu_x, imu_y, imu_z, imu_gx, imu_gy, imu_gz;

void log(const char* msg)
{
	std::lock_guard<std::mutex> lock(mm);
	logs.push_back(msg);
}

std::map<rs::stream, int> components_map =
{
	{ rs::stream::color,     3 },
	//{ rs::stream::infrared , 1 },      // Monochromatic
};

void sharpen2D(const cv::Mat &image, cv::Mat &result) {

	// Construct kernel (all entries initialized to 0)
	cv::Mat kernel(3, 3, CV_32F, cv::Scalar(0));
	// assigns kernel values
	kernel.at<float>(1, 1) = 5.0;
	kernel.at<float>(0, 1) = -1.0;
	kernel.at<float>(2, 1) = -1.0;
	kernel.at<float>(1, 0) = -1.0;
	kernel.at<float>(1, 2) = -1.0;

	//filter the image
	cv::filter2D(image, result, image.depth(), kernel);

}

int outputs(int i, std::vector<unsigned char *> frame_data_list, std::vector<double> tempo, int cols, int rows) {
	std::stringstream ss;
	int precision = std::numeric_limits<double>::max_digits10;
	tempo[i] = tempo[i] / 1000.0 + 9000.0;
	ss << "mov0/cam0/data/" << std::fixed << std::setprecision(9) << tempo[i] << ".png";
	stbi_write_png(ss.str().data(),
		cols, rows,
		components_map[rs::stream::color],
		frame_data_list[i],
		cols * components_map[rs::stream::color]);
	delete[] frame_data_list[i];
	IMU << std::fixed << std::setprecision(9) << tempo[i] << " , " 
		<< std::fixed << std::setprecision(9) << imu_gx[i] << " , " 
		<< std::fixed << std::setprecision(9) << imu_gy[i] << " , " 
		<< std::fixed << std::setprecision(9) << imu_gz[i] << " , "
		<< std::fixed << std::setprecision(9) << imu_x[i] << " , "
		<< std::fixed << std::setprecision(9) << imu_y[i] << " , "
		<< std::fixed << std::setprecision(9) << imu_z[i] << std::endl;

	timestamp_frame << std::fixed << std::setprecision(9) << tempo[i] << " , " << tempo[i] << ".png" << std::endl;
	return 0;
}

int main() try
{
	double tempo, x, y, z, gx, gy, gz;
	unsigned char   *   frame_data;
	//int num_of_frame = 60;	
	rs::context ctx;
	rs::device * dev = ctx.get_device(0);


	if (ctx.get_device_count() == 0){
		return EXIT_FAILURE;
		system("pause");
	}

	if (!dev->supports(rs::capabilities::motion_events))
	{
		printf("This device does not support motion tracking!");
		return EXIT_FAILURE;
	}
	//dev->get_option(rs::option::color_sharpness, 7);
	//To make all components synchronize with each other, a special option fisheye_strobe must be set to 1:

	auto motion_callback = [&x, &y, &z, &gx, &gy, &gz, &tempo](rs::motion_data entry){
		std::lock_guard<std::mutex> lock(mm);
		tempo = entry.timestamp_data.timestamp; //máximo tempo de filmagem = 99segundos
		
		if (entry.timestamp_data.source_id == RS_EVENT_IMU_ACCEL) {
			x = entry.axes[0];
			y = entry.axes[1];
			z = entry.axes[2];
		}
		if (entry.timestamp_data.source_id == RS_EVENT_IMU_GYRO) {
			gx = entry.axes[0];
			gy = entry.axes[1];
			gz = entry.axes[2];
		}
	};

	IMU.open("mov0/imu0/data.csv");
	timestamp_frame.open("mov0/cam0/data.csv");

	//determinar resolução
	dev->enable_stream(rs::stream::color, rs::preset::largest_image);
	//int cols = 640;
	//int rows = 480;
	int cols = 1920;
	int rows = 1080;
	dev->enable_stream(rs::stream::color, cols, rows, rs::format::rgb8, 30);
	//dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 30);
	dev->enable_motion_tracking(motion_callback);
	dev->start(rs::source::all_sources);


	std::vector<unsigned char *> frame_data_list;
	int i = 0;
	/*if (dev->supports_option(rs::option::r200_emitter_enabled))
	{
		int value = !dev->get_option(rs::option::r200_emitter_enabled);
		dev->set_option(rs::option::r200_emitter_enabled, 0);
	}*/

	cv::Mat image(rows, cols, CV_8UC3);
	cv::Mat image1(rows, cols, CV_8UC3);
	cv::Mat image_sharpen(rows, cols, CV_8UC3);
	cv::Mat image_sharpen1(rows, cols, CV_8UC3);
	cv::Mat result(rows, 2*cols, CV_8UC3);
	bool stop = false;


	rs::intrinsics parameters_intrin = dev->get_stream_intrinsics(rs::stream::color);
	rs::extrinsics parameters_extrin = dev->get_motion_extrinsics_from(rs::stream::fisheye);
		// struct rs_extrinsics parameters_extrin;

	while (!stop) {
		
		dev->wait_for_frames();
		frame_data = new unsigned char[rows * cols * 3];
		memcpy(frame_data,((uint8_t*)dev->get_frame_data(rs::stream::color)), rows * cols * 3);
		//memcpy(frame_data, ((uint8_t*)dev->get_frame_data(rs::stream::infrared)), 640 * 480 * 1);
		//frame_data_list.push_back(frame_data);
		memcpy(image.data, frame_data, rows * cols * 3);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
		sharpen2D(image, image_sharpen);
		cv::cvtColor(image_sharpen, image_sharpen, cv::COLOR_BGR2RGB);
		memcpy(frame_data, image_sharpen.data, rows * cols * 3);
		frame_data_list.push_back(frame_data);

		cv::resize(image, image1, cv::Size(640, 480));
		cv::cvtColor(image_sharpen, image_sharpen, cv::COLOR_BGR2RGB);
		cv::resize(image_sharpen, image_sharpen1, cv::Size(640, 480));
		cv::hconcat(image1, image_sharpen1, result);

		cv::imshow("result", result);
		char key = cv::waitKey(1);
		if (key == 's') {
			stop = true;
		}	
		tempo1.push_back(tempo);
		imu_x.push_back(x);
		imu_y.push_back(y);
		imu_z.push_back(z);
		imu_gx.push_back(gx);
		imu_gy.push_back(gy);
		imu_gz.push_back(gz);
		i++;
	}
	std::cout << "acabou_1" << std::endl;
	for (int j = 0; j < i; j++) {
		outputs(j, frame_data_list, tempo1, cols, rows);
		//std::cout << tempo1[i] << std::endl;
	}

	IMU.close();
	std::cout << "acabou_2" << std::endl;

	std::system("pause");
    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	system("pause");
	return EXIT_FAILURE;
}

