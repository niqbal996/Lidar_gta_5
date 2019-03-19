#define _USE_MATH_DEFINES
#include "script.h"
#include "keyboard.h"

#include <string>
#include <ctime>
#include <fstream>
#include <math.h>

#include <Eigen/Dense>
#include <vector>

#include <chrono>

#include <iostream>
#include <sstream>
#include <iomanip>

class Timer
{
public:
	Timer() : beg_(clock_::now()), last_count(0) ,paused(false){}
	void reset() { beg_ = clock_::now(); last_count = 0; }
	double elapsed() const {

		return paused ? last_count :
			(last_count + std::chrono::duration_cast<second_>(clock_::now() - beg_).count());
	}

	void pause() {

		last_count += std::chrono::duration_cast<second_>
			(clock_::now() - beg_).count();
		paused = true;
	}
	void resume() {
		beg_ = clock_::now();
		paused = false;
	}

private:
	typedef std::chrono::high_resolution_clock clock_;
	typedef std::chrono::duration<double, std::ratio<1> > second_;
	std::chrono::time_point<clock_> beg_;
	double last_count;
	bool paused;
};


template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 2)
{
	std::ostringstream out;
	out.precision(n);
	out << std::fixed << a_value;
	return out.str();
}


#pragma warning(disable : 4244 4305) // double <-> float conversions


void notificationOnLeft(const std::string & notificationText) {
	///<Summary> Print string to GTAV's bottom left notifications
	UI::_SET_NOTIFICATION_TEXT_ENTRY("CELL_EMAIL_BCON");
	const int maxLen = 99;
	for (int i = 0; i < notificationText.length(); i += maxLen) {
		//Split into blocks that fit in 99char space
		const auto end = min(maxLen, notificationText.length() - i);
		const int len = static_cast<int>(end) - i;

		const char* divideTextAsConstCharArray = (notificationText.c_str()) + i;
		char* divideTextAsCharArray = new char[len + 1];

		strcpy_s(divideTextAsCharArray, len + 1, divideTextAsConstCharArray);
		UI::_ADD_TEXT_COMPONENT_STRING(divideTextAsCharArray);
	}
	UI::_DRAW_NOTIFICATION(false, 1);
}

struct ray {
	int hit;
	Vector3 hitCoordinates;
	Vector3 surfaceNormal;
	std::string entityTypeName;
	int rayResult;
	int hitEntityHandle;
};



void raycast(ray& result, const Eigen::Vector3f& source, const Eigen::Vector3f& direction, const float maxDistance, int intersectFlags, Timer& debug_timer) {

	const Eigen::Vector3f target = source + (direction * maxDistance);

	debug_timer.resume();

	int rayResult = WORLDPROBE::_GET_RAYCAST_RESULT(
		WORLDPROBE::_CAST_RAY_POINT_TO_POINT(
			source[0], source[1], source[2],
			target[0], target[1], target[2],
			intersectFlags, 0, 7
		),
		&result.hit, &result.hitCoordinates, &result.surfaceNormal, &result.hitEntityHandle
	);

	debug_timer.pause();
	int entityType = 
		(ENTITY::DOES_ENTITY_EXIST(result.hitEntityHandle)) ? \
		ENTITY::GET_ENTITY_TYPE(result.hitEntityHandle):
		0;
	switch (entityType)
	{
	case 0:
		result.entityTypeName = "Unknown";
		break;
	case 1:
		result.entityTypeName = "GTA.Ped";
		break;
	case 2:
		result.entityTypeName = "GTA.Vehicle";
		break;
	case 3:
		result.entityTypeName = "GTA.Prop";
		break;
	default:
		result.entityTypeName = "Unknown";
		break;
	}
}

void angleOffsetRaycast(const double angleOffsetX, const double angleOffsetZ, const int range,
	ray& result, const Eigen::Vector3f& cam_coord, const Eigen::Vector3f& rot, Timer& debug_timer) {
	
	const double rotationX = (rot[0] + angleOffsetX) * (M_PI / 180.0);
	const double rotationZ = (rot[2] + angleOffsetZ) * (M_PI / 180.0);
	const double multiplyXY = abs(cos(rotationX));

	const Eigen::Vector3f direction(
		static_cast<float>(sin(rotationZ) * multiplyXY * -1),
		static_cast<float>(cos(rotationZ) * multiplyXY),
		static_cast<float>(sin(rotationX))
	);

	raycast(result,cam_coord, direction, range, -1,debug_timer);
}

void lidar(const int hCount, const int vCount, const long int vertexCount,
			const double hStep, const double vStep, std::vector<std::vector<ray>>& rayArray,
			int range, const std::string& filePath, const std::string& filePath_label)
{
	Timer _t_tot,_t_core;
	std::ofstream fileOutput;
	fileOutput.open(filePath);
	std::ofstream fileOutput_label;
	fileOutput_label.open(filePath_label);
	/*fileOutput << "ply\nformat ascii 1.0\nelement vertex " + std::to_string((int)vertexCount)\
		+ "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
*/	
	/*Seperate the values for point cloud and the ground truth values.*/
	fileOutput << "\nproperty float x\nproperty float y\nproperty float z\n";
	fileOutput_label << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
	//GAMEPLAY::SET_GAME_PAUSED(true); // %TODO: Delete this? Or Make it optional
	//TIME::PAUSE_CLOCK(true);
	const auto cam_coord = CAM::GET_GAMEPLAY_CAM_COORD();
	const Eigen::Vector3f cam_coord_e(
		cam_coord.x,
		cam_coord.y,
		cam_coord.z);

	const auto cam_rot = CAM::GET_GAMEPLAY_CAM_ROT(2);
	const Eigen::Vector3f cam_rot_e(
		cam_rot.x,
		cam_rot.y,
		cam_rot.z);
	
	_t_core.pause();
	_t_core.reset();
	for (int i=0;i<hCount;i++)
	{
		std::string vertexData = "";

		for (int j=0;j<vCount;j++)
		{
			std::string entityName3 = "None";
			int entityHash = 0;
			unsigned char r = 0; unsigned char g = 0; unsigned char b = 0;
			angleOffsetRaycast(i*hStep, j*vStep, range,
				rayArray[i][j],cam_coord_e,cam_rot_e,_t_core);
			const auto& result = rayArray[i][j];
			if (result.hit)
			{
				r = 255; g = 255; b = 255;
			}
			if (result.hitEntityHandle != -1)
			{
				entityName3 = result.entityTypeName;
				if (entityName3 == "GTA.Vehicle")
				{
					r = 255; g = 0; b = 0;
				}
				else if (entityName3 == "GTA.Ped")
				{
					r = 0; g = 255; b = 0;
				}
				else if (entityName3 == "GTA.Prop")
				{
					r = 0; g = 0; b = 255;
				}
			}
			vertexData += std::to_string(result.hitCoordinates.x) + " " + std::to_string(result.hitCoordinates.y) + " " + std::to_string(result.hitCoordinates.z) + " " + std::to_string(r) + " " + std::to_string(g) + " " + std::to_string(b) + "\n";
		}
		fileOutput << vertexData;
	}
	fileOutput.close();
	//GAMEPLAY::SET_GAME_PAUSED(false);
	//TIME::PAUSE_CLOCK(false);
	
	std::string msg = "Lidar(). _tot: " + to_string_with_precision(_t_tot.elapsed()) \
		+ "_core: " + to_string_with_precision(_t_core.elapsed())\
		;
	notificationOnLeft(msg.c_str());
}
void ScriptMain()
{
	srand(GetTickCount());
	bool init = true;
	double parameters[6];
	int range;
	std::string filename;
	std::string label_filename;
	std::string ignore;
	std::ifstream inputFile;

	int hCount, vCount; long int vertexCount;
	int file_number = 0;

	std::vector<std::vector<ray>> rayArray;
	typedef std::chrono::high_resolution_clock clock_;
	typedef std::chrono::duration<double, std::ratio<1> > second_;
	while (true)
	{
		if (init) {
			inputFile.open("LiDAR GTA V/LiDAR GTA V.cfg");
			if (inputFile.bad()) {
				notificationOnLeft("Input file not found. Please re-install the plugin.");
				continue;
			}
			inputFile >> ignore >> ignore >> ignore >> ignore >> ignore;
			for (int i = 0; i < 6; i++) {
				inputFile >> ignore >> ignore >> parameters[i];
			}
			inputFile >> ignore >> ignore >> range;
			inputFile >> ignore >> ignore >> filename;
			inputFile.close();
			

			hCount = static_cast <int> (std::floor((parameters[1] - parameters[0]) / parameters[4]));
			vCount = static_cast <int> (std::floor((parameters[3] - parameters[2]) / parameters[5]));
			vertexCount = static_cast <long int>(hCount) * static_cast <long int>(vCount);

			

			rayArray.resize(hCount);
			for (auto& rArray : rayArray) {
				rArray.resize(vCount);
			}
			init = false;
			
		}
		if (IsKeyJustUp(VK_F6))
		{
			static const std::chrono::seconds lidar_screenshot_period(30); // snap every 0.2 seconds snap every half second. 
			

			while (!IsKeyJustUp(VK_F6)) {
				const auto t1 = std::chrono::high_resolution_clock::now();
				
				lidar(hCount, vCount, vertexCount,
					parameters[4], parameters[5], rayArray,
					range, "LiDAR GTA V/" + filename + "_" + std::to_string(file_number) + ".txt", 
							"LiDAR GTA V/" + label_filename + "_" + std::to_string(file_number) + ".txt");
				file_number++;

				auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
					std::chrono::high_resolution_clock::now() - t1);
				
				int wait = (lidar_screenshot_period > elapsed) ? (lidar_screenshot_period - elapsed).count() : 0;
				
				notificationOnLeft(
					"_t_" + to_string_with_precision(elapsed.count()) +
					"  _p_" + to_string_with_precision(lidar_screenshot_period.count())
				);
				WAIT(wait);
			}
		}
		WAIT(0);
	}
}