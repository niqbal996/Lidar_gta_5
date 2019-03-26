#define _USE_MATH_DEFINES
#include "script.h"
#include "keyboard.h"
#include <string>
#include <ctime>
#include <fstream>
#include <math.h>

#pragma warning(disable : 4244 4305) // double <-> float conversions

void notificationOnLeft(std::string notificationText) {
	UI::_SET_NOTIFICATION_TEXT_ENTRY("CELL_EMAIL_BCON");
	const int maxLen = 99;
	for (int i = 0; i < notificationText.length(); i += maxLen) {
		std::string divideText = notificationText.substr(i, min(maxLen, notificationText.length() - i));
		const char* divideTextAsConstCharArray = divideText.c_str();
		char* divideTextAsCharArray = new char[divideText.length() + 1];
		strcpy_s(divideTextAsCharArray, divideText.length() + 1, divideTextAsConstCharArray);
		UI::_ADD_TEXT_COMPONENT_STRING(divideTextAsCharArray);
	}
	int handle = UI::_DRAW_NOTIFICATION(false, 1);
}

struct ray {
	bool hit;
	Vector3 hitCoordinates;
	Vector3 surfaceNormal;
	std::string entityTypeName;
	int rayResult;
	int hitEntityHandle;
	int class_instance = 0;
};

ray raycast(Vector3 source, Vector3 direction, float maxDistance, int intersectFlags) {
	ray result;
	float targetX = source.x + (direction.x * maxDistance);
	float targetY = source.y + (direction.y * maxDistance);
	float targetZ = source.z + (direction.z * maxDistance);
	int rayHandle = WORLDPROBE::_CAST_RAY_POINT_TO_POINT(source.x, source.y, source.z, targetX, targetY, targetZ, intersectFlags, 0, 7);
	int hit = 0;
	int hitEntityHandle = -1;
	Vector3 hitCoordinates;
	hitCoordinates.x = 0;
	hitCoordinates.y = 0;
	hitCoordinates.z = 0;
	Vector3 surfaceNormal;
	surfaceNormal.x = 0;
	surfaceNormal.y = 0;
	surfaceNormal.z = 0;
	int rayResult = WORLDPROBE::_GET_RAYCAST_RESULT(rayHandle, &hit, &hitCoordinates, &surfaceNormal, &hitEntityHandle);
	int entityInstance = ENTITY::GET_OBJECT_INDEX_FROM_ENTITY_INDEX(result.hitEntityHandle);			//Instance ID of the class of the object hit
	result.rayResult = rayResult;
	result.hit = hit;
	result.hitCoordinates = hitCoordinates;
	result.surfaceNormal = surfaceNormal;
	result.hitEntityHandle = hitEntityHandle;
	std::string entityTypeName = "Unknown";
	if (ENTITY::DOES_ENTITY_EXIST(hitEntityHandle)) {
		int entityType = ENTITY::GET_ENTITY_TYPE(hitEntityHandle);
		if (entityType == 1) {
			entityTypeName = "GTA.Ped";
		}
		else if (entityType == 2) {
			entityTypeName = "GTA.Vehicle";
			result.class_instance = entityInstance;
		}
		else if (entityType == 3) {
			entityTypeName = "GTA.Prop";
		}
	}
	result.entityTypeName = entityTypeName;
	return result;
}

ray angleOffsetRaycast(double angleOffsetX, double angleOffsetZ, int range) {
	Vector3 rot = CAM::GET_GAMEPLAY_CAM_ROT(2);
	double rotationX = (rot.x + angleOffsetX) * (M_PI / 180.0);
	double rotationZ = (rot.z + angleOffsetZ) * (M_PI / 180.0);
	double multiplyXY = abs(cos(rotationX));
	Vector3 direction;
	direction.x = sin(rotationZ) * multiplyXY * -1;
	direction.y = cos(rotationZ) * multiplyXY;
	direction.z = sin(rotationX);
	ray result = raycast(CAM::GET_GAMEPLAY_CAM_COORD(), direction, range, -1);
	return result;
}

void lidar(double horiFovMin, double horiFovMax, double vertFovMin, double vertFovMax, double horiStep, double vertStep, int range, std::string filePath, std::string filePath_label)
{
	GAMEPLAY::SET_GAME_PAUSED(true);
	TIME::PAUSE_CLOCK(true);
	double vertexCount = (horiFovMax - horiFovMin) * (1 / horiStep) * (vertFovMax - vertFovMin) * (1 / vertStep);
	std::ofstream fileOutput;
	std::ofstream fileOutput_label_data;
	fileOutput.open(filePath);
	fileOutput_label_data.open(filePath_label);
	//fileOutput << "ply\nformat ascii 1.0\nelement vertex " + std::to_string((int)vertexCount) + "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
	for (double z = horiFovMin; z < horiFovMax; z += horiStep)
	{
		std::string vertexData = "";
		std::string label_Data = "";
		for (double x = vertFovMin; x < vertFovMax; x += vertStep)
		{
			std::string entityName3 = "None";
			int entityHash = 0;
			unsigned char r = 0; unsigned char g = 0; unsigned char b = 0;
			ray result = angleOffsetRaycast(x, z, range);
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
			vertexData += std::to_string(result.hitCoordinates.x) + " " + std::to_string(result.hitCoordinates.y) + " " + std::to_string(result.hitCoordinates.z) + " " + std::to_string(0) + "\n";	//place holder values for reflectance
			label_Data += std::to_string(r) + " " + std::to_string(g) + " " + std::to_string(b) + " " + std::to_string(result.class_instance) + "\n";		//placeholder value for instance values		
		}
		fileOutput << vertexData;
		fileOutput_label_data << label_Data;
	}
	fileOutput.close();
	fileOutput_label_data.close();
	GAMEPLAY::SET_GAME_PAUSED(false);
	TIME::PAUSE_CLOCK(false);
	//notificationOnLeft("LiDAR Point Cloud written to file.");
}

void ScriptMain()
{
	srand(GetTickCount());
	DWORD start_time;
	DWORD elapsed_time;
	int file_number = 0;
	bool init = TRUE;
	int range;
	std::string filename;
	std::string label_filename;
	std::ifstream inputFile;
	std::string ignore;
	double parameters[6];
	while (true)
	{	
		if (IsKeyJustUp(VK_F6))
			if (init) {
				inputFile.open("LiDAR GTA V/LiDAR GTA V.cfg");
				if (inputFile.bad()) {
					notificationOnLeft("Input file not found. Please re-install the plugin.");
					continue;
				}
				/*Ignore the first line of the config file*/
				inputFile >> ignore >> ignore >> ignore >> ignore >> ignore;
				for (int i = 0; i < 6; i++) {
					/*Read in parameter values of all 5 LiDAR parameters*/
					inputFile >> ignore >> ignore >> parameters[i];
				}
				/*LiDAR range and filenames*/
				inputFile >> ignore >> ignore >> range;
				inputFile >> ignore >> ignore >> filename;
				inputFile >> ignore >> ignore >> label_filename;
				inputFile.close();
			}
			elapsed_time = GetTickCount() - start_time;
			if (elapsed_time > 30000)				/*in milli seconds. if more than 30 milliseconds take a lidar screen shot*/
			{
				start_time = GetTickCount();
				lidar(parameters[0], parameters[1], parameters[2], parameters[3], parameters[4], parameters[5], range,
					"LiDAR GTA V/velodyne/" + filename + "_" + std::to_string(file_number) + ".txt",
					"LiDAR GTA V/labels/" + label_filename + "_" + std::to_string(file_number) + ".txt");
				file_number++;
				notificationOnLeft("Point Cloud sample number " + std::to_string(file_number) + " Generated.");
				notificationOnLeft("Resumed game.");
			}
		WAIT(0);
	}		
}