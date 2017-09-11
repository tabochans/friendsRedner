#pragma once
#include"Vec3.h"

class Convert__RGB_HSV {
public:
	
	static PTUtility::Vec3 HSV_to_RGB(const PTUtility::Vec3& HSV) {
		
		if (std::abs(HSV.y()) < 0.0001) {
			return PTUtility::Vec3::Ones() * HSV.z();
		}

		float C = HSV.y();
		int Hp = HSV.x() * 6.0f;
		float f = HSV.x() * 6.0f - Hp;

		PTUtility::Vec3 RGB(HSV.z(), HSV.z(), HSV.z());

		switch (Hp) {
		case 0:
			RGB.y() *= (1.0f - HSV.y() * (1.0f - f));
			RGB.z() *= (1.0f - HSV.y());
			break;
		case 1:
			RGB.x() *= (1.0f - HSV.y() * f);
			RGB.z() *= (1.0f - HSV.y());
			break;
		case 2:
			RGB.x() *= (1.0f - HSV.y());
			RGB.z() *= (1.0f - HSV.y() * (1.0f - f));
			break;
		case 3:
			RGB.x() *= (1.0f - HSV.y());
			RGB.y() *= (1.0f - HSV.y() * f);
			break;
		case 4:
			RGB.x() *= (1.0f - HSV.y() * (1.0f - f));
			RGB.y() *= (1.0f - HSV.y());
			break;
		case 5:
			RGB.y() *= (1.0f - HSV.y());
			RGB.z() *= (1.0f - HSV.y() * f);
			break;
		}
		return RGB;
	}
	static PTUtility::Vec3 RGB_to_HSV(const PTUtility::Vec3& RGB) {

		int MaxID = RGB.x() > RGB.y() ? (RGB.x() > RGB.z() ? 0 : 2) : (RGB.y() > RGB.z() ? 1 : 2);
		int MinID = RGB.x() < RGB.y() ? (RGB.x() < RGB.z() ? 0 : 2) : (RGB.y() < RGB.z() ? 1 : 2);
		float Max_Min = RGB[MaxID] - RGB[MinID];

		float H = 0.0f;
		switch (MinID) {
		case 0:
			H = 60.0f * (RGB.y() - RGB.x()) / Max_Min + 60.0f;
			break;
		case 1:
			H = 60.0f * (RGB.z() - RGB.y()) / Max_Min + 180.0f;
			break;
		case 2:
			H = 60.0f * (RGB.x() - RGB.z()) / Max_Min + 300.0f;
			break;
		}
		float V = RGB[MaxID];
		float S = Max_Min;

		return PTUtility::Vec3(H, S, V);
	}
	static PTUtility::Vec3 GetGradation(float param) {
		return HSV_to_RGB(PTUtility::Vec3(std::max(0.0f, std::min(1.0f, param)), 1.0f, 1.0f));
	}
};