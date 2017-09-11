#pragma once
#include"Texture.h"
#include"Vec3.h"

class IBLTexture : public Texture2D{
private:
	PTUtility::Vec3 GetColor(float u, float v);
public:
	PTUtility::Vec3 Get_IBLColor(const PTUtility::Vec3& Direction, float Power = 1.0){
		//とりあえずテキトーに
		float u, v;
		float ph, r;

		if (Direction.x()*Direction.x() + Direction.z()* Direction.z() > 0.01){
			ph = (acos(Direction.x() / sqrt(Direction.x()*Direction.x() + Direction.z()*Direction.z())));
		}
		else{
			ph = 0.0f;
		}
		return Power * GetColor(ph / (PTUtility::PI), 0.6 + (-Direction.y() + 1.0f));
	}
};
