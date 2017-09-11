#pragma once
#include"Vec3.h"

class iCamera{
	float DistanceToPlane;
protected:
	float GetDistanceToPlane()const{ return DistanceToPlane; }
	PTUtility::Vec3 _Pos;
	PTUtility::Vec3 _Center;
	PTUtility::Vec3 _Dir;

	PTUtility::Vec3 e1;
	PTUtility::Vec3 e2;

	float _AspectRatio;
	float _Fov;
	float _PlaneWidth;
	float _PlaneHeight;

public:
	virtual PTUtility::Vec3 GetPixelFromPosition(const PTUtility::Vec3& Position, float&u, float&v)const = 0;
	virtual float Pixel_PDFstr(const PTUtility::Vec3& TargetPos, const PTUtility::Vec3& TargetNormal, int ImageWidth, int ImageHeight)const = 0;
	PTUtility::Vec3 GetCameraPos()const{ return _Pos; }
	PTUtility::Vec3 Get_E1()const{ return e1; }
	PTUtility::Vec3 Get_E2()const{ return e2; }
	float GetPlaneWidth()const{ return _PlaneWidth; }
	float GetPlaneHeight()const{ return _PlaneHeight; }
	PTUtility::Vec3 GetCameraDir()const{ return _Dir; }
	iCamera(const PTUtility::Vec3& Position, const PTUtility::Vec3& Target, float Fov, float AspectRatio) : 
		_Pos(Position), _Center(Target), _Fov(1.0 * PTUtility::PI * Fov / 360.0), DistanceToPlane(1.0f), _AspectRatio(AspectRatio), _Dir((Target - Position).normalized())
	{
		_PlaneWidth = 2.0 * DistanceToPlane * tan(_Fov) * _AspectRatio;
		_PlaneHeight = 2.0 * DistanceToPlane * tan(_Fov);
	}
	virtual ~iCamera(){

	}
	virtual PTUtility::Vec3 GeneratePos(float u, float v)const = 0;
};

class PHCamera : public iCamera{
private:

public:

	PHCamera(const PTUtility::Vec3& Position, const PTUtility::Vec3& Target, float Fov, float AspectRatio) :
		iCamera(Position, Target, Fov, AspectRatio){

		//calculate e1,e2 vector
		PTUtility::Vec3 LK = (_Center - _Pos).normalized();

		if (LK.y() > 0.99){
			e1 = PTUtility::Vec3(1, 0, 0);
			e2 = PTUtility::Vec3(0, 0, 1);
		}
		else{
			e2 = LK.cross(PTUtility::Vec3(0, 1, 0)).normalized();
			e1 = e2.cross(LK).normalized();
		}
	}
	virtual PTUtility::Vec3 GeneratePos(float u, float v)const{
		return _Pos + _Dir * GetDistanceToPlane() + (u - 0.5)*_PlaneWidth * e2 + (v - 0.5) *_PlaneHeight*e1;
	}
	virtual PTUtility::Vec3 GetPixelFromPosition(const PTUtility::Vec3& Position, float&u, float&v)const {
		u = -100; v = -100;
		float ccos = _Dir.dot((Position - _Pos).normalized());
		if (ccos < 0.00001) {
			return PTUtility::Vec3::Zero();
		}
		PTUtility::Vec3 PosOnPlane = (GetDistanceToPlane() / ccos) * (Position - _Pos).normalized() + _Pos;
		u = (PosOnPlane - _Center).dot(e2) / _PlaneWidth;
		v = (PosOnPlane - _Center).dot(e1) / _PlaneHeight;
		return PosOnPlane;
	}
	virtual float Pixel_PDFstr(const PTUtility::Vec3& TargetPos, const PTUtility::Vec3& TargetNormal, int ImageWidth, int ImageHeight)const {
		float u, v = 1.0f;
		PTUtility::Vec3 PosOnPlane = GetPixelFromPosition(TargetPos, u, v);
		float dist2 = (PosOnPlane - _Pos).norm2();
		float ccos = (PosOnPlane - _Pos).normalized().dot(_Dir);
		float cos = TargetNormal.dot((_Pos - TargetPos).normalized());
		float dd2 = (_Pos - TargetPos).norm2();
		return (cos / dd2) * (1.0f / (ccos * (_PlaneWidth * _PlaneHeight) / dist2));
	}
};


class OrthCamera : public iCamera{
private:
public:
	OrthCamera(
		const PTUtility::Vec3& Position, const PTUtility::Vec3& PlaneCenter, float PlaneWidth, float PlaneHeight) :
		iCamera(Position, PlaneCenter, PlaneWidth, PlaneHeight){

		PTUtility::Vec3 LK = (_Center - _Pos).normalized();
		if (LK.y() > 0.996){
			e1 = PTUtility::Vec3(1, 0, 0);
			e1 = PTUtility::Vec3(0, 0, 1);
		}
		else{
			e1 = LK.cross(PTUtility::Vec3(1, 0, 0)).normalized();
			e2 = e1.cross(LK).normalized();
		}
	}
	virtual PTUtility::Vec3 GetPixelFromPosition(const PTUtility::Vec3& Position, float&u, float&v)const {
		u = -100; v = -100;
		return PTUtility::Vec3::Zero();
	}

	PTUtility::Vec3 GeneratePos(float u, float v)const{
		return _PlaneWidth * e2 * (u - 0.5f) + _PlaneHeight * e1 * (v - 0.5f) + _Center;
	}

	~OrthCamera(){
	}


};