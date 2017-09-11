#pragma once
#include"Vec3.h"
#include"MT.h"

namespace Sampling{
	const float DeltaPDF = 1000.0f;
	const float sMin = 0.0000001;

	float Fresnel_Reflectance(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float In_Index, float Out_Index);
	float FresnelConductor_Reflectance(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float Index, float absorption);

	PTUtility::Vec3 Cos_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, float* pdf = nullptr);
	PTUtility::Vec3 Bi_Cos_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, float Reflectance, float* pdf = nullptr);
	PTUtility::Vec3 Sphere_Sampling(RandomMT& MT, float* pdf = nullptr);
	PTUtility::Vec3 HemSphere_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, float* pdf = nullptr);
	PTUtility::Vec3 Phong_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float Power, float* pdf = nullptr);
	PTUtility::Vec3 GGX_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float Alpha, float* pdf = nullptr);
	PTUtility::Vec3 GGXGlass_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float Alpha, float In_Index, float Out_Index, float* pdf = nullptr);
	PTUtility::Vec3 Reflect_Sampling(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir);
	PTUtility::Vec3 Refract_Sampling(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float In_Index, float Out_Index);
	PTUtility::Vec3 Fresnel_Sampling(RandomMT& MT, const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float In_Index, float Out_Index, float* pdf = nullptr);
	
	float Fresnel_Reflect_PDF(const PTUtility::Vec3& Normal, const PTUtility::Vec3& InDir, float In_Index, float Out_Index);
}