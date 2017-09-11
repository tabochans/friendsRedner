#pragma once

#include"Vec3.h"
#include"Sampling.h"


namespace PTUtility{

	class SurfaceMaterial{
	public:
		typedef Vec3(*Sampling_Function)(const PTUtility::Vec3& InDir, const PTUtility::Vec3& Normal, float* pdf, RandomMT& MT, float FromIndex, float IntoIndex, float param1, float param2);

		// if material is opaque, InDir.dot(OutDir) < 0
		typedef float(*BRDF_Function)(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2);

		typedef float(*PDF_Function)(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2, char dm);
	};

	class SurfaceMat : private SurfaceMaterial {
	private:
		typedef Vec3(*Sampling_Function)(const PTUtility::Vec3& InDir, const PTUtility::Vec3& Normal, float* pdf, RandomMT& MT, float FromIndex, float IntoIndex, float param1, float param2);
		typedef float(*BRDF_Function)(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2);
		typedef float(*PDF_Function)(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2, char dm);
	};

	class PlaneSurface : private SurfaceMat {
	public:
		static float Plane(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2) {
			return 1.0f / (PTUtility::PI);
		}
		static PTUtility::Vec3 Plane_Sampling(const PTUtility::Vec3& InDir, const PTUtility::Vec3& Normal, float* pdf, RandomMT& MT, float FromIndex, float IntoIndex, float param1, float param2) {
			return Sampling::HemSphere_Sampling(MT, Normal, pdf);
		}
		static float PDF(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2, char dm) {
			return 1.0f / (2.0f * PTUtility::PI);
		}
	};

	class DiffuseSurface : private SurfaceMat {
	public:
		static float Diffuse(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2){
			return 1.0f / (PTUtility::PI);
		}
		static PTUtility::Vec3 Diffuse_Sampling(const PTUtility::Vec3& InDir, const PTUtility::Vec3& Normal, float* pdf, RandomMT& MT, float FromIndex, float IntoIndex, float param1, float param2){
			return Sampling::Cos_Sampling(MT, Normal, pdf);
		}
		static float PDF(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2, char dm) {
			return OutDir.dot(Normal) / PTUtility::PI;
		}
	};

	class LightSurface : private SurfaceMat {
	public:
		static float Light(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2) {
			return 1.0f;
		}
		static PTUtility::Vec3 Light_Sampling(const PTUtility::Vec3& InDir, const PTUtility::Vec3& Normal, float* pdf, RandomMT& MT, float FromIndex, float IntoIndex, float param1, float param2) {
			return Sampling::HemSphere_Sampling(MT, Normal, pdf);
		}
		static float PDF(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2, char dm) {
			return 1.0f / (2.0f * PTUtility::PI);
		}
	};

	template<typename int POWx100>
	class PhongSurface : private SurfaceMat {
	public:
		static float Phong(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2){
			if (Normal.dot(OutDir) < 0) {
				return 0.0f;
			}
			PTUtility::Vec3 ref = Sampling::Reflect_Sampling(Normal, InDir);
			float cos = std::max(Sampling::sMin, std::min(1.0f, ref.dot(OutDir)));
			return std::pow(cos, POWx100 * 0.01) * ((POWx100 * 0.01f + 1.0f) / (2.0f * PTUtility::PI));
		}
		static PTUtility::Vec3 Phong_Sampling(const PTUtility::Vec3& InDir, const PTUtility::Vec3& Normal, float* pdf, RandomMT& MT, float FromIndex, float IntoIndex, float param1, float param2){
			return Sampling::Phong_Sampling(MT, Normal, InDir, POWx100 * 0.01, pdf);
		}
		static float PDF(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2, char dm) {
			return Phong(InDir, OutDir, Normal, FromIndex, IntoIndex, param1, param2);
		}
	};

	class GGXSurface : private SurfaceMat {
	public:
		static float GGX(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2) {
			if (Normal.dot(OutDir) < 0) {
				return 0.0f;
			}
			
			float Alpha = param1;
			
			float Lm_l = (-1.0f + std::sqrt(1.0f + Alpha*Alpha * (1.0f / (OutDir.dot(Normal) + 0.00001) - 1.0f)));
			float Lm_v = (-1.0f + std::sqrt(1.0f + Alpha*Alpha * (1.0f / (-InDir.dot(Normal) + 0.00001) - 1.0f)));
			float G = 1.0f / (1.0f + Lm_l + Lm_v);

			PTUtility::Vec3 HalfVector = (-InDir + OutDir).normalized();
			float D = (Alpha * Alpha) / (PTUtility::PI * std::pow(1.0f - (1.0f - Alpha*Alpha) * std::pow(Normal.dot(HalfVector), 2.0f), 2.0f));

			float F = Sampling::FresnelConductor_Reflectance(HalfVector, InDir, IntoIndex, param2);

			float cosi = -InDir.dot(Normal);
			float coso = OutDir.dot(Normal);

			return std::max(Sampling::sMin, F * D * G / (4.0f * cosi * coso));
		}
		static PTUtility::Vec3 GGX_Sampling(const PTUtility::Vec3& InDir, const PTUtility::Vec3& Normal, float* pdf, RandomMT& MT, float FromIndex, float IntoIndex, float param1, float param2) {
			return Sampling::GGX_Sampling(MT, Normal, InDir, param1, pdf);
		}
		static float PDF(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2, char dm) {
			if (Normal.dot(OutDir) < 0.000001) {
				return 0.0f;
			}
			PTUtility::Vec3 half = (-InDir + OutDir).normalized();
			float cos = std::max(Sampling::sMin, std::min(1.0f, half.dot(Normal)));
			float sin = std::sqrt(1.000 - cos*cos);
			return (param1 * param1 * cos) / (PTUtility::PI * std::pow((param1*param1 - 1.0f) * cos * cos + 1.0f, 2.0f));
		}
	};

	class GGXGlass : private SurfaceMat {
	public:
		static float GGX(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2) {			
			float A2 = param1*param1;
			float Lm_l = (-1.0f + std::sqrt(1.0f + A2 * (1.0f / (std::abs(OutDir.dot(Normal)) + 0.00001) - 1.0f)));
			float Lm_v = (-1.0f + std::sqrt(1.0f + A2 * (1.0f / (std::abs(-InDir.dot(Normal)) + 0.00001) - 1.0f)));
			float G = 1.0f / (1.0f + Lm_l + Lm_v);

			float k = OutDir.dot(InDir);
			float cosi = std::abs(-InDir.dot(Normal));
			float coso = std::abs(OutDir.dot(Normal));

			PTUtility::Vec3 HalfVector = (-InDir + OutDir).normalized();
			if (OutDir.dot(Normal) < 0.0) {
				float eta = IntoIndex / FromIndex;
				float a = std::sqrt(1.0f / (eta*eta - 2.0f * k * eta + 1.0f));
				float b = -eta * a;
				HalfVector = (a * InDir + b * OutDir).normalized();
				if (eta < 1.0) {
					HalfVector = -HalfVector;
				}
			}
			float D = A2 / (PTUtility::PI * std::pow(1.0f - (1.0f - A2) * std::pow(Normal.dot(HalfVector), 2.0f), 2.0f));

			float F = Sampling::Fresnel_Reflectance(HalfVector, InDir, FromIndex, IntoIndex);
			if (OutDir.dot(Normal) < 0.0) {
				F = 1.0f - F;
			}
			
			return std::max(Sampling::sMin, F * D * G / (4.0f * cosi * coso));
		}
		static PTUtility::Vec3 GGX_Sampling(const PTUtility::Vec3& InDir, const PTUtility::Vec3& Normal, float* pdf, RandomMT& MT, float FromIndex, float IntoIndex, float param1, float param2) {
			return Sampling::GGXGlass_Sampling(MT, Normal, InDir, param1, FromIndex, IntoIndex, pdf);
		}
		static float PDF(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2, char dm) {
			float RefPDF = Sampling::Fresnel_Reflect_PDF(Normal, InDir, FromIndex, IntoIndex);
			if (Normal.dot(OutDir) > 0.000001) {

				PTUtility::Vec3 half = (-InDir + OutDir).normalized();
				float cos = std::max(Sampling::sMin, std::min(1.0f, half.dot(Normal)));
				float sin = std::sqrt(1.000 - cos*cos);
				return (RefPDF) * (param1 * param1 * cos) / (PTUtility::PI * std::pow((param1*param1 - 1.0f) * cos * cos + 1.0f, 2.0f));
			}
			else {
				float k = OutDir.dot(InDir);
				float cosi = std::abs(-InDir.dot(Normal));
				float coso = std::abs(OutDir.dot(Normal));

				float eta = IntoIndex / FromIndex;
				float a = std::sqrt(1.0f / (eta*eta - 2.0f * k * eta + 1.0f));
				float b = -eta * a;
				PTUtility::Vec3 half = (a * InDir + b * OutDir).normalized();
				if (eta < 1.0) {
					half = -half;
				}

				float cos = std::max(Sampling::sMin, std::min(1.0f, half.dot(Normal)));
				float sin = std::sqrt(1.000 - cos*cos);
				return (1.0001 - RefPDF) * (param1 * param1 * cos) / (PTUtility::PI * std::pow((param1*param1 - 1.0f) * cos * cos + 1.0f, 2.0f));
			}
		}
	};

	class MetalSurface : private SurfaceMat {
	public:
		static float Metal(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2){
			PTUtility::Vec3 ref = Sampling::Reflect_Sampling(Normal, InDir);
			if (ref.dot(OutDir) > 1.0f - 1.0f / Sampling::DeltaPDF) {
				return Sampling::DeltaPDF / (OutDir.dot(Normal) + 0.00001);
			}
			else {
				return 0.0f;
			}
		}
		static PTUtility::Vec3 Metal_Sampling(const PTUtility::Vec3& InDir, const PTUtility::Vec3& Normal, float* pdf, RandomMT& MT, float FromIndex, float IntoIndex, float param1, float param2){
			if (pdf){
				*pdf = Sampling::DeltaPDF;
			}
			return Sampling::Reflect_Sampling(Normal, InDir);
		}
		static float PDF(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2, char dm) {
			PTUtility::Vec3 ref = Sampling::Reflect_Sampling(Normal, InDir);
			if (ref.dot(OutDir) > 1.0f - 1.0f / Sampling::DeltaPDF) {
				return Sampling::DeltaPDF;
			}
			else{
				return Sampling::sMin;
			}
		}
	};

	class GlassSurface : private SurfaceMat {
	public:
		static float Glass(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2) {
			if (Normal.dot(OutDir) > 0) {
				return Sampling::DeltaPDF * Sampling::Fresnel_Reflectance(Normal, InDir, FromIndex, IntoIndex) / (OutDir.dot(Normal) + 0.0001);
			}
			else {
				return Sampling::DeltaPDF * (1.0f - Sampling::Fresnel_Reflectance(Normal, InDir, FromIndex, IntoIndex)) / (OutDir.dot(-Normal) + 0.0001);;
			}
		}
		static PTUtility::Vec3 Glass_Sampling(const PTUtility::Vec3& InDir, const PTUtility::Vec3& Normal, float* pdf, RandomMT& MT, float FromIndex, float IntoIndex, float param1, float param2) {
			float RefPDF = Sampling::Fresnel_Reflect_PDF(Normal, InDir, FromIndex, IntoIndex);
			if (MT.genrand64_real1() < RefPDF) {
				//Reflect
				if (pdf)*pdf = Sampling::DeltaPDF * RefPDF;
				return Sampling::Reflect_Sampling(Normal, InDir);
			}
			else {
				//Transmitte
				if (pdf)*pdf = Sampling::DeltaPDF * (1.000001 - RefPDF);
				return Sampling::Refract_Sampling(Normal, InDir, FromIndex, IntoIndex);
			}
		}
		static float PDF(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2, char dm) {
			PTUtility::Vec3 NN = Normal.dot(InDir) > 0 ? -Normal : Normal;
			float RefPDF = Sampling::Fresnel_Reflect_PDF(Normal, InDir, IntoIndex, IntoIndex);
			
			PTUtility::Vec3 ref = Sampling::Reflect_Sampling(NN, InDir);
			PTUtility::Vec3 rac = Sampling::Refract_Sampling(NN, InDir, FromIndex, IntoIndex);

			float refcos = OutDir.dot(ref);
			float raccos = OutDir.dot(rac);

			if (refcos > 1.0f - 1.0f / Sampling::DeltaPDF) {
				return RefPDF * Sampling::DeltaPDF;
			}
			else if (raccos > 1.0f - 1.0f / Sampling::DeltaPDF) {
				return (1.000001 - RefPDF) * Sampling::DeltaPDF;
			}
			else {
				return Sampling::sMin;
			}
		}
	};

	class TransSurface : private SurfaceMat {
	public:
		static float Leaf(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2) {
			// param1 describes reflectance
			
			if (OutDir.dot(Normal) > 0) {
				return (param1) / (PTUtility::PI);
			}
			else {
				return (1.0f - param1) / (PTUtility::PI);
			}
		}
		static PTUtility::Vec3 Leaf_Sampling(const PTUtility::Vec3& InDir, const PTUtility::Vec3& Normal, float* pdf, RandomMT& MT, float FromIndex, float IntoIndex, float param1, float param2) {
			return Sampling::Bi_Cos_Sampling(MT, Normal, param1, pdf);
		}
		static float PDF(const PTUtility::Vec3& InDir, const PTUtility::Vec3& OutDir, const PTUtility::Vec3& Normal, float FromIndex, float IntoIndex, float param1, float param2, char dm) {
			float cos = OutDir.dot(Normal);
			if (cos > 0) {
				return (param1) * cos / (PTUtility::PI);
			}
			else {
				return (1.0f - param1) * (-cos) / (PTUtility::PI);
			}
		}
	};

}
