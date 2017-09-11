#pragma once

#pragma once
#include"Vec3.h"
#include"Vec2.h"
#include"OBVH.h"
#include"Camera.h"
#include"LightManager.h"
#include"ScopedTimer.h"
#include"MT.h"
#include"SuperSampling.h"
#include"Scene.h"
#include"PostFilter.h"
#include<time.h>
#include<string>

namespace PTUtility{
	class bdPathTracing{
	private:

		int RDepth()const {
			return 8;
		}
		float Russian_PDF(int Depth) const{
			int R_DEPTH = RDepth();
			float rus_pdf = 1.0f;
			if (Depth > R_DEPTH) {
				return exp(5.0 * (R_DEPTH - Depth));
			}
			else {
				return 1.0f;
			}
		}
		float Russian_PDF_next(int Depth, int NextDepth) const{
			return Russian_PDF(NextDepth) / Russian_PDF(Depth);
		}


		//Rendering Parameter
		struct BDPTParam {
			Vec3 _Pos;
			Vec3 _Result;
			Primitive::Ray _NextRay;
			Primitive::Ray _CurrentRay;
			Vec3 _Through;
			float _PDF;
			int _Depth;
			bool _IfLoop;
			const iCamera* _pCamera;

			BDPTParam() :
				_pCamera(nullptr), 
				_Through(1, 1, 1),
				_IfLoop(true), _Depth(0),
				_Pos(0, 0, 0),
				_Result(0, 0, 0),
				_NextRay(Vec3(0, 0, 0), Vec3(0, 0, 1)),
				_CurrentRay(Vec3(0, 0, 0), Vec3(0, 0, 1)),
				_PDF(1.0f){
			}
		};

		struct Geometry{
			Vec3 Position;
			Vec3 Normal;
			Vec3 UV;
			Vec3 Albedo;
			Vec3 Emission;
			Vec3 TextureColor[5];
			float Index;
			float Param1;
			float Param2;
			bool DeltaSurface;
			const Attribute* attribute;
		};
		void CalculateGeometry(const AbsG::RF& RF, const Scene& scene, Primitive::Ray ray, Geometry& result)const;

		const int m_ImageWidth;
		const int m_ImageHeight;

		PTUtility::Vec3 TraceRay(
			const Primitive::Ray inRay, const Scene& scene, const iCamera& camera, float* LightImage, RandomMT& MT)const;

		float PDFSteradian_To_PDFArea(
			const PTUtility::Vec3& FromPos, const Primitive::inoutFacet* FromFacet,  
			const PTUtility::Vec3& ToPos, const Primitive::inoutFacet* ToFacet,
			float PDF_Steradian) const{
			float dist2 = (FromPos - ToPos).norm2();
			float ToCos = ((ToFacet->_Normal[0] + ToFacet->_Normal[1] + ToFacet->_Normal[2]).normalized()).dot((FromPos - ToPos).normalized());
			ToCos = std::max(std::min(1.0f, ToCos), 0.0f);
			return ToCos * PDF_Steradian / dist2;
		}

		struct BDPT_PATH_VERTEX {
			PTUtility::Vec3 _Albedo;
			PTUtility::Vec3 _Pos;
			PTUtility::Vec3 _Normal;
			float _LightArea;
			PTUtility::Vec3 _Emission;
			PTUtility::Vec3 _Dir;
			PTUtility::SurfaceMaterial::BRDF_Function _BRDF;
			PTUtility::SurfaceMaterial::PDF_Function _PDFfunction;
			bool _DeltaSurface;
			float _FromIndex;
			float _IntoIndex;
			float _Param1;
			float _Param2;

			PTUtility::Vec3 _Through;
			float _PDF_Path;
			float _PDF_rus_ster;
			bool _Valid;
			BDPT_PATH_VERTEX() : 
				_Albedo(1, 1, 1), _Emission(0, 0, 0), _LightArea(1.0f), _DeltaSurface(false), _FromIndex(1.0f), _IntoIndex(1.0f), _Param1(0.00001), _Param2(0.00001), 
				_Pos(0, 1, 0), _Normal(0, 1, 0), _Dir(1, 0, 0), 
				_Through(1, 1, 1), _PDF_Path(1.0), _PDF_rus_ster(1.0), _Valid(true), _BRDF(nullptr), _PDFfunction(nullptr){
			}
			~BDPT_PATH_VERTEX() {
			};
		};
		struct BV {
			PTUtility::Vec3 _Pos;
			PTUtility::Vec3 _Normal;
			PTUtility::Vec3 _FromEyeDir;
			PTUtility::Vec3 _FromLightDir;
			PTUtility::SurfaceMaterial::BRDF_Function _BRDF;
			PTUtility::SurfaceMaterial::PDF_Function _PDFfunction;
			float _FromIndex;
			float _IntoIndex;
			float _Param1;
			float _Param2;
			bool _DeltaSurface;

			BV() :
				_Pos(0, 1, 0), _Normal(0, 1, 0), _Param1(0.00001), _Param2(0.00001),
				_FromEyeDir(1,0,0), _FromLightDir(1, 0, 0), _FromIndex(1.0f), _IntoIndex(1.0f), 
				_BRDF(nullptr), _PDFfunction(nullptr), _DeltaSurface(false) {
			}
			~BV() {
			};
		};

		BDPT_PATH_VERTEX bdpt(BDPTParam& pm, const Scene& scene, float* LightImage, RandomMT& MT)const;

		bool GenerateLightRay_Sphere_Area(const Scene& scene, Primitive::Ray& result, PTUtility::Vec3& normal, PTUtility::Vec3& LightRadiance, PTUtility::Vec3& LightWeight, float& PDF, RandomMT& MT)const;
		bool GenerateLightRay_Sphere(const Scene& scene, Primitive::Ray& result, PTUtility::Vec3& normal, PTUtility::Vec3& LightRadiance, float& PDF, RandomMT& MT)const;
		bool GenerateLightRay_Area(const Scene& scene, Primitive::Ray& result, PTUtility::Vec3& normal, PTUtility::Vec3& LightRadiance, float& PDF, RandomMT& MT)const;

		PTUtility::Vec3 MargePath(
			const Scene& scene,
			const iCamera& camera,
			const std::vector<BDPT_PATH_VERTEX>& LightSubPath,
			const std::vector<BDPT_PATH_VERTEX>& EyeSubPath,
			const PTUtility::Vec3& LightRadiance, float SampleLight_AreaPDF, 
			float* LightImage, RandomMT& MT)const;

		PTUtility::Vec3 PTUtility::bdPathTracing::CalculatePathC(
			const std::vector<BDPT_PATH_VERTEX>& LightSubPath,
			const std::vector<BDPT_PATH_VERTEX>& EyeSubPath,
			const PTUtility::Vec3& LightRadiance, float SampleLight_AreaPDF, float pixelPDF,
			int S, int T, RandomMT& MT)const;

		PTUtility::Vec3 MIS_Path(
			const std::vector<BV>& Path, 
			const PTUtility::Vec3& LightRadiance, float SampleLight_AreaPDF,
			const PTUtility::Vec3& BaseC, int S, float BasePDF, float PixelPDF, RandomMT& MT)const;

		void AddLightImage(float* Image, const float* LightImages, int NumLightImage) const;
		void NormalizeImage(float* Image, float NumSample) const;
		void Gamma(unsigned char* cImage, const float* Image, float Gamma, float SampleCount = 1.0f) const;

	public:
		void RenderImage(
			const std::string& ImageFileName,
			const iCamera& Camera,
			const Scene& scene,
			int NumSample,
			int NumSuperSample, 
			const PostFilter<unsigned char>* Filter = nullptr)const;

		void RenderImage_seq(int SwitchCount, const std::string& FolderName, 
			const iCamera& Camera, const Scene& scene, int RenderTime_sec, int TimeStep_sec, const PostFilter<unsigned char>* Filter = nullptr)const;

		void WriteImage(
			const float* Image, const float* LightImages, 
			const std::string& FileName, float Samplecount, int NumLightImage, float GammaValue, const PostFilter<unsigned char>* Filter = nullptr)const;
		explicit bdPathTracing(int Width, int Height) : m_ImageWidth(Width), m_ImageHeight(Height){

		}
		~bdPathTracing(){

		}
	};

}