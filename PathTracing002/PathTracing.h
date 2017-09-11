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
#include<time.h>
#include<string>

namespace PTUtility{
	class PathTracing2{
	private:

		//Rendering Parameter
		struct RenderParam{
			Vec3 _Normal;
			Vec3 _Pos;
			Vec3 _Result;
			Primitive::Ray _NextRay;
			Primitive::Ray _CurrentRay;
			Vec3 _Weight;
			int _Depth;
			bool _IfLoop;
			bool _IfDoneNEE;

			enum Ray_Stat{
				INSIDE_TRANSLUCENT = 0,
				OUTSIDE_TRANSLUCENT = 1,
			};
			Ray_Stat _Stat;
			float _Index;

			RenderParam() :
				_Weight(1, 1, 1),
				_IfLoop(true), _Depth(0),
				_Normal(0, 0, 1),
				_Pos(0, 0, 0),
				_Result(0, 0, 0),
				_NextRay(Vec3(0, 0, 0), Vec3(0, 0, 1)),
				_CurrentRay(Vec3(0, 0, 0), Vec3(0, 0, 1)),
				_Stat(OUTSIDE_TRANSLUCENT),
				_Index(1.0000f),
				_IfDoneNEE(false){
			}
		};
		struct Geometry{
			Vec3 Position;
			Vec3 Normal;
			Vec3 UV;
			Vec3 Albedo;
			Vec3 Emission;
			float Param11;
			float Param12;
			Vec3 TextureColor[5];
			const Attribute* attribute;
			float Index;
		};
		void CalculateGeometry(const AbsG::RF& RF, const Scene& scene, Primitive::Ray ray, Geometry& result)const;

		const int m_ImageWidth;
		const int m_ImageHeight;

		PTUtility::Vec3 TraceRay(const Primitive::Ray inRay, const Scene& scene, RandomMT& MT);
		void Trace(RenderParam& pm, const Scene& scene, RandomMT& MT);

		PTUtility::Vec3 DirectLighting(const Primitive::Ray inRay, const Scene& scene, RandomMT& MT);
		PTUtility::Vec3 CrossTest(const Primitive::Ray inRay, const Scene& scene, RandomMT& MT);

	public:
		void RenderImage(
			const std::string& ImageFileName,
			const iCamera& Camera,
			const Scene& scene,
			int NumSample,
			int NumSuperSample);

		explicit PathTracing2(int Width, int Height) : m_ImageWidth(Width), m_ImageHeight(Height){

		}
		~PathTracing2(){

		}
	};

}