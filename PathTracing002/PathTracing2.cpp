#include"PathTracing.h"
#include"Intersect.h"
#include"stb_image_write.h"
#include"Sampling.h"

using namespace PTUtility;
using namespace Primitive;

void PathTracing2::CalculateGeometry(const AbsG::RF& RF, const Scene& scene, Ray ray, Geometry& result)const{
	const inoutFacet* Facet = RF._Facet;

	result.Position = ray.m_Org + (RF._t)*ray.m_Dir;
	result.Normal = (RF._u * (Facet->_Normal[1] - Facet->_Normal[0]) + RF._v * (Facet->_Normal[2] - Facet->_Normal[0]) + Facet->_Normal[0]).normalized();
	result.UV = RF._u * (Facet->_UV[1] - Facet->_UV[0]) + RF._v * (Facet->_UV[2] - Facet->_UV[0]) + Facet->_UV[0];
	result.Albedo = Vec3(1.0, 0.0, 1.0);
	result.Emission = Vec3(0, 0, 0);
	result.Index = 1.0f;
	result.Param11 = 0.00001;
	result.Param12 = 0.00001;

	short AttributeID = Facet->_AttributeID;
	result.attribute = nullptr;
	const std::vector<std::string>* Textures = nullptr;
	if (AttributeID >= 0){
		result.attribute = scene.GetAttribute(AttributeID);
		result.Albedo = result.attribute->m_Albedo;
		result.Emission = result.attribute->m_Emission;
		result.Index = result.attribute->m_Index;
		result.Param11 = result.attribute->m_Param1;
		result.Param12 = result.attribute->m_Param2;

		Textures = &result.attribute->TextureFileNames;
		const int MAX_TEXTURE = 3;
		Vec3 TextureColor[MAX_TEXTURE];
		for (int i = 0; i < std::min((int)(*Textures).size(), MAX_TEXTURE); ++i){
			result.TextureColor[i] = scene.GetTexture((*Textures)[i])->GetColor(result.UV.x(), 1.0 - result.UV.y());
		}
	}
}

Vec3 PathTracing2::DirectLighting(const Primitive::Ray inRay, const Scene& scene, RandomMT& MT){
	AbsG::RF result;
	int ishit = scene.GetOneFacetFromRay(inRay, result);
	if (ishit > 0){
		Geometry gmt;
		CalculateGeometry(result, scene, inRay, gmt);
		if (gmt.TextureColor[0].norm2() > 0.01){ return gmt.TextureColor[0]; }
		else{return gmt.Albedo; }
	}
	else{
		return Vec3(0.0, 0.0, 0.0);
	}
}

Vec3 PathTracing2::CrossTest(const Primitive::Ray inRay, const Scene& scene, RandomMT& MT){
	AbsG::RF result;
	int ishit = scene.GetOneFacetFromRay(inRay, result);
	if (ishit > 0){
		Geometry gmt;
		CalculateGeometry(result, scene, inRay, gmt);
		return gmt.Albedo * (0.6 + gmt.Normal.y());
	}
	else{
		return Vec3(0.0, 0.0, 0.0);
	}
}

void PathTracing2::Trace(RenderParam& pm, const Scene& scene, RandomMT& MT){
	const int MAX_DEPTH = 16;
	pm._Depth++;

	if (pm._Depth > 100){
		pm._IfLoop = false;
		return;
	}

	//russian roulette
	float rus_pdf = 1.0f;
	if (pm._Depth > MAX_DEPTH){
		rus_pdf = exp(MAX_DEPTH - pm._Depth);
		if (MT.genrand64_real1() > rus_pdf){
			pm._IfLoop = false;
			return;
		}
	}
	pm._Weight /= rus_pdf;

	AbsG::RF result;
	int NumHit = scene.GetOneFacetFromRay(pm._CurrentRay, result);

	if (NumHit > 0){
		//Ray intersects some triangles

		Geometry gmt;
		CalculateGeometry(result, scene, pm._CurrentRay, gmt);
		if (gmt.TextureColor[0].norm2() > 0.01){
			gmt.Albedo = gmt.TextureColor[0];
		}
		Vec3 gmtNormal = gmt.Normal;
		
		float idvIndex = gmt.Index;
		if (gmt.Normal.dot(pm._CurrentRay.m_Dir) > 0) {
			idvIndex = 1.0f;
		}


		//if (pm._Depth == 1){
			//if (pm._CurrentRay.m_Dir.dot(gmt.Normal) < 0) {
				pm._Result += pm._Weight*gmt.Emission;
				if (gmt.Emission.norm2() > 0.001) { pm._IfLoop = false; }
			//}
		//}


		// always gmt.nomal・CurrentRay < 0
		gmt.Normal = gmt.Normal.dot(pm._CurrentRay.m_Dir) > 0 ? -gmt.Normal : gmt.Normal;
		pm._NextRay.m_Org = gmt.Position;


		//{
		//	//Occlusion Test

		//	float pdf = 1.0f;
		//	Vec3 LPos;
		//	scene.GetLightManager()->SamplePosition_AreaLight(pdf, LPos, MT);
		//	float FromLightDist = (LPos - gmt.Position).norm();
		//	Ray FromLightRay(LPos, (gmt.Position - LPos).normalized());

		//	if (scene.IfOccluded(FromLightRay, 0.0f, FromLightDist)) {
		//		pm._Result = Vec3(0, 0, 0);
		//	}
		//	else {
		//		pm._Result = Vec3(4, 4, 4) * FromLightRay.m_Dir.dot(-gmt.Normal);
		//	}
		//	pm._IfLoop = false;
		//	return;
		//}
		

		//{//Next Event Estimation AreaLight
		//	Vec3 NEE_result(0, 0, 0);
		//	float NEE_pdf;
		//	Vec3 toLightDir;
		//	if (scene.NextEventEstimation_AreaLight(pm._NextRay, MT, NEE_result, NEE_pdf, toLightDir) == 1){
		//		pm._Result += pm._Weight * 
		//			(NEE_result * std::max(0.0f, std::min(1.0f, toLightDir.dot(gmt.Normal)))) * 
		//			gmt.Albedo * gmt.attribute->m_BRDF(pm._CurrentRay.m_Dir, toLightDir, gmt.Normal, pm._CurrentRay.m_Index, gmt.Index, gmt.Param11, gmt.Param12);
		//	}
		//}

		float pdf_ster = 1.0f;
		pm._NextRay.m_Dir = gmt.attribute->m_BRDF_Sample(pm._CurrentRay.m_Dir, gmt.Normal, &pdf_ster, MT, pm._CurrentRay.m_Index, idvIndex, gmt.Param11, gmt.Param12);

		// always OutNormal・NextRay > 0
		Vec3 OutNormal = gmt.Normal.dot(pm._NextRay.m_Dir) > 0 ? gmt.Normal : -gmt.Normal;

		pm._NextRay.m_Org += OutNormal * 0.001;
		pm._Weight *= gmt.Albedo;
		float brdf = gmt.attribute->m_BRDF(pm._CurrentRay.m_Dir, pm._NextRay.m_Dir, gmt.Normal, pm._CurrentRay.m_Index, idvIndex, gmt.Param11, gmt.Param12);
		float cos = std::max(std::min(1.0f, pm._NextRay.m_Dir.dot(OutNormal)), 0.0f);
		pm._Weight *= (cos * (brdf / pdf_ster));

		if (gmtNormal.dot(pm._NextRay.m_Dir) < 0) {
			pm._NextRay.m_Index = gmt.Index;
		}
		else {
			pm._NextRay.m_Index = 1.0f;
		}
	}
	else{
		//Ray does not intersect any object
		pm._IfLoop = false;
	}
	pm._CurrentRay = pm._NextRay;
}

Vec3 PathTracing2::TraceRay(const Primitive::Ray inRay, const Scene& scene, RandomMT& MT){
	RenderParam param;
	param._Result = Vec3::Zero();
	param._CurrentRay = inRay;
	param._Weight = Vec3::Ones();

	while (param._IfLoop){
		Trace(param, scene, MT);
	}
	return param._Result;
}

void PathTracing2::RenderImage(
	const std::string& ImageFileName,
	const iCamera& Camera,
	const Scene& scene,
	int NumSample,
	int NumSuperSample){

	time_t st = time(NULL);

	unsigned char* Image = new unsigned char[m_ImageWidth * m_ImageHeight * 4];
	for (int i = 0; i < 4 * m_ImageWidth*m_ImageHeight; i++){
		Image[i] = 0;
	}

	Vec3 Camera_E1 = Camera.Get_E1().normalized();
	Vec3 Camera_E2 = Camera.Get_E2().normalized();

	SuperSampling::HaltonSampling SSample(Camera_E1, Camera_E2, time(NULL), NumSuperSample + 5);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
	for (int i = 0; i < (m_ImageWidth)* (m_ImageHeight); i++){
		RandomMT MT(i + time(NULL) * 1937);

		int w = i % m_ImageWidth;
		int h = m_ImageHeight - 1 - i / m_ImageWidth;

		float uu = w / (float)m_ImageWidth;
		float vv = h / (float)m_ImageHeight;

		Vec3 Color(0, 0, 0);
		for (int ss = 0; ss < NumSuperSample; ss++){

			SuperSampling::iSuperSampling::Input SIN;
			SIN._i[0] = ss;
			Vec3 RV = SSample.GetPosition_Square(SIN);
			RV *= Vec3(Camera.GetPlaneWidth() / (float)m_ImageWidth, Camera.GetPlaneHeight()/ (float)m_ImageHeight, 0);
			Ray r(Camera.GetCameraPos(), Camera.GeneratePos(uu, vv) + RV - Camera.GetCameraPos());
			for (int s = 0; s < NumSample; s++){
				Color += TraceRay(r, scene, MT);
			}
		}
		Color /= (float)(NumSample*NumSuperSample);


		Image[4 * i + 0] = std::max(std::min(0.999f, Color.x()), 0.0f) * 255;
		Image[4 * i + 1] = std::max(std::min(0.999f, Color.y()), 0.0f) * 255;
		Image[4 * i + 2] = std::max(std::min(0.999f, Color.z()), 0.0f) * 255;
		Image[4 * i + 3] = 255;

		fprintf(stderr, "\r レイトレェ。。。 %5.2f%%", 100.0*i / ((float)m_ImageWidth * m_ImageHeight));
	}

	const float Gamma = 2.0;
	//Gamma
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
	for (int i = 0; i < (m_ImageWidth)* (m_ImageHeight); i++){
		Image[4 * i + 0] = std::pow(Image[4 * i + 0] / 255.0f, 1.0f / Gamma) * 255;
		Image[4 * i + 1] = std::pow(Image[4 * i + 1] / 255.0f, 1.0f / Gamma) * 255;
		Image[4 * i + 2] = std::pow(Image[4 * i + 2] / 255.0f, 1.0f / Gamma) * 255;
		Image[4 * i + 3] = 255;
	}

	printf("\n　ぱすとれしゅーりょ \n");
	stbi_write_bmp(ImageFileName.c_str(), m_ImageWidth, m_ImageHeight, 4, Image);

	printf("\n");
	time_t ed = time(NULL);
	int rtm = int(ed) - int(st);
	int th, tm, ts;
	th = rtm / 3600;
	tm = (rtm - th * 3600) / 60;
	ts = rtm - 3600 * th - 60 * tm;

	printf("経過　%d時間 %d分　%d秒\n", th, tm, ts);

	delete[] Image;
}