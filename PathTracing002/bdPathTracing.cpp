#include"bdPathTracing.h"
#include"PathTracing.h"
#include"Intersect.h"
#include"stb_image_write.h"
#include"Sampling.h"
#include"RGB_HSV.h"
#include<omp.h>
#include<sstream>
#include<utility>
#include<vector>

using namespace PTUtility;
using namespace Primitive;

void bdPathTracing::CalculateGeometry(const AbsG::RF& RF, const Scene& scene, Ray ray, Geometry& result)const{
	const inoutFacet* Facet = RF._Facet;

	result.Position = ray.m_Org + (RF._t)*ray.m_Dir;
	result.Normal = (RF._u * (Facet->_Normal[1] - Facet->_Normal[0]) + RF._v * (Facet->_Normal[2] - Facet->_Normal[0]) + Facet->_Normal[0]).normalized();
	result.UV = RF._u * (Facet->_UV[1] - Facet->_UV[0]) + RF._v * (Facet->_UV[2] - Facet->_UV[0]) + Facet->_UV[0];
	result.Albedo = Vec3(1.0, 0.0, 1.0);
	result.Emission = Vec3(0, 0, 0);
	result.Index = 1.0f;
	result.DeltaSurface = false;
	result.Param1 = 0.00001;
	result.Param2 = 0.00001;


	short AttributeID = Facet->_AttributeID;
	result.attribute = nullptr;
	const std::vector<std::string>* Textures = nullptr;
	if (AttributeID >= 0){
		
		result.attribute = scene.GetAttribute(AttributeID);
		result.Albedo = result.attribute->m_Albedo;
		result.Emission = result.attribute->m_Emission;
		result.Index = result.attribute->m_Index;
		result.DeltaSurface = result.attribute->m_DeltaSurface;
		result.Param1 = result.attribute->m_Param1;
		result.Param2 = result.attribute->m_Param2;

		Textures = &result.attribute->TextureFileNames;
		const int MAX_TEXTURE = 3;
		Vec3 TextureColor[MAX_TEXTURE];
		for (int i = 0; i < std::min((int)(*Textures).size(), MAX_TEXTURE); ++i){
			result.TextureColor[i] = scene.GetTexture((*Textures)[i])->GetColor(result.UV.x(), 1.0 - result.UV.y());
		}
	}
}

bdPathTracing::BDPT_PATH_VERTEX bdPathTracing::bdpt(BDPTParam& pm, const Scene& scene, float* LightImage, RandomMT& MT) const {
	pm._Depth++;
	BDPT_PATH_VERTEX bdptV;

	//russian roulette
	float rus_pdf = Russian_PDF(pm._Depth);
	if (MT.genrand64_real1() > rus_pdf) {
		pm._IfLoop = false;
		bdptV._Valid = false;
		return bdptV;
	}
	pm._PDF *= rus_pdf;

	AbsG::RF result;
	int NumHit = scene.GetOneFacetFromRay(pm._CurrentRay, result);

	if (NumHit > 0) {
		//Ray intersects some triangles

		Geometry gmt;
		CalculateGeometry(result, scene, pm._CurrentRay, gmt);
		if (gmt.TextureColor[0].norm2() > 0.01) {
			gmt.Albedo = gmt.TextureColor[0];
		}

		float idvIndex = gmt.Index;
		if (gmt.Normal.dot(pm._CurrentRay.m_Dir) > 0) {
			idvIndex = 1.0f;
		}

		Vec3 gmtNormal = gmt.Normal;
		// always gmt.nomal・CurrentRay < 0
		gmt.Normal = gmt.Normal.dot(pm._CurrentRay.m_Dir) > 0 ? -gmt.Normal : gmt.Normal;


		pm._NextRay.m_Org = gmt.Position;
		float pdf = 1.0;

		pm._Through *= gmt.Albedo;

		float pdf_ster = 1.0f;
		pm._NextRay.m_Dir = gmt.attribute->m_BRDF_Sample(pm._CurrentRay.m_Dir, gmt.Normal, &pdf_ster, MT, pm._CurrentRay.m_Index, idvIndex, gmt.Param1, gmt.Param2);
				
		// always OutNormal・NextRay > 0
		Vec3 OutNormal = gmt.Normal.dot(pm._NextRay.m_Dir) > 0 ? gmt.Normal : -gmt.Normal;

		pm._NextRay.m_Org += OutNormal * 0.000001;
		
		float brdf = gmt.attribute->m_BRDF(pm._CurrentRay.m_Dir, pm._NextRay.m_Dir, gmt.Normal, pm._CurrentRay.m_Index, idvIndex, gmt.Param1, gmt.Param2);
		float cos = std::max(std::min(1.0f, pm._NextRay.m_Dir.dot(OutNormal)), 0.0f);
		pm._Through *= (brdf * cos);
		pm._PDF *= pdf_ster;

		bdptV._PDF_rus_ster = pdf_ster * (rus_pdf);
		bdptV._BRDF = gmt.attribute->m_BRDF;
		bdptV._Dir = pm._CurrentRay.m_Dir;
		bdptV._PDFfunction = gmt.attribute->m_BRDF_PDF;
		bdptV._PDF_Path = pm._PDF;
		bdptV._Normal = gmtNormal;
		bdptV._Pos = gmt.Position;
		bdptV._Through = pm._Through;
		bdptV._Albedo = gmt.Albedo;
		bdptV._FromIndex = pm._CurrentRay.m_Index;
		bdptV._DeltaSurface = gmt.DeltaSurface;
		bdptV._Param1 = gmt.Param1;
		bdptV._Param2 = gmt.Param2;


		bdptV._Emission = gmt.Emission;
		if (gmt.Emission.norm2() > 0) {
			bdptV._LightArea = result._Facet->CalculateArea();
		}

		if (gmtNormal.dot(pm._NextRay.m_Dir) < 0) {
			pm._NextRay.m_Index = gmt.Index;
		}
		else {
			pm._NextRay.m_Index = 1.0f;
		}
		bdptV._IntoIndex = pm._NextRay.m_Index;
	}
	else {
		//Ray does not intersect any object
		pm._IfLoop = false;
		bdptV._Valid = false;
		return bdptV;
	}

	pm._CurrentRay = pm._NextRay;
	return bdptV;
}

Vec3 bdPathTracing::TraceRay(const Primitive::Ray inRay, const Scene& scene, const iCamera& camera, float* LightImage, RandomMT& MT) const {
	
	//Generate Eye Subpath
	std::vector<BDPT_PATH_VERTEX> EyeSubpath;
	{
		BDPTParam param;
		param._Result = Vec3::Zero();
		param._CurrentRay = inRay;
		param._PDF = 1.0;

		while (param._IfLoop) {
			BDPT_PATH_VERTEX vv;
			vv = bdpt(param, scene, LightImage, MT);
			if (vv._Valid) {
				EyeSubpath.push_back(vv);
			}
		}
	}

	//Generate Light Subpath
	std::vector<BDPT_PATH_VERTEX> LightSubpath;
	Vec3 LightRadiance(0, 0, 0);
	Vec3 Lnormal;
	float LightPDF = 1.0f;
	{
		Primitive::Ray FromLight(Vec3(0, 0, 0), Vec3(0, 1, 0));
		GenerateLightRay_Area(scene, FromLight, Lnormal, LightRadiance, LightPDF, MT);

		BDPT_PATH_VERTEX onLight;
		onLight._Normal = Lnormal;
		onLight._PDFfunction = LightSurface::PDF;
		
		onLight._BRDF = LightSurface::Light;
		onLight._Pos = FromLight.m_Org;
		onLight._PDF_Path = LightPDF / (2.0f * PI);
		onLight._Through = LightRadiance * onLight._Normal.dot(FromLight.m_Dir);
		onLight._PDF_rus_ster = onLight._PDF_Path;
		LightSubpath.push_back(onLight);

		BDPTParam param;
		param._Result = Vec3::Zero();
		param._CurrentRay = FromLight;
		param._PDF = onLight._PDF_Path;
		param._Through = onLight._Through;

		while (param._IfLoop) {
			BDPT_PATH_VERTEX vv;
			vv = bdpt(param, scene, LightImage, MT);
			if (vv._Valid) {
				LightSubpath.push_back(vv);
			}
		}
	}
	return MargePath(scene, camera, LightSubpath, EyeSubpath, LightRadiance, LightPDF, LightImage, MT);
}

Vec3 bdPathTracing::MargePath(
	const Scene& scene,
	const iCamera& camera, 
	const std::vector<BDPT_PATH_VERTEX>& LightSubPath,
	const std::vector<BDPT_PATH_VERTEX>& EyeSubPath,
	const PTUtility::Vec3& LightRadiance, float SampleLight_AreaPDF, float* LightImage, RandomMT& MT) const {

	int count = 0;
	float wcount = -0.0001;
	Vec3 Result(0, 0, 0);

	//connect Camera position
	for (auto L = LightSubPath.begin() + 1; L != LightSubPath.end(); ++L) {
		
		if (L->_DeltaSurface) {
			continue;
		}

		const float OFFSET = 0.001;
		Vec3 LtoCamera = (camera.GetCameraPos() - L->_Pos).normalized();
		Primitive::Ray LtoC = Primitive::Ray(L->_Pos + LtoCamera * OFFSET, LtoCamera);
		float dist = (L->_Pos - camera.GetCameraPos()).norm();
		float inv_dist2 = std::pow(dist, -2);
		if (scene.IfOccluded(LtoC, 0.0f, dist)) {
			continue;
		}

		float u, v = 0.0f;
		Vec3 pp = camera.GetPixelFromPosition(L->_Pos, u, v);
		if (std::abs(u) < 0.5f && std::abs(v) < 0.5f){
			int IX = (u + 0.5f) * m_ImageWidth;
			int IY = (-v + 0.5f) * m_ImageHeight;
			int ImageIDX = (IX + IY * m_ImageWidth) * 4;
			float* Pixel = LightImage + ImageIDX;

			float inv_dist2 = 1.0f / (dist * dist);
			BDPT_PATH_VERTEX Camera;
			Camera._Pos = camera.GetCameraPos();
			Camera._PDF_Path = 1.0f;
			std::vector<BDPT_PATH_VERTEX> ea;
			ea.push_back(Camera);

			float pixelPDF = camera.Pixel_PDFstr(L->_Pos, L->_Normal, m_ImageWidth, m_ImageHeight);
			Vec3 Col = CalculatePathC(LightSubPath, ea, LightRadiance, SampleLight_AreaPDF, pixelPDF, -1, L - LightSubPath.begin(), MT);

			if (Col.norm2() > Sampling::sMin) {
				*(Pixel + 0) += Col.x(); *(Pixel + 1) += Col.y(); *(Pixel + 2) += Col.z(); *(Pixel + 3) += 1.0f;
			}
		}
	}
	
	if (EyeSubPath.size() > 0) {

	float pixelPDF = camera.Pixel_PDFstr(EyeSubPath[0]._Pos, EyeSubPath[0]._Normal, m_ImageWidth, m_ImageHeight);

		//bidirectional?
		for (auto L = LightSubPath.begin(); L != LightSubPath.end(); ++L) {
			for (auto E = EyeSubPath.begin(); E != EyeSubPath.end(); ++E) {

				int EyeLength = (E - EyeSubPath.begin());
				int LightLength = (L - LightSubPath.begin());
				int PathLength = 1 + (EyeLength + 1) + LightLength;

				float connectPDF = std::min(1.0f, std::pow(0.3f, (PathLength - RDepth())));
				if (MT.genrand64_real1() > connectPDF) {
					continue;
				}

				if (E->_DeltaSurface || L->_DeltaSurface) {
					continue;
				}
				if (std::abs(E->_IntoIndex - L->_IntoIndex) > 0.001) {
					continue;
				}

				const float OFFSET = 0.001;
				Vec3 EtoLDir = (L->_Pos - E->_Pos).normalized();
				Primitive::Ray EtoL = Primitive::Ray(E->_Pos + EtoLDir * OFFSET, EtoLDir);
				float dist = (L->_Pos - E->_Pos).norm();
				if (dist < 0.00001) {
					continue;
				}
				if (scene.IfOccluded(EtoL, 0.0f, dist)) {
					continue;
				}

				Vec3 cal = CalculatePathC(LightSubPath, EyeSubPath, LightRadiance, SampleLight_AreaPDF, pixelPDF, EyeLength, LightLength, MT) / connectPDF;
				if (cal.norm2() > Sampling::sMin) {
					Result += cal;
				}
			}
		}

		//standard path tracing
		for (auto E = EyeSubPath.begin(); E != EyeSubPath.end(); ++E) {
			if (E->_Emission.norm2() > 0.00001) {
				int EyeLength = (E - EyeSubPath.begin());
				int PathLength = EyeLength + 1;
				float sampleLightPDF = (E->_Emission.x() + E->_Emission.y() + E->_Emission.z()) / scene.GetLightManager()->GetLightWeightTotal();
				Vec3 cal = CalculatePathC(LightSubPath, EyeSubPath, E->_Emission, sampleLightPDF, pixelPDF, EyeLength, -1, MT);
				if (cal.norm2() > Sampling::sMin) {
					Result += cal;
				}
			}
		}
	}
	return Result;
}

PTUtility::Vec3 PTUtility::bdPathTracing::CalculatePathC(
	const std::vector<BDPT_PATH_VERTEX>& LightSubPath, 
	const std::vector<BDPT_PATH_VERTEX>& EyeSubPath, 
	const PTUtility::Vec3& LightRadiance, float SampleLight_AreaPDF, float pixelPDF, 
	int S, int T, RandomMT& MT)const
{
	Vec3 Result;

	std::vector<BV> BV_Array(S + T + 4);
	//discribe camera
	BV_Array[0]._BRDF = nullptr;
	BV_Array[0]._PDFfunction = nullptr;
	BV_Array[0]._Pos = EyeSubPath[0]._Pos - EyeSubPath[0]._Dir;

	for (int i = 0; i < S + 1; ++i) {
		BV_Array[i + 1]._BRDF = EyeSubPath[i]._BRDF;
		BV_Array[i + 1]._PDFfunction = EyeSubPath[i]._PDFfunction;
		BV_Array[i + 1]._Normal = EyeSubPath[i]._Normal;
		BV_Array[i + 1]._Pos = EyeSubPath[i]._Pos;
		BV_Array[i + 1]._FromEyeDir = EyeSubPath[i]._Dir;
		BV_Array[i + 1]._FromIndex = EyeSubPath[i]._FromIndex;
		BV_Array[i + 1]._IntoIndex = EyeSubPath[i]._IntoIndex;
		BV_Array[i + 1]._DeltaSurface = EyeSubPath[i]._DeltaSurface;
		BV_Array[i + 1]._Param1 = EyeSubPath[i]._Param1;
		BV_Array[i + 1]._Param2 = EyeSubPath[i]._Param2;
	}
	for (int i = 0; i < T + 1; ++i) {
		BV_Array[i + 2 + S]._BRDF = LightSubPath[T - i]._BRDF;
		BV_Array[i + 2 + S]._PDFfunction = LightSubPath[T - i]._PDFfunction;
		BV_Array[i + 2 + S]._Normal = LightSubPath[T - i]._Normal;
		BV_Array[i + 2 + S]._Pos = LightSubPath[T - i]._Pos;
		BV_Array[i + 2 + S]._FromLightDir = LightSubPath[T - i]._Dir;
		BV_Array[i + 2 + S]._FromIndex = LightSubPath[T - i]._FromIndex;
		BV_Array[i + 2 + S]._IntoIndex = LightSubPath[T - i]._IntoIndex;
		BV_Array[i + 2 + S]._DeltaSurface = LightSubPath[T - i]._DeltaSurface;
		BV_Array[i + 2 + S]._Param1 = LightSubPath[T - i]._Param1;
		BV_Array[i + 2 + S]._Param2 = LightSubPath[T - i]._Param2;
	}

	//dummy vertex
	BV_Array[T + 3 + S]._BRDF = nullptr;
	BV_Array[T + 3 + S]._PDFfunction = nullptr;
	BV_Array[T + 3 + S]._Pos = LightSubPath[0]._Pos - Vec3(1, 0, 0);

	for (int i = 1; i < BV_Array.size() - 1; ++i) {
		BV_Array[i]._FromEyeDir = (BV_Array[i]._Pos - BV_Array[i - 1]._Pos).normalized();
		BV_Array[i]._FromLightDir = (BV_Array[i]._Pos - BV_Array[i + 1]._Pos).normalized();
	}


	if (T >= 0 && S >= 0) {
		int Path_Length = S + T;
		auto E = EyeSubPath.begin() + S;
		auto L = LightSubPath.begin() + T;

		Primitive::Ray EtoL = Primitive::Ray(E->_Pos, (L->_Pos - E->_Pos).normalized());
		float dist = (L->_Pos - E->_Pos).norm();
		if (dist < 0.000001) {
			return Vec3::Zero();
		}
		float inv_dist2 = std::pow(dist, -2);
		float Lcos = std::max(0.0f, std::min(1.0f, L->_Normal.dot(-EtoL.m_Dir)));
		float Ecos = std::max(0.0f, std::min(1.0f, E->_Normal.dot(EtoL.m_Dir)));
		float Gfactor = inv_dist2 * (Lcos) * (Ecos);

		Vec3 LightC;
		Vec3 EyeC;
		if (T > 0) {
			LightC = (L - 1)->_Through / (L - 1)->_PDF_Path;
		}
		else {
			LightC = LightRadiance / SampleLight_AreaPDF;
		}
		if (S == 0) {
			EyeC = Vec3::Ones();
		}
		else {
			EyeC = (E - 1)->_Through / (E - 1)->_PDF_Path;
		}

		float BRDF1 = E->_BRDF(E->_Dir, EtoL.m_Dir, E->_Normal, E->_FromIndex, E->_IntoIndex, E->_Param1, E->_Param2);
		float BRDF2 = L->_BRDF(L->_Dir, -EtoL.m_Dir, L->_Normal, L->_FromIndex, L->_IntoIndex, L->_Param1, L->_Param2);
		Vec3 Path_c = EyeC * LightC * (Gfactor * BRDF1 * BRDF2 * E->_Albedo * L->_Albedo);
		Result = MIS_Path(BV_Array, LightRadiance, SampleLight_AreaPDF, Path_c, S, 1.0f, 1, MT);
	}
	else if (T == -1) {
		auto E = EyeSubPath.begin() + S;
		if (S == 0) {
			return E->_Emission;
		}
		else {
			Vec3 Path_c = (E->_Emission) * (E - 1)->_Through / (E - 1)->_PDF_Path;
			Result = MIS_Path(BV_Array, LightRadiance, SampleLight_AreaPDF, Path_c, S, 1.0f, 1, MT);
		}
	}
	else if (S == -1) {
		int Path_Length = S + T;
		auto E = EyeSubPath.begin();
		auto L = LightSubPath.begin() + T;

		float inv_dist2 = 1.0f / (L->_Pos - E->_Pos).norm2();
		Vec3 LtoCamera = (E->_Pos - L->_Pos).normalized();
		float Lcos = std::max(0.0f, std::min(1.0f, L->_Normal.dot(LtoCamera)));
		Vec3 Path_c;
		if (L == LightSubPath.begin()) {
			Path_c = (LightRadiance / SampleLight_AreaPDF) * Lcos * inv_dist2;
		}
		else {
			Vec3 Normal = L->_Dir.dot(L->_Normal) > 0 ? -L->_Normal : L->_Normal;
			float BRDF = L->_BRDF(L->_Dir, LtoCamera, Normal, L->_FromIndex, L->_IntoIndex, L->_Param1, L->_Param2);
			Path_c = BRDF * L->_Albedo * ((L - 1)->_Through / (L - 1)->_PDF_Path) * pixelPDF;
		}
		Result = MIS_Path(BV_Array, LightRadiance, SampleLight_AreaPDF, Path_c, S, 1.0f, 1, MT);
	}
	return Result;
}

PTUtility::Vec3 PTUtility::bdPathTracing::MIS_Path(
	const std::vector<BV>& Path,
	const PTUtility::Vec3 & LightRadiance, float SampleLight_AreaPDF,
	const PTUtility::Vec3 & BaseC, int S, float BasePDF, float PixelPDF, RandomMT& MT) const
{
	const float p = 2.0f;

	float Bp = 1.0f;
	float NumPath = Bp;

	bool fg = false;

	int fg_delta = 1;
	for (int i = 1; i < S + 2; ++i) {
		const BV* N2 = &Path[(S + 2 - i)];
		const BV* N1 = N2 - 1;
		const BV* N3 = N2 + 1;

		int EyeLength = S + 2 - i;
		int LightLength = Path.size() - EyeLength - 1;

		float dist2_N12 = (N1->_Pos - N2->_Pos).norm2();
		float dist2_N23 = (N2->_Pos - N3->_Pos).norm2();
		float PDF_old = 1.0f;
		if (i == S + 1) {
			PDF_old = PixelPDF;
		}
		else {
			Vec3 Normal = N1->_Normal.dot(N1->_FromEyeDir) > 0.0 ? -N1->_Normal : N1->_Normal;
			float PDF_Str1 = N1->_PDFfunction(N1->_FromEyeDir, -N1->_FromLightDir, Normal, N1->_FromIndex, N1->_IntoIndex, N1->_Param1, N1->_Param2, 1);
			PDF_old = PDF_Str1 * std::abs(N2->_Normal.dot(-N2->_FromEyeDir) / dist2_N12) * Russian_PDF(EyeLength) * Russian_PDF(LightLength);
		}

		float PDF_new = 1.0f;
		if (S + 2 - i == Path.size() - 2) {
			PDF_new = SampleLight_AreaPDF * Russian_PDF(EyeLength);
		}
		else {
			Vec3 Normal = N3->_Normal.dot(N3->_FromLightDir) > 0.0 ? -N3->_Normal : N3->_Normal;
			float PDF_v_ster = N3->_PDFfunction(N3->_FromLightDir, -N3->_FromEyeDir, Normal, N3->_IntoIndex, N3->_FromIndex, N3->_Param1, N3->_Param2, 1);
			PDF_new = PDF_v_ster * std::abs(N2->_Normal.dot(-(N2->_FromLightDir)) / dist2_N23) * Russian_PDF(EyeLength - 1) * Russian_PDF(LightLength + 1);
		}
		Bp *= PDF_new / PDF_old;

		fg_delta = 1;
		if (N1->_DeltaSurface){
			fg_delta = -1;
		}
		if (N2->_DeltaSurface){
			fg_delta = -1;
		}
		if (fg_delta > 0){
			NumPath += std::abs(pow(Bp, p));
			fg = true;
		}
	}

	fg_delta = 1;
	Bp = 1.0f;
	for (int i = S + 2; i < Path.size() - 1; ++i) {
		const BV* N2 = &Path[i];
		const BV* N1 = N2 - 1;
		const BV* N3 = N2 + 1;

		int EyeLength = i;
		int LightLength = Path.size() - i;

		float dist2_N12 = (N1->_Pos - N2->_Pos).norm2();
		float dist2_N23 = (N2->_Pos - N3->_Pos).norm2();
		
		float PDF_old = 1.0f;
		if (i == Path.size() - 2) {
			PDF_old = SampleLight_AreaPDF;
		}
		else {
			Vec3 Normal = N3->_Normal.dot(N3->_FromLightDir) > 0.0 ? -N3->_Normal : N3->_Normal;
			float PDF_Str1 = N3->_PDFfunction(N3->_FromLightDir, -N3->_FromEyeDir, Normal, N3->_IntoIndex, N3->_FromIndex, N3->_Param1, N3->_Param2, 1);
			PDF_old = PDF_Str1 * std::abs(N2->_Normal.dot(-N2->_FromLightDir) / dist2_N23) * Russian_PDF(EyeLength) * Russian_PDF(LightLength);
		}

		float PDF_new = 1.0f;
		if (i == 1) {
			PDF_new = PixelPDF;
		}
		else {
			Vec3 Normal = N1->_Normal.dot(N1->_FromEyeDir) > 0.0 ? -N1->_Normal : N1->_Normal;
			float PDF_v_ster = N1->_PDFfunction(N1->_FromEyeDir, -N1->_FromLightDir, Normal, N1->_FromIndex, N1->_IntoIndex, N1->_Param1, N1->_Param2, 1);
			PDF_new = PDF_v_ster * std::abs(N2->_Normal.dot(-(N2->_FromEyeDir)) / dist2_N12) * Russian_PDF(EyeLength + 1) * Russian_PDF(LightLength - 1);
		}
		Bp *= PDF_new / PDF_old;
		
		fg_delta = 1;
		if (N2->_DeltaSurface) {
			fg_delta = -1;
		}
		if (N3->_DeltaSurface) {
			fg_delta = -1;
		}
		if (fg_delta > 0) {
			NumPath += std::abs(pow(Bp, p));
			fg = true;
		}
	}

	//return BaseC;
	return BaseC / NumPath;
	//return 0.1 * Convert__RGB_HSV::GetGradation((S) / (float)(Path.size())) / NumPath;
}

bool bdPathTracing::GenerateLightRay_Sphere(const Scene& scene, Primitive::Ray& result, PTUtility::Vec3& normal, PTUtility::Vec3& LightRadiance, float& PDF, RandomMT& MT)const {
	const LightManager::SphereLight* pSL = nullptr;
	Vec3 pos;
	pSL = scene.GetLightManager()->SamplePosition_SphereLight(PDF, pos, MT);
	if (pSL == nullptr){
		return false;
	}
	normal = (pos - pSL->_Position).normalized();
	Vec3 LightDir = Sampling::HemSphere_Sampling(MT, normal, nullptr);
	result = Primitive::Ray(pos + normal * 0.00001, LightDir);
	LightRadiance = (pSL->_Flux / (4.0f * PI * pSL->_Radius * pSL->_Radius));
	return true;
}

bool bdPathTracing::GenerateLightRay_Area(
	const Scene& scene, Primitive::Ray& result, PTUtility::Vec3& normal, PTUtility::Vec3& LightRadiance, float& PDF, RandomMT& MT)const {
	const LightManager::AreaLight* pAL = nullptr;
	Vec3 pos;
	pAL = scene.GetLightManager()->SamplePosition_AreaLight(PDF, pos, MT);
	if (pAL == nullptr){
		return false;
	}
	normal = (pAL->_Facet._Normal[0] + pAL->_Facet._Normal[1] + pAL->_Facet._Normal[2]).normalized();
	
	if (pAL->_BiDirectional) {
		int fg = (MT.genrand64_real1() > 0.5 ? -1 : 1);
		normal *= fg;

		Vec3 LightDir = Sampling::HemSphere_Sampling(MT, normal, nullptr);
		result = Primitive::Ray(pos + normal * 0.00002, LightDir);
		LightRadiance = pAL->_Radiance;
		return true;
	}
	else {
		Vec3 LightDir = Sampling::HemSphere_Sampling(MT, normal, nullptr);
		result = Primitive::Ray(pos, LightDir);
		LightRadiance = pAL->_Radiance;
		return true;
	}
}


bool bdPathTracing::GenerateLightRay_Sphere_Area(
	const Scene& scene, Primitive::Ray& result, PTUtility::Vec3& normal, PTUtility::Vec3& LightWeight,
	PTUtility::Vec3& LightRadiance, float& PDF, RandomMT& MT)const {
	bool ret = false;
	if (MT.genrand64_real1() < 1.0){
		ret = GenerateLightRay_Area(scene, result, normal, LightWeight, PDF, MT);
	}
	else{
		ret = GenerateLightRay_Sphere(scene, result, normal, LightWeight, PDF, MT);
	}
	LightRadiance = LightWeight * PDF;
	return ret;
}

void bdPathTracing::AddLightImage(float* Image, const float* LightImages, int NumLightImage) const {
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
	for (int i = 0; i < (m_ImageWidth) * (m_ImageHeight); i++) {
		Vec3 Tol;
		for (int s = 0; s < NumLightImage; ++s) {
			Tol[0] += LightImages[(4 * i + 0) + s * m_ImageWidth*m_ImageHeight * 4];
			Tol[1] += LightImages[(4 * i + 1) + s * m_ImageWidth*m_ImageHeight * 4];
			Tol[2] += LightImages[(4 * i + 2) + s * m_ImageWidth*m_ImageHeight * 4];
		}
		float val[3];
		for (int j = 0; j < 3; ++j) {
			Image[i * 4 + j] = (Image[4 * i + j] + Tol[j]);
		}
	}
}

void bdPathTracing::NormalizeImage(float* Image, float NumSample) const {
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
	for (int i = 0; i < (m_ImageWidth) * (m_ImageHeight); i++) {
		for (int j = 0; j < 3; ++j) {
			Image[i * 4 + j] = Image[i * 4 + j] / (float)NumSample;
		}
	}
}

void bdPathTracing::Gamma(unsigned char* cImage, const float* Image, float Gamma, float SampleCount) const {
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
	for (int i = 0; i < (m_ImageWidth) * (m_ImageHeight); i++) {
		cImage[4 * i + 0] = std::min(255, std::max(0, (int)(std::pow(Image[4 * i + 0] / SampleCount, 1.0f / Gamma) * 255)));
		cImage[4 * i + 1] = std::min(255, std::max(0, (int)(std::pow(Image[4 * i + 1] / SampleCount, 1.0f / Gamma) * 255)));
		cImage[4 * i + 2] = std::min(255, std::max(0, (int)(std::pow(Image[4 * i + 2] / SampleCount, 1.0f / Gamma) * 255)));
		cImage[4 * i + 3] = 255;
	}
}


void bdPathTracing::RenderImage(
	const std::string& ImageFileName,
	const iCamera& Camera,
	const Scene& scene,
	int NumSample,
	int NumSuperSample, const PostFilter<unsigned char>* Filter) const{

	time_t st = time(NULL);

	float* fImage = new float[m_ImageWidth * m_ImageHeight * 4];
	for (int i = 0; i < 4 * m_ImageWidth*m_ImageHeight; i++) {
		fImage[i] = 0.0f;
	}

	Vec3 Camera_E1 = Camera.Get_E1().normalized();
	Vec3 Camera_E2 = Camera.Get_E2().normalized();

	SuperSampling::HaltonSampling SSample(Camera_E1, Camera_E2, time(NULL), NumSuperSample + 5);

	int Num_Threads = omp_get_max_threads();

	float* LImages = new float[m_ImageWidth * m_ImageHeight * 4 * Num_Threads];
	for (int i = 0; i < 4 * Num_Threads * m_ImageWidth*m_ImageHeight; i++) {
		LImages[i] = 0.0f;
	}

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
	for (int i = 0; i < (m_ImageWidth)* (m_ImageHeight); i++) {
		RandomMT MT(i + time(NULL) * 1937);

		int w = i % m_ImageWidth;
		int h = m_ImageHeight - 1 - i / m_ImageWidth;

		float uu = w / (float)m_ImageWidth;
		float vv = h / (float)m_ImageHeight;

		int ThreadID = omp_get_thread_num();

		Vec3 Color(0, 0, 0);
		for (int ss = 0; ss < NumSuperSample; ss++) {

			SuperSampling::iSuperSampling::Input SIN;
			SIN._i[0] = ss;
			Vec3 RV = SSample.GetPosition_Square(SIN);
			RV *= Vec3(Camera.GetPlaneWidth() / (float)m_ImageWidth, Camera.GetPlaneHeight() / (float)m_ImageHeight, 0);
			Ray r(Camera.GetCameraPos(), Camera.GeneratePos(uu, vv) + RV - Camera.GetCameraPos());
			for (int s = 0; s < NumSample; s++) {
				Color += TraceRay(r, scene, Camera, LImages + m_ImageWidth*m_ImageHeight * 4 * ThreadID, MT);
			}
		}
		fImage[4 * i + 0] += Color.x();
		fImage[4 * i + 1] += Color.y();
		fImage[4 * i + 2] += Color.z();

		fprintf(stderr, "\r レイトレェ。。。 %5.2f%%", 100.0*i / ((float)m_ImageWidth * m_ImageHeight));
	}

	AddLightImage(fImage, LImages, Num_Threads);
	delete[] LImages;

	unsigned char* cImage = new unsigned char[m_ImageWidth * m_ImageHeight * 4];
	const float GammaValue = 2.0f;
	Gamma(cImage, fImage, GammaValue, NumSample * NumSuperSample);

	if (Filter) {
		unsigned char* ResultImage = new unsigned char[m_ImageWidth * m_ImageHeight * 4];
		Filter->Filter(cImage, ResultImage, 4, m_ImageWidth, m_ImageHeight);
		stbi_write_bmp(ImageFileName.c_str(), m_ImageWidth, m_ImageHeight, 4, ResultImage);
		delete[] ResultImage;
	}
	else {
		stbi_write_bmp(ImageFileName.c_str(), m_ImageWidth, m_ImageHeight, 4, cImage);
	}

	delete[] fImage;
	delete[] cImage;


	printf("\n　ばいでぃれくしょなる　しゅーりょ \n");
	printf("\n");
	time_t ed = time(NULL);
	int rtm = int(ed) - int(st);
	int th, tm, ts;
	th = rtm / 3600;
	tm = (rtm - th * 3600) / 60;
	ts = rtm - 3600 * th - 60 * tm;
	printf("経過　%d時間 %d分　%d秒\n", th, tm, ts);

}


void bdPathTracing::RenderImage_seq(
	int SwitchCount, const std::string& FolderName,
	const iCamera& Camera, const Scene& scene, int RenderTime_sec, int TimeStep_sec, const PostFilter<unsigned char>* Filter) const{

	const float GammaValue = 2.0f;

	std::chrono::system_clock::time_point StartTime = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point RenderTime = std::chrono::system_clock::now();
	const int NUM_SUPER_SAMPLES = 10;

	time_t st = time(NULL);
	float* temp_Image = new float[m_ImageWidth * m_ImageHeight * 4];
	for (int i = 0; i < 4 * m_ImageWidth*m_ImageHeight; i++) {
		temp_Image[i] = 0.0f;
	}

	int Num_Threads = omp_get_max_threads();

	float* LImages = new float[m_ImageWidth * m_ImageHeight * 4 * Num_Threads];
	for (int i = 0; i < 4 * Num_Threads * m_ImageWidth*m_ImageHeight; i++) {
		LImages[i] = 0.0f;
	}

	Vec3 Camera_E1 = Camera.Get_E1().normalized();
	Vec3 Camera_E2 = Camera.Get_E2().normalized();

	SuperSampling::HaltonSampling SSample(Camera_E1, Camera_E2, time(NULL), NUM_SUPER_SAMPLES + 5);

	bool LoopFlag = true;
	int RenderCount = 0;
	int SampleCount = 0;
	float RenderAverageTime = 0.0f;

	printf("Render Start \n");
	while (LoopFlag) {
		for (int id = 0; id < SwitchCount && LoopFlag; ++id) {

			#ifdef _OPENMP
			#pragma omp parallel for schedule(dynamic, 1)
			#endif
			for (int i = 0; i < (m_ImageWidth) * (m_ImageHeight) / SwitchCount; i++) {
				
				int loopID = i * SwitchCount + SwitchCount * rand() / ((double)(RAND_MAX) + 1.01f);
				int ThreadID = omp_get_thread_num();

				RandomMT MT(SampleCount * 35247 + loopID + 1);

				int w = loopID % m_ImageWidth;
				int h = m_ImageHeight - 1 - loopID / m_ImageWidth;

				float uu = w / (float)m_ImageWidth;
				float vv = h / (float)m_ImageHeight;

				SuperSampling::iSuperSampling::Input SIN;
				SIN._i[0] = SampleCount % NUM_SUPER_SAMPLES;
				Vec3 RV = SSample.GetPosition_Square(SIN);
				RV *= Vec3(Camera.GetPlaneWidth() / (float)m_ImageWidth, Camera.GetPlaneHeight() / (float)m_ImageHeight, 0);
				Ray r(Camera.GetCameraPos(), Camera.GeneratePos(uu, vv) + RV - Camera.GetCameraPos());
				Vec3 Color = TraceRay(r, scene, Camera, LImages + m_ImageWidth * m_ImageHeight * 4 * ThreadID, MT);

				temp_Image[4 * loopID + 0] += Color.x();
				temp_Image[4 * loopID + 1] += Color.y();
				temp_Image[4 * loopID + 2] += Color.z();
			}
			++SampleCount;

			std::chrono::system_clock::time_point NowTime = std::chrono::system_clock::now();
			auto PassedTime = std::chrono::duration_cast<std::chrono::milliseconds>(NowTime - StartTime).count();
			auto RenderPassTime = std::chrono::duration_cast<std::chrono::milliseconds>(NowTime - RenderTime).count();

			RenderAverageTime = (PassedTime) / (float)SampleCount;


			if (RenderPassTime > TimeStep_sec * 1000.0f) {

				printf("\n　write Image : %d \n", PassedTime);

				std::stringstream ss;
				ss << FolderName << "\\" << "Image___Number" << RenderCount++ << "_Samples_" << SampleCount << ".bmp";
				WriteImage(temp_Image, LImages, ss.str().c_str(), (float)SampleCount / (float)SwitchCount, Num_Threads, GammaValue, nullptr);
				RenderTime = std::chrono::system_clock::now();
			}

			if (PassedTime + RenderAverageTime < RenderTime_sec * 1000.0f) {
			}
			else {
				LoopFlag = false;
			}
		}
	}
	WriteImage(temp_Image, LImages, "FinalImage.bmp", (float)SampleCount / (float)SwitchCount, Num_Threads, GammaValue, Filter);
	
	printf("\n　しゅーりょ \n");

	delete[] LImages;
	delete[] temp_Image;
}

void bdPathTracing::WriteImage(
	const float* Image, const float* LightImages, 
	const std::string& FileName, float Samplecount, int NumLightImage, float GammaValue, const PostFilter<unsigned char>* Filter)const {
	
	float* fImage = new float[m_ImageWidth * m_ImageHeight * 4];

	#ifdef _OPENMP
	#pragma omp parallel for schedule(dynamic, 1)
	#endif
	for (int i = 0; i < (m_ImageWidth) * (m_ImageHeight) * 4; i++) {
		fImage[i] = Image[i];
	}

	AddLightImage(fImage, LightImages, NumLightImage);
	
	unsigned char* cImage = new unsigned char[m_ImageWidth * m_ImageHeight * 4];
	if (Filter) {
		unsigned char* ccImage = new unsigned char[m_ImageWidth * m_ImageHeight * 4];
		Gamma(ccImage, fImage, GammaValue, Samplecount);
		Filter->Filter(ccImage, cImage, 4, m_ImageWidth, m_ImageHeight);
		delete[] ccImage;
	}
	else {
		Gamma(cImage, fImage, GammaValue, Samplecount);
	}
	stbi_write_bmp(FileName.c_str(), m_ImageWidth, m_ImageHeight, 4, cImage);

	delete[] cImage;
	delete[] fImage;
}