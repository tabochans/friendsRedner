#include"gHeader.h"
#include"Vec2.h"
#include"Vec3.h"
#include"PathTracing.h"
#include"bdPathTracing.h"
#include"Sampling.h"
#include"OBVH.h"
#include"Intersect.h"
#include"tiny_obj_loader.h"
#include"ScopedTimer.h"
#include"LightManager.h"
#include"MT.h"
std::vector<ScopedTimer::ID_TIME> ScopedTimer::m_Table;
#include"Scene.h"
#include"SIMD_Test.h"
#include"SceneMaterial.h"
#include"PostFilter.h"
#include<vector>
#include<random>
#include<array>
#include<iostream>
#include<fstream>

using namespace PTUtility;
using namespace std;

const int RENDER_TIME = 4 * 60 + 33;

class CornelBoxMaterial : public SceneMaterialManager {
	virtual void SetParamtoAttribute(Attribute* at, const std::string& MaterialName) const {
		if (MaterialName == std::string("occlude")) {
			at->m_BRDF = GGXSurface::GGX;
			at->m_BRDF_PDF = GGXSurface::PDF;
			at->m_BRDF_Sample = GGXSurface::GGX_Sampling;
			at->m_Param1 = 0.2f;
			at->m_Index = 0.277f;
			at->m_Param2 = 2.928f;
		}
		else if (MaterialName == std::string("leftWall")) {
			at->m_BRDF = GGXSurface::GGX;
			at->m_BRDF_PDF = GGXSurface::PDF;
			at->m_BRDF_Sample = GGXSurface::GGX_Sampling;
			at->m_Param1 = 0.35f;
			at->m_Index = 0.277f;
			at->m_Param2 = 2.928f;
		}
		else if (MaterialName == std::string("backWall")) {
			at->m_BRDF = GGXSurface::GGX;
			at->m_BRDF_PDF = GGXSurface::PDF;
			at->m_BRDF_Sample = GGXSurface::GGX_Sampling;
			at->m_Param1 = 0.035f;
			at->m_Index = 0.277f;
			at->m_Param2 = 2.928f;
		}
		else if (MaterialName == std::string("sphere")) {
			at->m_BRDF = GlassSurface::Glass;
			at->m_BRDF_PDF = GlassSurface::PDF;
			at->m_BRDF_Sample = GlassSurface::Glass_Sampling;
			at->m_Index = 1.6;
			at->m_DeltaSurface = true;
		}
		else {
			at->m_BRDF = DiffuseSurface::Diffuse;
			at->m_BRDF_PDF = DiffuseSurface::PDF;
			at->m_BRDF_Sample = DiffuseSurface::Diffuse_Sampling;
		}
	}
};


class Mat1 : public SceneMaterialManager{
	virtual void SetParamtoAttribute(Attribute* at, const std::string& MaterialName) const{
		if (MaterialName == std::string("sp_00_pod")) {
			at->m_BRDF = GGXSurface::GGX;
			at->m_BRDF_PDF = GGXSurface::PDF;
			at->m_BRDF_Sample = GGXSurface::GGX_Sampling;
			at->m_Param1 = 0.2f;
			at->m_Index = 0.277f;
			at->m_Param2 = 2.928f;
		}
		else if (MaterialName == std::string("dyz")) {
			at->m_BRDF = GGXSurface::GGX;
			at->m_BRDF_PDF = GGXSurface::PDF;
			at->m_BRDF_Sample = GGXSurface::GGX_Sampling;
			at->m_Param1 = 0.03f;
			at->m_Param2 = 2.928f;
			at->m_Index = 0.277f;
		}
		else if (
			MaterialName == std::string("BlockInter") || 
			MaterialName == std::string("BlockSurface") || 
			MaterialName == std::string("tile") ||
			MaterialName == std::string("blade_mt") || 
			MaterialName == std::string("blade_dif")) {
			at->m_BRDF = MetalSurface::Metal;
			at->m_BRDF_PDF = MetalSurface::PDF;
			at->m_BRDF_Sample = MetalSurface::Metal_Sampling;
			at->m_DeltaSurface = true;
		}
		else if (MaterialName == std::string("SBunnySurface")) {
			at->m_BRDF = GlassSurface::Glass;
			at->m_BRDF_PDF = GlassSurface::PDF;
			at->m_BRDF_Sample = GlassSurface::Glass_Sampling;
			at->m_Index = 1.5;
			at->m_DeltaSurface = true;
		}
		else if (MaterialName == std::string("BunnySurface")) {
			at->m_BRDF = GlassSurface::Glass;
			at->m_BRDF_PDF = GlassSurface::PDF;
			at->m_BRDF_Sample = GlassSurface::Glass_Sampling;
			at->m_Index = 1.15;
			at->m_DeltaSurface = true;
		}
		else if (MaterialName == std::string("sp_00_luk_mali") || MaterialName == std::string("sp_00_stup")) {
			at->m_BRDF = GGXSurface::GGX;
			at->m_BRDF_PDF = GGXSurface::PDF;
			at->m_BRDF_Sample = GGXSurface::GGX_Sampling;
			at->m_Param1 = 0.05f;
			at->m_Albedo = Vec3(1, 1, 1);
			at->m_Index = 0.277f;
			at->m_Param2 = 2.928f;
		}
		else if (MaterialName == std::string("sp_svod_kapitel")) {
			at->m_BRDF = MetalSurface::Metal;
			at->m_BRDF_PDF = MetalSurface::PDF;
			at->m_BRDF_Sample = MetalSurface::Metal_Sampling;
			at->m_DeltaSurface = true;
		}
		else if (MaterialName == std::string("Pot") || MaterialName == std::string("Pot2")) {
			at->m_BRDF = GGXSurface::GGX;
			at->m_BRDF_PDF = GGXSurface::PDF;
			at->m_BRDF_Sample = GGXSurface::GGX_Sampling;
			at->m_Param1 = 0.05f;
			at->m_Index = 0.0516f;
			at->m_Param2 = 3.905f;
		}
		else{
			at->m_BRDF = DiffuseSurface::Diffuse;
			at->m_BRDF_PDF = DiffuseSurface::PDF;
			at->m_BRDF_Sample = DiffuseSurface::Diffuse_Sampling;
		}
	}
};

class Mat2 : public SceneMaterialManager {
	virtual void SetParamtoAttribute(Attribute* at, const std::string& MaterialName) const {
		at->m_BRDF = DiffuseSurface::Diffuse;
		at->m_BRDF_PDF = DiffuseSurface::PDF;
		at->m_BRDF_Sample = DiffuseSurface::Diffuse_Sampling;
	}
};

class Mat3 : public SceneMaterialManager {
	virtual void SetParamtoAttribute(Attribute* at, const std::string& MaterialName) const {

		at->m_BRDF = GGXSurface::GGX;
		at->m_BRDF_PDF = GGXSurface::PDF;
		at->m_BRDF_Sample = GGXSurface::GGX_Sampling;
		at->m_Param1 = 0.1f;
		at->m_Index = 0.0516f;
		at->m_Param2 = 3.905f;
	}
};

void RenderBox(){
	//BoxScene

	CornelBoxMaterial mat;

	Scene scene;
	scene.CreateScene("OBJ\\CornelBox.obj", &mat);
	PHCamera camera(Vec3(0.0, 2.0, 3.7), Vec3(0, 1.0, 0), 45.0, 1);
	RandomMT MT1(1234);

	std::cout << "Test Start" << std::endl;
	std::cout << "######################" << std::endl;
	PathTracing2 pt(512, 512);
	pt.RenderImage("CornelPT.bmp", camera, scene, 15, 10);
	std::cout << "######################" << std::endl;
}

void RenderBoxBD() {
	//BoxScene

	CornelBoxMaterial mat;
	//Mat3 mat;


	Scene scene;
	scene.CreateScene("OBJ\\CornelBox.obj", &mat);
	PHCamera camera(Vec3(0.0, 2.0, 3.7), Vec3(0, 1.0, 0), 45.0, 1);
	RandomMT MT1(1234);

	std::cout << "Test Start" << std::endl;
	std::cout << "######################" << std::endl;
	bdPathTracing pt(512 / 1, 512 / 1);
	pt.RenderImage("CornelBDPT.bmp", camera, scene, 6, 5, nullptr);
	std::cout << "######################" << std::endl;
}

void RenderBD(){
	//BoxScene

	Mat1 mat1;
	Mat2 mat2;
	Mat3 mat3;

	Scene scene;
	scene.CreateScene("OBJ\\scene00obj.obj", nullptr);
	PHCamera camera(Vec3(4.0, 3.0, 0.7), Vec3(1, 2.0, 0), 95.0, 1920.0 / 1024.0);
	RandomMT MT1(1234);
	NolocalMeanFilter<unsigned char> Filter(15, 15, 0.35f, 0.35f, 255);

	std::cout << "Test Start" << std::endl;
	std::cout << "######################" << std::endl;
	bdPathTracing pt(1920 / 16, 1024 / 16);
	pt.RenderImage("bdpt_result.bmp", camera, scene, 1, 1, nullptr);
	std::cout << "######################" << std::endl;

	ScopedTimer::PrintTimeTable(std::cout);
}

void RenderPT() {
	//BoxScene

	Mat1 mat1;

	Scene scene;
	scene.CreateScene("OBJ\\scene00obj.obj", &mat1);
	PHCamera camera(Vec3(4.0, 3.0, 0.7), Vec3(1, 2.0, 0), 95.0, 1920.0 / 1024.0);
	RandomMT MT1(1234);

	std::cout << "Test Start" << std::endl;
	std::cout << "######################" << std::endl;
	PathTracing2 pt(1920 / 1, 1024 / 1);
	pt.RenderImage("bpt_result.bmp", camera, scene, 2, 2);
	std::cout << "######################" << std::endl;
}

void Render_Seq() {
	//BoxScene

	{
		ScopedTimer _prof("bdpt");

		std::chrono::system_clock::time_point STime = std::chrono::system_clock::now();

		Mat1 mat1;
		Scene scene;
		NolocalMeanFilter<unsigned char> Filter(7, 7, 0.33f, 0.22f, 255);
		scene.CreateScene("OBJ\\scene00obj.obj", &mat1);
		PHCamera camera(Vec3(4.0, 3.0, 0.7), Vec3(1, 2.0, 0), 95.0, 1980.0 / 1024.0);
		RandomMT MT1(1234);
		bdPathTracing pt(1980 / 1, 1024 / 1);

		std::chrono::system_clock::time_point SSTime = std::chrono::system_clock::now();
		
		pt.RenderImage_seq(16, "Result", camera, scene, RENDER_TIME - std::chrono::duration_cast<std::chrono::seconds>(SSTime - STime).count() - 7, 40, &Filter);
	}
	ScopedTimer::PrintTimeTable(std::cout);
}


int main(){

	//Render_Seq();
	//RenderBD();
	//RenderPT();

	RenderBoxBD();
	//RenderBox();

	return 0;
}