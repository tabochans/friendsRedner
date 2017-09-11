#pragma once
#include"MT.h"
#include"Vec3.h"

namespace SuperSampling{

	class iSuperSampling{
	protected:
		const PTUtility::Vec3 m_v1;
		const PTUtility::Vec3 m_v2;
		RandomMT* m_MT;
		const int NUMSAMPLE;
	public:

		struct Input{
			int _i[3];
			PTUtility::Vec3 _f;
		};

		//v1 and v2 define triangle
		iSuperSampling(const PTUtility::Vec3& v1, const PTUtility::Vec3& v2, int Seed, int NumSample) : m_v1(v1), m_v2(v2), NUMSAMPLE(NUMSAMPLE){
			m_MT = new RandomMT(Seed);
		}
		virtual ~iSuperSampling(){
			delete m_MT;
		}

		// return position in triangle defined using v1 and v2
		// z value is always zero
		virtual PTUtility::Vec3 GetPosition_Triangle(const Input& input) = 0;

		// return position in square defined using v1 and v2
		// z value is always zero
		virtual PTUtility::Vec3 GetPosition_Square(const Input& input) = 0;
	};

	class RandomSampling : public iSuperSampling{
	private:
	public:
		RandomSampling(const PTUtility::Vec3& v1, const PTUtility::Vec3& v2, int Seed, int NumSample) : iSuperSampling(v1, v2, Seed, NumSample){
		}
		virtual ~RandomSampling(){

		}
		virtual PTUtility::Vec3 GetPosition_Triangle(const Input& input);
		virtual PTUtility::Vec3 GetPosition_Square(const Input& input);
	};

	class HaltonSampling : public iSuperSampling{
	private:
		PTUtility::Vec3* m_SampleArray;

	public:
		HaltonSampling(const PTUtility::Vec3& v1, const PTUtility::Vec3& v2, int Seed, int NumSample) : iSuperSampling(v1, v2, Seed, NumSample){
			m_SampleArray = new PTUtility::Vec3[NumSample];
			int base[2] = { 2, 3 };
			
			for (int b = 0; b < 2; ++b){
				int c = 1;
				int cc = base[b];
				for (int i = 0; i < NumSample; ++i){
					m_SampleArray[i][b] = float(c) / float(cc);
					c += base[b];
					if (c >= cc){
						c = 1;
						cc*= base[b];
					}
				}
			}
		}
		virtual ~HaltonSampling(){
			delete[] m_SampleArray;
		}

		//input._i[0] must be filled n < NumSample
		virtual PTUtility::Vec3 GetPosition_Triangle(const Input& input);

		//input._i[0] must be filled n < NumSample
		virtual PTUtility::Vec3 GetPosition_Square(const Input& input);
	};

	class PoissonDiscSampling : public iSuperSampling{
	private:
	public:
		PoissonDiscSampling(const PTUtility::Vec3& v1, const PTUtility::Vec3& v2, int Seed, int NumSample) : iSuperSampling(v1, v2, Seed, NumSample){
		}
		virtual ~PoissonDiscSampling(){

		}

		virtual PTUtility::Vec3 GetPosition_Triangle(const Input& input);
		virtual PTUtility::Vec3 GetPosition_Square(const Input& input);
	};

}