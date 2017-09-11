#include"SuperSampling.h"

namespace SuperSampling{
	PTUtility::Vec3 RandomSampling::GetPosition_Triangle(const Input& input){
		float u = m_MT->genrand64_real1();
		float v = m_MT->genrand64_real1();
		if (u + v > 1.000){
			u = 1.0 - u;
			v = 1.0 - v;
		}
		return m_v1 * u + m_v2 * v;
	}
	PTUtility::Vec3 RandomSampling::GetPosition_Square(const Input& input){
		float u = m_MT->genrand64_real1();
		float v = m_MT->genrand64_real1();
		return m_v1 * u + m_v2 * v;
	}

	PTUtility::Vec3 HaltonSampling::GetPosition_Triangle(const Input& input){
		float u = m_SampleArray[input._i[0]][0];
		float v = m_SampleArray[input._i[0]][1];
		if (u + v > 1.000){
			u = 1.0 - u;
			v = 1.0 - v;
		}
		return m_v1 * u + m_v2 * v;
	}
	PTUtility::Vec3 HaltonSampling::GetPosition_Square(const Input& input){
		float u = m_SampleArray[input._i[0]][0];
		float v = m_SampleArray[input._i[0]][1];
		return m_v1 * u + m_v2 * v;
	}

	PTUtility::Vec3 PoissonDiscSampling::GetPosition_Triangle(const Input& input){
		return PTUtility::Vec3::Zero();
	}
	PTUtility::Vec3 PoissonDiscSampling::GetPosition_Square(const Input& input){
		return PTUtility::Vec3::Zero();
	}

}