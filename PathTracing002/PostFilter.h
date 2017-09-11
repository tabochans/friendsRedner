#pragma once
#include<cmath>
#include<algorithm>

namespace PTUtility {

	template<typename T>
	class PostFilter {
	protected:
		int _KernelW;
		int _KernelH;
		float _param1;
		float _param2;
		float _NormalizeFactor;

	public:
		virtual void Filter(const T* srcImage, T* dstImage, int channel, int ImageWidth, int ImageHeight) const = 0;

		PostFilter(int KernelW, int KernelH, float param1, float param2, float NormalizeFactor) : _KernelW(KernelW), _KernelH(KernelH), _param1(param1), _param2(param2), _NormalizeFactor(NormalizeFactor){
		}
		virtual ~PostFilter() {

		}
	};

	template<typename T>
	class GaussFilter : public PostFilter<T>{
	private:

	public:
		virtual void Filter(const T* srcImage, T* dstImage, int channel, int ImageWidth, int ImageHeight) const;
		GaussFilter(int KernelW, int KernelH, float param1, float param2, float NormalizeFactor) : PostFilter(KernelW, KernelH, param1, param2, NormalizeFactor){}
	};

	template<typename T>
	class NolocalMeanFilter : public PostFilter<T> {
	private:

		float Calculate_Weight(const T* Image, int ImageWidth, int ImageHeight, int channel, int pID, int qID, int Width, int Height) const;

	public:
		virtual void Filter(const T* srcImage, T* dstImage, int channel, int ImageWidth, int ImageHeight) const;
		NolocalMeanFilter(int KernelW, int KernelH, float param1, float param2, float NormalizeFactor) : PostFilter(KernelW, KernelH, param1, param2, NormalizeFactor) {}
	};
}


namespace PTUtility {

#include"PostFilter.h"

	using namespace PTUtility;

	template<typename T>
	void GaussFilter<T>::Filter(const T* srcImage, T* dstImage, int channel, int ImageWidth, int ImageHeight) const {

		const int WX = _KernelW * 0.5f;
		const int WY = _KernelH * 0.5f;

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
		for (int i = 0; i < (ImageWidth) * (ImageHeight); ++i) {

			int XX = i % ImageWidth;
			int YY = ImageHeight - 1 - i / ImageWidth;

			if (XX < WX || XX > ImageWidth - WX - 1 || YY < WY || YY > ImageHeight - WY - 1) {
				for (int c = 0; c < channel; ++c) {
					dstImage[i * channel + c] = srcImage[i * channel + c];
				}
				continue;
			}

			for (int c = 0; c < channel; ++c) {

				float result = 0.0f;
				float totalWeight = 0.0f;
				for (int dx = -WX; dx < WX + 1; ++dx) {
					for (int dy = -WY; dy < WY + 1; ++dy) {
						int x = XX + dx;
						int y = YY + dy;

						const T p_Value = srcImage[(x + (ImageHeight - y - 1) * ImageWidth) * channel + c];
						float dist = std::sqrt(dx * dx + dy * dy);
						float weight = std::exp(-dist * _param1);

						result += p_Value * weight;
						totalWeight += weight;
					}
				}
				dstImage[i * channel + c] = result / totalWeight;
			}
		}
	}

	template<typename T>
	float NolocalMeanFilter<T>::Calculate_Weight(const T* Image, int ImageWidth, int ImageHeight, int channel, int pID, int qID, int Width, int Height) const {

		const int WX = Width * 0.5f;
		const int WY = Height * 0.5f;

		float result = 0.00001f;
		for (int dy = -WY; dy < WY + 1; ++dy) {
			for (int dx = -WX; dx < WX + 1; ++dx) {
				for (int c = 0; c < channel; ++c) {

					int pvID = pID + (dx + dy * ImageWidth) * channel + channel;
					int qvID = qID + (dx + dy * ImageWidth) * channel + channel;
					if (qvID < 0 || qvID > ImageWidth * ImageHeight * channel - 1) {
						continue;
					}
					float dist = ((int)Image[pvID]) - ((int)Image[qvID]);
					result += dist * dist;
				}
			}
		}
		result /= (float)(Width * Height);
		return exp(-std::max(std::sqrt(result) - 2.0f * _param1 * _param1 * _NormalizeFactor, 0.0f) / (_param2*_param2*_NormalizeFactor));
	}

	template<typename T>
	void NolocalMeanFilter<T>::Filter(const T* srcImage, T* dstImage, int channel, int ImageWidth, int ImageHeight) const {

		const int WX = _KernelW * 0.5f;
		const int WY = _KernelH * 0.5f;


#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
		for (int i = 0; i < (ImageWidth) * (ImageHeight); ++i) {

			int XX = i % ImageWidth;
			int YY = ImageHeight - 1 - i / ImageWidth;

			if (XX < WX || XX > ImageWidth - WX - 1 || YY < WY || YY > ImageHeight - WY - 1) {
				dstImage[i * channel + 0] = srcImage[i * channel + 0];
				dstImage[i * channel + 1] = srcImage[i * channel + 1];
				dstImage[i * channel + 2] = srcImage[i * channel + 2];
				dstImage[i * channel + 3] = srcImage[i * channel + 3];
				continue;
			}
			float result[3] = { 0,0,0 };
			float totalWeight = 0.0f;

			int pID = i * channel;
			for (int dy = -WY; dy < WY + 1; ++dy) {
				for (int dx = -WX; dx < WX + 1; ++dx) {
					if (dx == 0 && dy == 0) {
						result[0] += srcImage[pID + 0];
						result[1] += srcImage[pID + 1];
						result[2] += srcImage[pID + 2];
						totalWeight += 1.0f;
						continue;
					}

					int x = XX + dx;
					int y = YY + dy;
					int qID = (x + (ImageHeight - y - 1) * ImageWidth) * channel;
					float weight = Calculate_Weight(srcImage, ImageWidth, ImageHeight, channel, pID, qID, _KernelW, _KernelH);
					totalWeight += weight;

					result[0] += (float)srcImage[qID + 0] * weight;
					result[1] += (float)srcImage[qID + 1] * weight;
					result[2] += (float)srcImage[qID + 2] * weight;
				}
			}

			dstImage[pID + 0] = result[0] / totalWeight;
			dstImage[pID + 1] = result[1] / totalWeight;
			dstImage[pID + 2] = result[2] / totalWeight;
			dstImage[pID + 3] = srcImage[pID + 3];
		}
	}


}