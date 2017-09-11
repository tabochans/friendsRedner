#include"PostFilter.h"

using namespace PTUtility;

void GaussFilter::Filter(const float* srcImage, float* dstImage, int channel, int ImageWidth, int ImageHeight) const {

	//const int WX = _KernelW * 0.5f;
	//const int WY = _KernelH * 0.5f;
	const int WX = 1;
	const int WY = 1;

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
	for (int i = 0; i < (ImageWidth) * (ImageHeight); ++i) {

		int XX = i % ImageWidth;
		int YY = ImageHeight - 1 - i / ImageWidth;

		if (XX < WX || XX > ImageWidth - WX - 1 || YY < WY || YY > ImageHeight - WY - 1) {
			continue;
		}

		for (int c = 0; c < channel; ++c) {

			float result = 0.0f;
			float totalWeight = 0.0f;
			for (int dx = -WX; dx < WX + 1; ++dx) {
				for (int dy = -WY; dy < WY + 1; ++dy) {
					int x = XX + dx;
					int y = YY + dy;

					const float p_Value = std::min(1.0f, std::max(0.0f, srcImage[(x + (ImageHeight - y - 1) * ImageWidth) * channel + c]));
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

float NolocalMeanFilter::Calculate_Weight(const float* Image, int ImageWidth, int ImageHeight, int channel, int pID, int qID, int Width, int Height) const{

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

				float dist = std::min(1.0f, std::max(0.0f, Image[pvID])) - std::min(1.0f, std::max(0.0f, Image[qvID]));
				result += dist * dist;
			}
		}
	}
	return exp(-std::max(std::sqrt(result) - 2.0f * _param1 * _param1, 0.0f) / (_param2*_param2));
}


void NolocalMeanFilter::Filter(const float* srcImage, float* dstImage, int channel, int ImageWidth, int ImageHeight) const {

	const int WX = _KernelW * 0.5f;
	const int WY = _KernelH * 0.5f;


	#ifdef _OPENMP
	#pragma omp parallel for schedule(dynamic, 1)
	#endif
	for (int i = 0; i < (ImageWidth) * (ImageHeight); ++i) {

		int XX = i % ImageWidth;
		int YY = ImageHeight - 1 - i / ImageWidth;

		if (XX < WX || XX > ImageWidth - WX - 1 || YY < WY || YY > ImageHeight - WY - 1) {
			continue;
		}
		float result[3] = { 0,0,0 };
		float totalWeight = 0.0f;

		int pID = i * channel;
		for (int dy = -WY; dy < WY + 1; ++dy) {
			for (int dx = -WX; dx < WX + 1; ++dx) {
				if (dx == 0 && dy == 0) {
					result[0] += std::min(1.0f, std::max(0.0f, srcImage[pID + 0]));
					result[1] += std::min(1.0f, std::max(0.0f, srcImage[pID + 1]));
					result[2] += std::min(1.0f, std::max(0.0f, srcImage[pID + 2]));
					totalWeight += 1.0f;
					continue;
				}

				int x = XX + dx;
				int y = YY + dy;
				int qID = (x + (ImageHeight - y - 1) * ImageWidth) * channel;
				float weight = Calculate_Weight(srcImage, ImageWidth, ImageHeight, channel, pID, qID, _KernelW, _KernelH);
				totalWeight += weight;

				result[0] += std::min(1.0f, std::max(0.0f, srcImage[qID + 0])) * weight;
				result[1] += std::min(1.0f, std::max(0.0f, srcImage[qID + 1])) * weight;
				result[2] += std::min(1.0f, std::max(0.0f, srcImage[qID + 2])) * weight;
			}
		}
		dstImage[pID + 0] = result[0] / totalWeight;
		dstImage[pID + 1] = result[1] / totalWeight;
		dstImage[pID + 2] = result[2] / totalWeight;
	}
}