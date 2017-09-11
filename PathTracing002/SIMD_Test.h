#pragma once
#include <immintrin.h>
#include<random>
#include<iostream>
#include"ScopedTimer.h"

struct simd_test{
	const int Round;
	struct float8{
		float f[8];
	};

	float8* _A;
	float8* _B;
	float8* _Result;

	__m256* _SA;
	__m256* _SB;
	__m256* _SResult;

	float Gene(){
		return rand() / (float)(RAND_MAX);
	}

	void test_simd();
	void test();
	void TestSub();

	void Init(){
		_A = new float8[Round];
		_B = new float8[Round];
		_Result = new float8[Round];

		_SA = new __m256[Round];
		_SB = new __m256[Round];
		_SResult = new __m256[Round];

		for (int i = 0; i < Round; ++i){
			float8 AA;
			float8 BB;
			for (int j = 0; j < 8; ++j){
				AA.f[j] = Gene();
				BB.f[j] = Gene();
			}
			_A[i] = AA;
			_B[i] = BB;

			_SA[i] = _mm256_loadu_ps(AA.f);
			_SB[i] = _mm256_loadu_ps(BB.f);
		}
	}

	explicit simd_test(int r) : Round(r){
		Init();
	}
	~simd_test(){
		delete[] _A;
		delete[] _B;
		delete[] _Result;

		delete[] _SA;
		delete[] _SB;
		delete[] _SResult;

	}

};
