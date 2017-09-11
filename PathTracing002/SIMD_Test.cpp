#pragma once
#include"SIMD_Test.h"

void simd_test::test_simd(){
	for (int i = 0; i < Round; ++i){
		_SResult[i] = _mm256_sub_ps(_SA[i], _SB[i]);
	}
	for (int i = 0; i < Round; ++i){
		_SResult[i] = _mm256_add_ps(_SA[i], _SB[i]);
	}
	for (int i = 0; i < Round; ++i){
		_SResult[i] = _mm256_mul_ps(_SA[i], _SB[i]);
	}
}

void simd_test::test(){
	for (int i = 0; i < Round; ++i){
		for (int j = 0; j < 8; ++j){
			_Result[i].f[j] = _A[i].f[j] - _B[i].f[j];
		}
	}
	for (int i = 0; i < Round; ++i){
		for (int j = 0; j < 8; ++j){
			_Result[i].f[j] = _A[i].f[j] + _B[i].f[j];
		}
	}
	for (int i = 0; i < Round; ++i){
		for (int j = 0; j < 8; ++j){
			_Result[i].f[j] = _A[i].f[j] * _B[i].f[j];
		}
	}
}

void simd_test::TestSub(){
	{
		ScopedTimer _prof("SIMD");
		test_simd();
	}
	{
		ScopedTimer _prof("Not SIMD");
		test();
	}
	ScopedTimer::PrintTimeTable(std::cout);
}