// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

// CsgDouble4 holds four doubles and offers the operations that the exact filter needs.
// CsgDouble4 is internal. Its layout and alignment depend on the compile flags.
// Do not use it in an exported declaration. Do not pass it across a module boundary.
// All backends give the same result for each operation.
// The backend is selected by macros. The scalar backend is the fallback.

#if defined(__AVX__)
#define CSG_SIMD_AVX 1
#include <immintrin.h>
#elif defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#define CSG_SIMD_SSE2 1
#include <emmintrin.h>
#elif defined(__aarch64__) && defined(__ARM_NEON)
#define CSG_SIMD_NEON 1
#include <arm_neon.h>
#elif defined(__wasm_simd128__)
#define CSG_SIMD_WASM 1
#include <wasm_simd128.h>
#else
#define CSG_SIMD_SCALAR 1
#include <cmath>
#endif

namespace Urho3D
{

/// Four doubles. Lane 0 is the first lane.
class CsgDouble4
{
public:
    CsgDouble4() = default;

    /// Create a vector with the same value in all lanes.
    static CsgDouble4 Splat(double value) { return Create(value, value, value, value); }

    /// Create a vector from four lanes.
    static CsgDouble4 Create(double x0, double x1, double x2, double x3)
    {
        CsgDouble4 result;
#if CSG_SIMD_AVX
        result.v_ = _mm256_setr_pd(x0, x1, x2, x3);
#elif CSG_SIMD_SSE2
        result.lo_ = _mm_setr_pd(x0, x1);
        result.hi_ = _mm_setr_pd(x2, x3);
#elif CSG_SIMD_NEON
        const double low[2] = {x0, x1};
        const double high[2] = {x2, x3};
        result.lo_ = vld1q_f64(low);
        result.hi_ = vld1q_f64(high);
#elif CSG_SIMD_WASM
        result.lo_ = wasm_f64x2_make(x0, x1);
        result.hi_ = wasm_f64x2_make(x2, x3);
#else
        result.v_[0] = x0;
        result.v_[1] = x1;
        result.v_[2] = x2;
        result.v_[3] = x3;
#endif
        return result;
    }

    /// Write the four lanes to the output array.
    void Store(double* out) const
    {
#if CSG_SIMD_AVX
        _mm256_storeu_pd(out, v_);
#elif CSG_SIMD_SSE2
        _mm_storeu_pd(out, lo_);
        _mm_storeu_pd(out + 2, hi_);
#elif CSG_SIMD_NEON
        vst1q_f64(out, lo_);
        vst1q_f64(out + 2, hi_);
#elif CSG_SIMD_WASM
        wasm_v128_store(out, lo_);
        wasm_v128_store(out + 2, hi_);
#else
        for (int i = 0; i < 4; ++i)
            out[i] = v_[i];
#endif
    }

    /// Return one lane. The index must be in the range from 0 to 3.
    double Get(int index) const
    {
        double lanes[4];
        Store(lanes);
        return lanes[index];
    }

    friend CsgDouble4 operator+(const CsgDouble4& a, const CsgDouble4& b)
    {
        CsgDouble4 r;
#if CSG_SIMD_AVX
        r.v_ = _mm256_add_pd(a.v_, b.v_);
#elif CSG_SIMD_SSE2
        r.lo_ = _mm_add_pd(a.lo_, b.lo_);
        r.hi_ = _mm_add_pd(a.hi_, b.hi_);
#elif CSG_SIMD_NEON
        r.lo_ = vaddq_f64(a.lo_, b.lo_);
        r.hi_ = vaddq_f64(a.hi_, b.hi_);
#elif CSG_SIMD_WASM
        r.lo_ = wasm_f64x2_add(a.lo_, b.lo_);
        r.hi_ = wasm_f64x2_add(a.hi_, b.hi_);
#else
        for (int i = 0; i < 4; ++i)
            r.v_[i] = a.v_[i] + b.v_[i];
#endif
        return r;
    }

    friend CsgDouble4 operator-(const CsgDouble4& a, const CsgDouble4& b)
    {
        CsgDouble4 r;
#if CSG_SIMD_AVX
        r.v_ = _mm256_sub_pd(a.v_, b.v_);
#elif CSG_SIMD_SSE2
        r.lo_ = _mm_sub_pd(a.lo_, b.lo_);
        r.hi_ = _mm_sub_pd(a.hi_, b.hi_);
#elif CSG_SIMD_NEON
        r.lo_ = vsubq_f64(a.lo_, b.lo_);
        r.hi_ = vsubq_f64(a.hi_, b.hi_);
#elif CSG_SIMD_WASM
        r.lo_ = wasm_f64x2_sub(a.lo_, b.lo_);
        r.hi_ = wasm_f64x2_sub(a.hi_, b.hi_);
#else
        for (int i = 0; i < 4; ++i)
            r.v_[i] = a.v_[i] - b.v_[i];
#endif
        return r;
    }

    friend CsgDouble4 operator*(const CsgDouble4& a, const CsgDouble4& b)
    {
        CsgDouble4 r;
#if CSG_SIMD_AVX
        r.v_ = _mm256_mul_pd(a.v_, b.v_);
#elif CSG_SIMD_SSE2
        r.lo_ = _mm_mul_pd(a.lo_, b.lo_);
        r.hi_ = _mm_mul_pd(a.hi_, b.hi_);
#elif CSG_SIMD_NEON
        r.lo_ = vmulq_f64(a.lo_, b.lo_);
        r.hi_ = vmulq_f64(a.hi_, b.hi_);
#elif CSG_SIMD_WASM
        r.lo_ = wasm_f64x2_mul(a.lo_, b.lo_);
        r.hi_ = wasm_f64x2_mul(a.hi_, b.hi_);
#else
        for (int i = 0; i < 4; ++i)
            r.v_[i] = a.v_[i] * b.v_[i];
#endif
        return r;
    }

    /// Return the absolute value of each lane.
    static CsgDouble4 Abs(const CsgDouble4& a)
    {
        CsgDouble4 r;
#if CSG_SIMD_AVX
        r.v_ = _mm256_andnot_pd(_mm256_set1_pd(-0.0), a.v_);
#elif CSG_SIMD_SSE2
        r.lo_ = _mm_andnot_pd(_mm_set1_pd(-0.0), a.lo_);
        r.hi_ = _mm_andnot_pd(_mm_set1_pd(-0.0), a.hi_);
#elif CSG_SIMD_NEON
        r.lo_ = vabsq_f64(a.lo_);
        r.hi_ = vabsq_f64(a.hi_);
#elif CSG_SIMD_WASM
        r.lo_ = wasm_f64x2_abs(a.lo_);
        r.hi_ = wasm_f64x2_abs(a.hi_);
#else
        for (int i = 0; i < 4; ++i)
            r.v_[i] = std::fabs(a.v_[i]);
#endif
        return r;
    }

    /// Compare a > b in each lane. Bit i of the result is set when lane i is greater. A NaN lane gives a clear bit.
    static int GreaterThanMask(const CsgDouble4& a, const CsgDouble4& b)
    {
#if CSG_SIMD_AVX
        return _mm256_movemask_pd(_mm256_cmp_pd(a.v_, b.v_, _CMP_GT_OQ));
#elif CSG_SIMD_SSE2
        return _mm_movemask_pd(_mm_cmpgt_pd(a.lo_, b.lo_)) | (_mm_movemask_pd(_mm_cmpgt_pd(a.hi_, b.hi_)) << 2);
#elif CSG_SIMD_NEON
        const uint64x2_t lo = vcgtq_f64(a.lo_, b.lo_);
        const uint64x2_t hi = vcgtq_f64(a.hi_, b.hi_);
        return static_cast<int>((vgetq_lane_u64(lo, 0) & 1u) | ((vgetq_lane_u64(lo, 1) & 1u) << 1)
            | ((vgetq_lane_u64(hi, 0) & 1u) << 2) | ((vgetq_lane_u64(hi, 1) & 1u) << 3));
#elif CSG_SIMD_WASM
        return static_cast<int>(wasm_i64x2_bitmask(wasm_f64x2_gt(a.lo_, b.lo_)))
            | (static_cast<int>(wasm_i64x2_bitmask(wasm_f64x2_gt(a.hi_, b.hi_))) << 2);
#else
        int mask = 0;
        for (int i = 0; i < 4; ++i)
            mask |= (a.v_[i] > b.v_[i] ? 1 : 0) << i;
        return mask;
#endif
    }

private:
#if CSG_SIMD_AVX
    __m256d v_;
#elif CSG_SIMD_SSE2
    __m128d lo_;
    __m128d hi_;
#elif CSG_SIMD_NEON
    float64x2_t lo_;
    float64x2_t hi_;
#elif CSG_SIMD_WASM
    v128_t lo_;
    v128_t hi_;
#else
    double v_[4];
#endif
};

} // namespace Urho3D
