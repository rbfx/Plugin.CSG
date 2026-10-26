// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

// CsgFloat4 holds four floats and offers the operations that the exact filter needs.
// CsgFloat4 is internal. Its layout and alignment depend on the compile flags.
// Do not use it in an exported declaration. Do not pass it across a module boundary.
// All backends give the same result for each operation.
// The backend is selected by macros. The scalar backend is the fallback.

#if defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
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

// The filter needs strict rounding of each operation. Do not use FMA or any other fused operation here.

/// Four floats in one 128-bit register. Lane 0 is the first lane.
class CsgFloat4
{
public:
    CsgFloat4() = default;

    /// Create a vector with the same value in all lanes.
    static CsgFloat4 Splat(float value) { return Create(value, value, value, value); }

    /// Create a vector from four lanes.
    static CsgFloat4 Create(float x0, float x1, float x2, float x3)
    {
        CsgFloat4 result;
#if CSG_SIMD_SSE2
        result.v_ = _mm_setr_ps(x0, x1, x2, x3);
#elif CSG_SIMD_NEON
        const float lanes[4] = {x0, x1, x2, x3};
        result.v_ = vld1q_f32(lanes);
#elif CSG_SIMD_WASM
        result.v_ = wasm_f32x4_make(x0, x1, x2, x3);
#else
        result.v_[0] = x0;
        result.v_[1] = x1;
        result.v_[2] = x2;
        result.v_[3] = x3;
#endif
        return result;
    }

    /// Write the four lanes to the output array.
    void Store(float* out) const
    {
#if CSG_SIMD_SSE2
        _mm_storeu_ps(out, v_);
#elif CSG_SIMD_NEON
        vst1q_f32(out, v_);
#elif CSG_SIMD_WASM
        wasm_v128_store(out, v_);
#else
        for (int i = 0; i < 4; ++i)
            out[i] = v_[i];
#endif
    }

    /// Return one lane. The index must be in the range from 0 to 3.
    float Get(int index) const
    {
        float lanes[4];
        Store(lanes);
        return lanes[index];
    }

    friend CsgFloat4 operator+(const CsgFloat4& a, const CsgFloat4& b)
    {
        CsgFloat4 r;
#if CSG_SIMD_SSE2
        r.v_ = _mm_add_ps(a.v_, b.v_);
#elif CSG_SIMD_NEON
        r.v_ = vaddq_f32(a.v_, b.v_);
#elif CSG_SIMD_WASM
        r.v_ = wasm_f32x4_add(a.v_, b.v_);
#else
        for (int i = 0; i < 4; ++i)
            r.v_[i] = a.v_[i] + b.v_[i];
#endif
        return r;
    }

    friend CsgFloat4 operator-(const CsgFloat4& a, const CsgFloat4& b)
    {
        CsgFloat4 r;
#if CSG_SIMD_SSE2
        r.v_ = _mm_sub_ps(a.v_, b.v_);
#elif CSG_SIMD_NEON
        r.v_ = vsubq_f32(a.v_, b.v_);
#elif CSG_SIMD_WASM
        r.v_ = wasm_f32x4_sub(a.v_, b.v_);
#else
        for (int i = 0; i < 4; ++i)
            r.v_[i] = a.v_[i] - b.v_[i];
#endif
        return r;
    }

    friend CsgFloat4 operator*(const CsgFloat4& a, const CsgFloat4& b)
    {
        CsgFloat4 r;
#if CSG_SIMD_SSE2
        r.v_ = _mm_mul_ps(a.v_, b.v_);
#elif CSG_SIMD_NEON
        r.v_ = vmulq_f32(a.v_, b.v_);
#elif CSG_SIMD_WASM
        r.v_ = wasm_f32x4_mul(a.v_, b.v_);
#else
        for (int i = 0; i < 4; ++i)
            r.v_[i] = a.v_[i] * b.v_[i];
#endif
        return r;
    }

    /// Return the absolute value of each lane.
    static CsgFloat4 Abs(const CsgFloat4& a)
    {
        CsgFloat4 r;
#if CSG_SIMD_SSE2
        r.v_ = _mm_andnot_ps(_mm_set1_ps(-0.0f), a.v_);
#elif CSG_SIMD_NEON
        r.v_ = vabsq_f32(a.v_);
#elif CSG_SIMD_WASM
        r.v_ = wasm_f32x4_abs(a.v_);
#else
        for (int i = 0; i < 4; ++i)
            r.v_[i] = std::fabs(a.v_[i]);
#endif
        return r;
    }

    /// Compare a > b in each lane. Bit i of the result is set when lane i is greater. A NaN lane gives a clear bit.
    static int GreaterThanMask(const CsgFloat4& a, const CsgFloat4& b)
    {
#if CSG_SIMD_SSE2
        return _mm_movemask_ps(_mm_cmpgt_ps(a.v_, b.v_));
#elif CSG_SIMD_NEON
        const uint32x4_t greater = vcgtq_f32(a.v_, b.v_);
        const uint32_t weights[4] = {1, 2, 4, 8};
        return static_cast<int>(vaddvq_u32(vmulq_u32(vshrq_n_u32(greater, 31), vld1q_u32(weights))));
#elif CSG_SIMD_WASM
        return static_cast<int>(wasm_i32x4_bitmask(wasm_f32x4_gt(a.v_, b.v_)));
#else
        int mask = 0;
        for (int i = 0; i < 4; ++i)
            mask |= (a.v_[i] > b.v_[i] ? 1 : 0) << i;
        return mask;
#endif
    }

private:
#if CSG_SIMD_SSE2
    __m128 v_;
#elif CSG_SIMD_NEON
    float32x4_t v_;
#elif CSG_SIMD_WASM
    v128_t v_;
#else
    float v_[4];
#endif
};

} // namespace Urho3D
