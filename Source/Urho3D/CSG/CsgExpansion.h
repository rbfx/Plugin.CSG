// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#include "Urho3D/CSG/_Plugin.h"

#include <cmath>

// Floating point expansions after Shewchuk, "Adaptive Precision Floating-Point Arithmetic".
// An expansion is a list of floating point terms. The exact value is the sum of the terms.
// The terms do not overlap. The magnitudes increase. The first term has the smallest magnitude.
// An empty expansion is zero.
// The code needs strict IEEE arithmetic: no contraction, no fast math, no extended precision.

#if defined(CSG_EXACT_USE_DOUBLE)
#if defined(__FMA__) || defined(__AVX2__) || defined(__aarch64__) || defined(_M_ARM64) || defined(FP_FAST_FMA)
#define CSG_HAS_HARDWARE_FMA 1
#endif
#else
#if defined(__FMA__) || defined(__AVX2__) || defined(__aarch64__) || defined(_M_ARM64) || defined(FP_FAST_FMAF)
#define CSG_HAS_HARDWARE_FMA 1
#endif
#endif

namespace Urho3D
{

#if defined(CSG_EXACT_USE_DOUBLE)
/// Floating point type of the exact arithmetic.
using CsgReal = double;
#else
/// Floating point type of the exact arithmetic.
using CsgReal = float;
#endif

/// Return the sign of an expansion: -1, 0 or 1. The sign is the sign of the last nonzero term.
PLUGIN_RBFXCSG_API int CsgExpansionSign(int count, const CsgReal* e);

/// Return the sum of the terms of an expansion as a double. The sum starts with the smallest term.
PLUGIN_RBFXCSG_API double CsgExpansionApproximate(int count, const CsgReal* e);

/// Fixed size expansion. Term 0 has the smallest magnitude. Zero has count_ equal to 0.
template <int N> struct CsgExactValue
{
    CsgReal terms_[N];
    int count_{};

    /// Return the sign of the value: -1, 0 or 1.
    int Sign() const { return CsgExpansionSign(count_, terms_); }
    /// Return the value rounded to a double. The terms are summed from the smallest one.
    double Approximate() const { return CsgExpansionApproximate(count_, terms_); }
};

/// Expansion for the values of a 2x2 minor: two products of two terms each.
/// The magnitude bound is CsgExactMinorBits in CsgExact.h.
using CsgExactMinor = CsgExactValue<4>;
/// Expansion for the values of a 3x3 determinant: three minors, each multiplied by one term.
/// The magnitude bound is CsgExactVolumeBits in CsgExact.h.
using CsgExactVolume = CsgExactValue<24>;

#if defined(CSG_EXACT_USE_DOUBLE)
/// The inputs of CsgExpansionProductDifferenceSign must have a magnitude below 2^CsgProductDifferenceInputBits.
constexpr int CsgProductDifferenceInputBits = 123;
#else
/// The inputs of CsgExpansionProductDifferenceSign must have a magnitude below 2^CsgProductDifferenceInputBits.
constexpr int CsgProductDifferenceInputBits = 75;
#endif

/// Exact sum of two numbers: a + b = result + error. The result is the rounded sum.
PLUGIN_RBFXCSG_API CsgReal CsgTwoSum(CsgReal a, CsgReal b, CsgReal& error);

/// Exact difference of two numbers: a - b = result + error. The result is the rounded difference.
PLUGIN_RBFXCSG_API CsgReal CsgTwoDiff(CsgReal a, CsgReal b, CsgReal& error);

/// Exact product of two numbers: a * b = result + error. The result is the rounded product.
/// The function uses the hardware FMA when it exists. Otherwise it uses the Dekker product.
PLUGIN_RBFXCSG_API CsgReal CsgTwoProduct(CsgReal a, CsgReal b, CsgReal& error);

/// Same as CsgTwoProduct. Always uses the Veltkamp split and the Dekker product.
PLUGIN_RBFXCSG_API CsgReal CsgTwoProductDekker(CsgReal a, CsgReal b, CsgReal& error);

#if defined(CSG_HAS_HARDWARE_FMA)
/// Same as CsgTwoProduct. Always uses the fused multiply-add.
PLUGIN_RBFXCSG_API CsgReal CsgTwoProductFma(CsgReal a, CsgReal b, CsgReal& error);
#endif

/// Multiply an expansion by a number. The result has no zero terms.
/// The capacity of out is 2 * count. Return the number of terms in out.
PLUGIN_RBFXCSG_API int CsgScaleExpansion(int count, const CsgReal* e, CsgReal b, CsgReal* out);

/// Add two expansions. The result has no zero terms.
/// The capacity of out is countE + countF. The out buffer must not overlap the inputs.
/// Both inputs must be strongly nonoverlapping. The output of CsgCompress has this property.
/// Other inputs can give an overlapping result and a wrong sign.
/// Return the number of terms in out.
PLUGIN_RBFXCSG_API int CsgExpansionSum(int countE, const CsgReal* e, int countF, const CsgReal* f, CsgReal* out);

/// Compress an expansion in place. The value stays the same. The new count is not larger.
/// Return the new number of terms.
PLUGIN_RBFXCSG_API int CsgCompress(int count, CsgReal* e);

/// Return the sign of e * f - g * h. The inputs are expansions.
/// The magnitude of each input must be below 2^CsgProductDifferenceInputBits.
/// Each term must be a multiple of an integer power of two that is not smaller than 1.
/// Each expansion has at most 24 terms.
PLUGIN_RBFXCSG_API int CsgExpansionProductDifferenceSign(
    int ce, const CsgReal* e, int cf, const CsgReal* f, int cg, const CsgReal* g, int ch, const CsgReal* h);

} // namespace Urho3D
