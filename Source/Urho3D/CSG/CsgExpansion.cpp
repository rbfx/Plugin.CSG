// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/CsgExpansion.h"

#include <Urho3D/Core/AssertBase.h>

#include <cfloat>
#include <cstdint>
#include <limits>

static_assert(FLT_EVAL_METHOD == 0, "The exact CSG arithmetic needs evaluation in the type of the operands.");
static_assert(std::numeric_limits<Urho3D::CsgReal>::is_iec559, "The exact CSG arithmetic needs IEEE 754 numbers.");

namespace Urho3D
{

namespace
{

/// The maximum number of terms in an input expansion of the product difference.
constexpr int MaxInputTerms = 24;
/// The maximum number of terms in the product of two input expansions.
constexpr int MaxProductTerms = 2 * MaxInputTerms * MaxInputTerms;

/// Veltkamp splitter: 2^ceil(digits / 2) + 1. It is 4097 for float and 134217729 for double.
constexpr CsgReal Splitter =
    static_cast<CsgReal>(std::uint64_t{1} << ((std::numeric_limits<CsgReal>::digits + 1) / 2)) + CsgReal(1);

/// Scale factor 2^-80. It makes the product of two 2^75 values fit in the range of float.
constexpr CsgReal InputScale = CsgReal(0x1p-80);

/// Exact sum when |a| >= |b|.
inline CsgReal FastTwoSum(CsgReal a, CsgReal b, CsgReal& error)
{
    const CsgReal x = a + b;
    const CsgReal bVirtual = x - a;
    error = b - bVirtual;
    return x;
}

/// Split a number to two halves. The sum of the halves is the number. Each half has half of the bits.
inline void Split(CsgReal a, CsgReal& high, CsgReal& low)
{
    const CsgReal c = Splitter * a;
    const CsgReal aBig = c - a;
    high = c - aBig;
    low = a - high;
}

/// Add the term to the output when it is not zero.
inline void PushNonZero(CsgReal* out, int& count, CsgReal value)
{
    if (value != CsgReal(0))
        out[count++] = value;
}

/// Compute the sum of the products of the terms of f with the expansion e.
/// Use two buffers of the same size alternately. Return the buffer with the result in result.
int ProductExpansion(int ce, const CsgReal* e, int cf, const CsgReal* f, bool negate, CsgReal* bufferA,
    CsgReal* bufferB, CsgReal*& result)
{
    CsgReal scaled[2 * MaxInputTerms];
    CsgReal* current = bufferA;
    CsgReal* next = bufferB;
    int count = 0;
    for (int i = 0; i < cf; ++i)
    {
        const int scaledCount = CsgScaleExpansion(ce, e, negate ? -f[i] : f[i], scaled);
        if (count == 0)
        {
            for (int j = 0; j < scaledCount; ++j)
                current[j] = scaled[j];
            count = scaledCount;
        }
        else
        {
            URHO3D_ASSERT(count + scaledCount <= MaxProductTerms, "CSG product expansion is too long");
            count = CsgExpansionSum(count, current, scaledCount, scaled, next);
            CsgReal* const swap = current;
            current = next;
            next = swap;
        }
    }
    result = current;
    return count;
}

} // namespace

int CsgExpansionSign(int count, const CsgReal* e)
{
    for (int i = count - 1; i >= 0; --i)
    {
        if (e[i] > CsgReal(0))
            return 1;
        if (e[i] < CsgReal(0))
            return -1;
    }
    return 0;
}

double CsgExpansionApproximate(int count, const CsgReal* e)
{
    double sum = 0.0;
    for (int i = 0; i < count; ++i)
        sum += static_cast<double>(e[i]);
    return sum;
}

CsgReal CsgTwoSum(CsgReal a, CsgReal b, CsgReal& error)
{
    const CsgReal x = a + b;
    const CsgReal bVirtual = x - a;
    const CsgReal aVirtual = x - bVirtual;
    const CsgReal bRoundoff = b - bVirtual;
    const CsgReal aRoundoff = a - aVirtual;
    error = aRoundoff + bRoundoff;
    return x;
}

CsgReal CsgTwoDiff(CsgReal a, CsgReal b, CsgReal& error)
{
    const CsgReal x = a - b;
    const CsgReal bVirtual = a - x;
    const CsgReal aVirtual = x + bVirtual;
    const CsgReal bRoundoff = bVirtual - b;
    const CsgReal aRoundoff = a - aVirtual;
    error = aRoundoff + bRoundoff;
    return x;
}

CsgReal CsgTwoProductDekker(CsgReal a, CsgReal b, CsgReal& error)
{
    const CsgReal x = a * b;
    CsgReal aHigh, aLow, bHigh, bLow;
    Split(a, aHigh, aLow);
    Split(b, bHigh, bLow);
    const CsgReal error1 = x - aHigh * bHigh;
    const CsgReal error2 = error1 - aLow * bHigh;
    const CsgReal error3 = error2 - aHigh * bLow;
    error = aLow * bLow - error3;
    return x;
}

#if defined(CSG_HAS_HARDWARE_FMA)
CsgReal CsgTwoProductFma(CsgReal a, CsgReal b, CsgReal& error)
{
    const CsgReal x = a * b;
    error = std::fma(a, b, -x);
    return x;
}
#endif

CsgReal CsgTwoProduct(CsgReal a, CsgReal b, CsgReal& error)
{
#if defined(CSG_HAS_HARDWARE_FMA)
    return CsgTwoProductFma(a, b, error);
#else
    return CsgTwoProductDekker(a, b, error);
#endif
}

int CsgScaleExpansion(int count, const CsgReal* e, CsgReal b, CsgReal* out)
{
    if (count <= 0)
        return 0;

    int outCount = 0;
    CsgReal error;
    CsgReal q = CsgTwoProduct(e[0], b, error);
    PushNonZero(out, outCount, error);
    for (int i = 1; i < count; ++i)
    {
        CsgReal productLow;
        const CsgReal productHigh = CsgTwoProduct(e[i], b, productLow);
        CsgReal sum;
        sum = CsgTwoSum(q, productLow, error);
        PushNonZero(out, outCount, error);
        q = FastTwoSum(productHigh, sum, error);
        PushNonZero(out, outCount, error);
    }
    PushNonZero(out, outCount, q);
    return outCount;
}

int CsgExpansionSum(int countE, const CsgReal* e, int countF, const CsgReal* f, CsgReal* out)
{
    if (countF <= 0)
    {
        for (int i = 0; i < countE; ++i)
            out[i] = e[i];
        return countE > 0 ? countE : 0;
    }
    if (countE <= 0)
    {
        for (int i = 0; i < countF; ++i)
            out[i] = f[i];
        return countF;
    }

    int eIndex = 0;
    int fIndex = 0;
    int outCount = 0;
    CsgReal eNow = e[0];
    CsgReal fNow = f[0];
    CsgReal q;
    CsgReal qNew;
    CsgReal error;

    // The term with the smaller magnitude is taken first.
    if ((fNow > eNow) == (fNow > -eNow))
    {
        q = eNow;
        eNow = ++eIndex < countE ? e[eIndex] : CsgReal(0);
    }
    else
    {
        q = fNow;
        fNow = ++fIndex < countF ? f[fIndex] : CsgReal(0);
    }

    if (eIndex < countE && fIndex < countF)
    {
        if ((fNow > eNow) == (fNow > -eNow))
        {
            qNew = FastTwoSum(eNow, q, error);
            eNow = ++eIndex < countE ? e[eIndex] : CsgReal(0);
        }
        else
        {
            qNew = FastTwoSum(fNow, q, error);
            fNow = ++fIndex < countF ? f[fIndex] : CsgReal(0);
        }
        q = qNew;
        PushNonZero(out, outCount, error);
        while (eIndex < countE && fIndex < countF)
        {
            if ((fNow > eNow) == (fNow > -eNow))
            {
                qNew = CsgTwoSum(q, eNow, error);
                eNow = ++eIndex < countE ? e[eIndex] : CsgReal(0);
            }
            else
            {
                qNew = CsgTwoSum(q, fNow, error);
                fNow = ++fIndex < countF ? f[fIndex] : CsgReal(0);
            }
            q = qNew;
            PushNonZero(out, outCount, error);
        }
    }
    while (eIndex < countE)
    {
        qNew = CsgTwoSum(q, eNow, error);
        eNow = ++eIndex < countE ? e[eIndex] : CsgReal(0);
        q = qNew;
        PushNonZero(out, outCount, error);
    }
    while (fIndex < countF)
    {
        qNew = CsgTwoSum(q, fNow, error);
        fNow = ++fIndex < countF ? f[fIndex] : CsgReal(0);
        q = qNew;
        PushNonZero(out, outCount, error);
    }
    PushNonZero(out, outCount, q);
    return outCount;
}

int CsgCompress(int count, CsgReal* e)
{
    if (count <= 0)
        return 0;

    // Pass 1: from the largest term down. Collect the rounded sums in the upper part of the array.
    int bottom = count - 1;
    CsgReal q = e[bottom];
    for (int i = count - 2; i >= 0; --i)
    {
        CsgReal error;
        const CsgReal qNew = FastTwoSum(q, e[i], error);
        if (error != CsgReal(0))
        {
            e[bottom--] = qNew;
            q = error;
        }
        else
        {
            q = qNew;
        }
    }

    // Pass 2: from the smallest term up. The write position stays below the read position.
    int top = 0;
    for (int i = bottom + 1; i < count; ++i)
    {
        CsgReal error;
        const CsgReal qNew = FastTwoSum(e[i], q, error);
        if (error != CsgReal(0))
            e[top++] = error;
        q = qNew;
    }
    if (q != CsgReal(0))
        e[top++] = q;
    return top;
}

int CsgExpansionProductDifferenceSign(
    int ce, const CsgReal* e, int cf, const CsgReal* f, int cg, const CsgReal* g, int ch, const CsgReal* h)
{
    URHO3D_ASSERT(ce <= MaxInputTerms && cf <= MaxInputTerms && cg <= MaxInputTerms && ch <= MaxInputTerms,
        "CSG input expansion is too long");

    const int firstSign = CsgExpansionSign(ce, e) * CsgExpansionSign(cf, f);
    const int secondSign = CsgExpansionSign(cg, g) * CsgExpansionSign(ch, h);
    if (firstSign == 0)
        return -secondSign;
    if (secondSign == 0)
        return firstSign;
    if (firstSign != secondSign)
        return firstSign;

    // Copy and compress the inputs. The second factors get the scale 2^-80, which is exact.
    CsgReal eCopy[MaxInputTerms];
    CsgReal fCopy[MaxInputTerms];
    CsgReal gCopy[MaxInputTerms];
    CsgReal hCopy[MaxInputTerms];
    for (int i = 0; i < ce; ++i)
        eCopy[i] = e[i];
    for (int i = 0; i < cf; ++i)
        fCopy[i] = f[i];
    for (int i = 0; i < cg; ++i)
        gCopy[i] = g[i];
    for (int i = 0; i < ch; ++i)
        hCopy[i] = h[i];
    ce = CsgCompress(ce, eCopy);
    cf = CsgCompress(cf, fCopy);
    cg = CsgCompress(cg, gCopy);
    ch = CsgCompress(ch, hCopy);
    for (int i = 0; i < cf; ++i)
        fCopy[i] *= InputScale;
    for (int i = 0; i < ch; ++i)
        hCopy[i] *= InputScale;

    // Three buffers. The first product uses buffers 0 and 1. The second product uses the free one and buffer 2.
    CsgReal buffers[3][MaxProductTerms];
    CsgReal* first = nullptr;
    const int firstCount = ProductExpansion(ce, eCopy, cf, fCopy, false, buffers[0], buffers[1], first);
    CsgReal* freeBuffer = first == buffers[0] ? buffers[1] : buffers[0];
    CsgReal* second = nullptr;
    // The second product is negated, so the sum of both products is the difference.
    const int secondCount = ProductExpansion(cg, gCopy, ch, hCopy, true, freeBuffer, buffers[2], second);
    CsgReal* const sumBuffer = second == freeBuffer ? buffers[2] : freeBuffer;
    URHO3D_ASSERT(firstCount + secondCount <= MaxProductTerms, "CSG product difference expansion is too long");
    const int sumCount = CsgExpansionSum(firstCount, first, secondCount, second, sumBuffer);
    return CsgExpansionSign(sumCount, sumBuffer);
}

} // namespace Urho3D
