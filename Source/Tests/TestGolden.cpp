// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

// Golden comparison of boolean results. The test case has the hidden tag [.golden].
// A normal test run skips it. Run it with:
//   Tests.CSG "[.golden]"
//
// The test builds a fixed corpus of operands and boolean results. Each case gives one text line:
//   name;closed;openEdges;outOfRange;faces;triangles;volume6;vertexHash
// The line has no floating point value. The volume6 field is the exact sum of the triple products of the grid
// points of all triangles. It is six times the volume on the grid. The vertexHash field is the FNV-1a 64 hash
// of the sorted unique grid points.
//
// The environment variable CSG_GOLDEN_MODE selects the mode:
//   write    Write <dir>/golden-<float|double>.txt and pass.
//   compare  Write <dir>/golden-<float|double>-current.txt and compare it with <dir>/golden-<float|double>.txt.
//            This is the default.
// The environment variable CSG_GOLDEN_DIR selects the directory <dir>.
// The name float or double follows CSG_EXACT_USE_DOUBLE.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBoolean.h"
#include "Urho3D/CSG/CsgCorefine.h"
#include "Urho3D/CSG/CsgSolid.h"

#include "TestGeometry.h"
#include "TestInt128.h"

#include <Urho3D/Math/Matrix3x4.h>
#include <Urho3D/Math/Quaternion.h>

#include <catch2/catch_amalgamated.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#ifndef CSG_TEST_HAS_NATIVE_INT128
#error "The golden test needs a native 128 bit integer to print the volume."
#endif

using namespace Urho3D;
using namespace Urho3D::CsgInternal;

namespace
{

using SolidPtr = SharedPtr<const CsgSolid>;

#if defined(CSG_EXACT_USE_DOUBLE)
constexpr const char* MODE_NAME = "double";
#else
constexpr const char* MODE_NAME = "float";
#endif

constexpr const char* DEFAULT_GOLDEN_DIR = "/home/rk/src/rbfx/rbfx-csg/.backup/golden";

SolidPtr MakeSolid(ea::vector<CsgPolygon>&& polygons)
{
    return SolidPtr(CsgSolid::FromOperand(ea::move(polygons)));
}

SolidPtr MakeBox(const Vector3& center, unsigned shared, const Matrix3x4& transform = Matrix3x4::IDENTITY)
{
    return MakeSolid(MakeBoxPolygons(center, Vector3(1.0f, 1.0f, 1.0f), shared, transform));
}

SolidPtr MakeRotatedBox(const Vector3& center, unsigned shared)
{
    const Quaternion rotation(37.0f, Vector3(1.0f, 1.0f, 0.0f).Normalized());
    return MakeBox(center, shared, Matrix3x4(Vector3::ZERO, rotation, 1.0f));
}

/// A closed tetrahedron with outward faces. The corner order does not matter.
SolidPtr MakeTetrahedron(const Vector3 (&corners)[4], unsigned shared)
{
    static const int faces[4][3] = {{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}};
    const Vector3 center = (corners[0] + corners[1] + corners[2] + corners[3]) * 0.25f;
    ea::vector<CsgPolygon> polygons;
    for (const auto& face : faces)
    {
        Vector3 p[3] = {corners[face[0]], corners[face[1]], corners[face[2]]};
        if ((p[1] - p[0]).CrossProduct(p[2] - p[0]).DotProduct(center - p[0]) > 0.0f)
            ea::swap(p[1], p[2]);
        ea::vector<ModelVertex> vertices(3);
        for (int i = 0; i < 3; ++i)
            vertices[i].SetPosition(p[i]);
        polygons.push_back(CsgPolygon(ea::move(vertices), shared));
    }
    return MakeSolid(ea::move(polygons));
}

/// Return the polygons with each position moved by the transform.
ea::vector<CsgPolygon> TransformPolygons(const ea::vector<CsgPolygon>& polygons, const Matrix3x4& transform)
{
    ea::vector<CsgPolygon> result;
    for (const CsgPolygon& polygon : polygons)
    {
        ea::vector<ModelVertex> vertices;
        for (const ModelVertex& source : polygon.vertices_)
        {
            ModelVertex vertex;
            vertex.SetPosition(transform * source.GetPosition());
            vertices.push_back(vertex);
        }
        result.push_back(CsgPolygon(ea::move(vertices), polygon.shared_));
    }
    return result;
}

/// Return the decimal text of a signed 128 bit number.
std::string ToDecimal(CsgTest::Int128 value)
{
    const bool negative = value < 0;
    CsgTest::UInt128Native magnitude = negative ? static_cast<CsgTest::UInt128Native>(0) - static_cast<CsgTest::UInt128Native>(value)
                                                : static_cast<CsgTest::UInt128Native>(value);
    std::string digits;
    do
    {
        digits.push_back(static_cast<char>('0' + static_cast<int>(magnitude % 10)));
        magnitude /= 10;
    } while (magnitude != 0);
    if (negative)
        digits.push_back('-');
    std::reverse(digits.begin(), digits.end());
    return digits;
}

/// Return the determinant of three grid points. The coordinates are below 2^29.
/// A cross product component is below 2^59. Each term is below 2^88 and the sum of three is below 2^90.
CsgTest::Int128 Determinant(const CsgGridPoint& a, const CsgGridPoint& b, const CsgGridPoint& c)
{
    using CsgTest::Int128;
    using CsgTest::Mul64;
    const int64_t cx = int64_t{b.y_} * c.z_ - int64_t{b.z_} * c.y_;
    const int64_t cy = int64_t{b.z_} * c.x_ - int64_t{b.x_} * c.z_;
    const int64_t cz = int64_t{b.x_} * c.y_ - int64_t{b.y_} * c.x_;
    return Mul64<Int128>(a.x_, cx) + Mul64<Int128>(a.y_, cy) + Mul64<Int128>(a.z_, cz);
}

std::string Hex16(uint64_t value)
{
    char buffer[17];
    std::snprintf(buffer, sizeof(buffer), "%016llx", static_cast<unsigned long long>(value));
    return buffer;
}

/// Return the FNV-1a 64 hash of the sorted unique grid points of the solid.
/// Each coordinate adds its 8 bytes, least significant byte first.
uint64_t VertexHash(const CsgExactLoops& loops)
{
    std::vector<std::tuple<int64_t, int64_t, int64_t>> points;
    points.reserve(loops.ids_.size());
    for (const int id : loops.ids_)
    {
        const CsgGridPoint& g = loops.grid_[id];
        points.emplace_back(g.x_, g.y_, g.z_);
    }
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());

    uint64_t hash = 14695981039346656037ull;
    const auto add = [&](int64_t coordinate)
    {
        const uint64_t bits = static_cast<uint64_t>(coordinate);
        for (int i = 0; i < 8; ++i)
        {
            hash ^= (bits >> (8 * i)) & 0xff;
            hash *= 1099511628211ull;
        }
    };
    for (const auto& [x, y, z] : points)
    {
        add(x);
        add(y);
        add(z);
    }
    return hash;
}

/// Return the text line of a case.
std::string MakeRecord(const std::string& name, const SolidPtr& solid)
{
    const CsgExactLoops loops = solid->GetExactLoops();
    const auto triangleStart = solid->GetTriangleStart();
    const auto triangleVertices = solid->GetTriangleVertices();

    CsgTest::Int128 volume6 = 0;
    for (unsigned f = 0; f < solid->FaceCount(); ++f)
    {
        for (unsigned t = triangleStart[f]; t < triangleStart[f + 1]; ++t)
        {
            CsgGridPoint corner[3];
            for (unsigned k = 0; k < 3; ++k)
            {
                const unsigned local = triangleVertices[3 * t + k];
                const int id = loops.ids_[loops.vertexStart_[f] + local];
                corner[k] = loops.grid_[id];
            }
            volume6 += Determinant(corner[0], corner[1], corner[2]);
        }
    }

    const unsigned triangles = triangleStart.empty() ? 0u : triangleStart.back();
    std::string line = name;
    line += ';' + std::to_string(solid->IsClosed() ? 1 : 0);
    line += ';' + std::to_string(solid->OpenEdgeCount());
    line += ';' + std::to_string(solid->IsOutOfRange() ? 1 : 0);
    line += ';' + std::to_string(solid->FaceCount());
    line += ';' + std::to_string(triangles);
    line += ';' + ToDecimal(volume6);
    line += ';' + Hex16(VertexHash(loops));
    return line;
}

/// The corpus: the lines in a fixed order and the total time of the booleans.
class Corpus
{
public:
    void AddOperand(const std::string& name, const SolidPtr& solid)
    {
        Add("operand/" + name, solid);
    }

    SolidPtr Run(const std::string& name, const SolidPtr& a, const SolidPtr& b, CsgOperation operation)
    {
        const auto start = std::chrono::steady_clock::now();
        const SolidPtr result = Boolean(a, b, operation, CSG_DEFAULT_EPSILON);
        booleanSeconds_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        ++booleanCount_;
        if (!name.empty())
            Add(name, result);
        return result;
    }

    /// Run the three operations and the reverse difference of a pair of operands.
    void RunPair(const std::string& prefix, const SolidPtr& a, const SolidPtr& b)
    {
        Run(prefix + "/union", a, b, CsgOperation::Union);
        Run(prefix + "/difference", a, b, CsgOperation::DifferenceAB);
        Run(prefix + "/intersection", a, b, CsgOperation::Intersection);
        Run(prefix + "/difference-reverse", b, a, CsgOperation::DifferenceAB);
    }

    const std::vector<std::string>& GetLines() const { return lines_; }
    double GetBooleanSeconds() const { return booleanSeconds_; }
    unsigned GetBooleanCount() const { return booleanCount_; }
    unsigned GetDuplicateCount() const { return duplicates_; }

private:
    void Add(const std::string& name, const SolidPtr& solid)
    {
        if (!names_.insert(name).second)
            ++duplicates_;
        lines_.push_back(MakeRecord(name, solid));
    }

    std::vector<std::string> lines_;
    std::set<std::string> names_;
    double booleanSeconds_{};
    unsigned booleanCount_{};
    unsigned duplicates_{};
};

void AddShapes(Corpus& corpus)
{
    const auto addPair = [&](const std::string& name, const SolidPtr& a, const SolidPtr& b)
    {
        corpus.AddOperand("shapes/" + name + "/a", a);
        corpus.AddOperand("shapes/" + name + "/b", b);
        corpus.RunPair("shapes/" + name, a, b);
    };

    const auto box = MakeBox(Vector3::ZERO, 1);
    addPair("overlap", box, MakeBox(Vector3(1.0f, 0.0f, 0.0f), 2));
    addPair("sharedface", box, MakeBox(Vector3(2.0f, 0.0f, 0.0f), 2));
    addPair("rotatedbox", box, MakeRotatedBox(Vector3(0.3f, 0.2f, 0.1f), 2));
    addPair("spherebox", MakeSolid(MakeSpherePolygons(Vector3(0.1f, 0.2f, 0.35f), 1.3f, 12, 16, 1)),
        MakeBox(Vector3::ZERO, 2));
    addPair("sphereinside", box, MakeSolid(MakeSpherePolygons(Vector3(0.1f, 0.0f, 0.0f), 0.5f, 8, 12, 2)));
    addPair("smallinside", box,
        MakeSolid(MakeBoxPolygons(Vector3(0.1f, 0.2f, -0.1f), Vector3(0.25f, 0.25f, 0.25f), 2)));
    addPair("smalloutside", box,
        MakeSolid(MakeBoxPolygons(Vector3(3.0f, 0.5f, -0.5f), Vector3(0.25f, 0.25f, 0.25f), 2)));
    for (int x = -2; x <= 2; x += 2)
    {
        for (int y = -2; y <= 2; y += 2)
        {
            for (int z = -2; z <= 2; z += 2)
            {
                if (x == 0 && y == 0 && z == 0)
                    continue;
                addPair("touch/" + std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(z), box,
                    MakeBox(Vector3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)), 2));
            }
        }
    }
    addPair("disjoint", box, MakeBox(Vector3(5.0f, 0.5f, 0.0f), 2));
    for (const int step : {9, 24, 37, 50, 109, 116})
    {
        const float angle = 0.3f + 0.7f * static_cast<float>(step);
        const Matrix3x4 transform(Vector3::ZERO, Quaternion(angle, Vector3::ONE.Normalized()), 1.0f);
        addPair("cornerOnEdge/" + std::to_string(step), MakeBox(Vector3::ZERO, 1, transform),
            MakeBox(Vector3(1.0f, 0.0f, 0.0f), 2, transform));
    }

    // The zero sum constructions.
    const Vector3 apex(0.25f, 1.0f, 0.5f);
    for (const float side : {1.0f, -1.0f})
    {
        const Vector3 corners[4] = {apex, Vector3(-0.5f, 1.0f + side, -0.5f), Vector3(1.0f, 1.0f + side, -0.25f),
            Vector3(0.0f, 1.0f + side, 0.75f)};
        addPair(side > 0.0f ? "zerosum/tetrahedron/above" : "zerosum/tetrahedron/below",
            MakeTetrahedron(corners, 2), box);
    }
    {
        const float w = 0.2f / CsgGridScale;
        const Vector3 corners[4] = {Vector3(0.25f, 0.0f, 0.5f), Vector3(0.25f + w, 0.0f, 0.5f),
            Vector3(0.25f, 0.0f, 0.5f + w), Vector3(0.25f, 2.0f, 0.5f)};
        addPair("zerosum/needle", box, MakeTetrahedron(corners, 2));
    }
    {
        const float h = 0.2f / CsgGridScale;
        addPair("zerosum/plate", box,
            MakeSolid(MakeBoxPolygons(Vector3(1.0f, 0.0f, 0.0f), Vector3(1.0f, h, 0.75f), 2)));
    }
    {
        const Matrix3x4 transform(Vector3::ZERO, Quaternion(18.5f, Vector3::ONE.Normalized()), 1.0f);
        addPair("zerosum/coplanar", MakeBox(Vector3::ZERO, 1, transform),
            MakeBox(Vector3(1.0f, 0.0f, 0.0f), 2, transform));
    }
}

void AddSweep(Corpus& corpus)
{
    struct Axis
    {
        const char* name_;
        Vector3 axis_;
    };
    const Axis axes[] = {{"x", Vector3::RIGHT}, {"y", Vector3::UP}, {"z", Vector3::FORWARD}, {"xyz", Vector3::ONE}};
    for (const Axis& axis : axes)
    {
        for (unsigned i = 0;; ++i)
        {
            const float angle = 0.3f + 0.7f * static_cast<float>(i);
            if (angle > 180.0f)
                break;
            char angleText[32];
            std::snprintf(angleText, sizeof(angleText), "%.4f", static_cast<double>(angle));
            const std::string prefix = std::string("sweep/") + axis.name_ + "/" + angleText;

            const Quaternion world(angle, axis.axis_.Normalized());
            const Quaternion own(37.0f, Vector3(1.0f, 1.0f, 0.0f).Normalized());
            const Matrix3x4 worldTransform(Vector3::ZERO, world, 1.0f);
            const auto a = MakeBox(Vector3::ZERO, 1, worldTransform);
            const auto b = MakeBox(Vector3(1.0f, 0.0f, 0.0f), 2, worldTransform);
            const auto c = MakeBox(Vector3(0.5f, 1.0f, 0.3f), 3, Matrix3x4(Vector3::ZERO, world * own, 1.0f));
            corpus.AddOperand(prefix + "/a", a);
            corpus.AddOperand(prefix + "/b", b);
            corpus.AddOperand(prefix + "/c", c);

            const auto ab = corpus.Run(prefix + "/ab", a, b, CsgOperation::Union);
            corpus.Run(prefix + "/left", ab, c, CsgOperation::Union);
            const auto bc = corpus.Run("", b, c, CsgOperation::Union);
            corpus.Run(prefix + "/right", a, bc, CsgOperation::Union);
            corpus.Run(prefix + "/difference", a, b, CsgOperation::DifferenceAB);
            corpus.Run(prefix + "/intersection", a, b, CsgOperation::Intersection);
        }
    }
}

/// Deterministic random numbers. The class takes only the bits of the engine, so the result does not depend
/// on the standard library.
class Random
{
public:
    explicit Random(uint64_t seed)
        : engine_(seed)
    {
    }

    bool Bit() { return (engine_() >> 63) != 0; }

    /// Return a float in [low, high), rounded down to a multiple of 1/64.
    float Float(float low, float high)
    {
        const float unit = static_cast<float>(engine_() >> 40) / 16777216.0f;
        const float value = low + (high - low) * unit;
        return std::floor(value * 64.0f) / 64.0f;
    }

private:
    std::mt19937_64 engine_;
};

/// Draw one brush of a chain: a box or a sphere with a random size, position and rotation.
SolidPtr MakeBrush(Random& random, unsigned shared)
{
    const bool isBox = random.Bit();
    const float size = random.Float(0.3f, 1.5f);
    const Vector3 center(random.Float(-2.0f, 2.0f), random.Float(-2.0f, 2.0f), random.Float(-2.0f, 2.0f));
    Vector3 axis(random.Float(-1.0f, 1.0f), random.Float(-1.0f, 1.0f), random.Float(-1.0f, 1.0f));
    const float angle = random.Float(0.0f, 180.0f);
    if (axis.LengthSquared() < 1e-6f)
        axis = Vector3::UP;
    const Quaternion rotation(angle, axis.Normalized());
    if (isBox)
    {
        return MakeSolid(MakeBoxPolygons(
            center, Vector3(size, size, size), shared, Matrix3x4(Vector3::ZERO, rotation, 1.0f)));
    }
    // The sphere turns around its own center.
    return MakeSolid(TransformPolygons(
        MakeSpherePolygons(Vector3::ZERO, size, 6, 8, shared), Matrix3x4(center, rotation, 1.0f)));
}

void AddChains(Corpus& corpus)
{
    constexpr unsigned CHAIN_COUNT = 20;
    constexpr unsigned BRUSH_COUNT = 8;
    const CsgOperation operations[] = {CsgOperation::Union, CsgOperation::DifferenceAB, CsgOperation::Intersection};
    Random random(20261027);
    for (unsigned n = 0; n < CHAIN_COUNT; ++n)
    {
        const std::string prefix = "chain/" + std::to_string(n);
        std::vector<SolidPtr> brushes;
        for (unsigned i = 0; i < BRUSH_COUNT; ++i)
        {
            brushes.push_back(MakeBrush(random, i + 1));
            corpus.AddOperand(prefix + "/brush" + std::to_string(i), brushes.back());
        }
        // Link i joins the result so far with brush i + 1. The first result so far is brush 0.
        SolidPtr current = brushes[0];
        for (unsigned i = 0; i + 1 < BRUSH_COUNT; ++i)
            current = corpus.Run(prefix + "/" + std::to_string(i), current, brushes[i + 1], operations[i % 3]);
    }
}

std::string GetEnv(const char* name, const char* fallback)
{
    const char* value = std::getenv(name);
    return value && *value ? value : fallback;
}

/// Return the text before the first ';'.
std::string LineName(const std::string& line)
{
    return line.substr(0, line.find(';'));
}

/// Return the number of cases with a result that is not closed. The second field of a line is closed.
unsigned CountOpen(const std::vector<std::string>& lines)
{
    unsigned count = 0;
    for (const std::string& line : lines)
    {
        const size_t first = line.find(';');
        if (first != std::string::npos && line.compare(first + 1, 2, "0;") == 0)
            ++count;
    }
    return count;
}

void WriteLines(const std::filesystem::path& path, const std::vector<std::string>& lines)
{
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    REQUIRE(file.good());
    for (const std::string& line : lines)
        file << line << '\n';
}

} // namespace

TEST_CASE("Golden corpus of boolean results", "[.golden]")
{
    CsgInternal::g_csgDiagnostics.Reset();
    Corpus corpus;
    AddShapes(corpus);
    AddSweep(corpus);
    AddChains(corpus);
    const int failedSplits = CsgInternal::g_csgDiagnostics.failedSplits_.load();
    const int failedRegionFits = CsgInternal::g_csgDiagnostics.failedRegionFits_.load();
    const std::vector<std::string>& lines = corpus.GetLines();
    INFO("booleans " << corpus.GetBooleanCount() << ", lines " << lines.size() << ", boolean wall time "
                     << corpus.GetBooleanSeconds() << " s");
    // The wall time is also visible with a passing run.
    std::printf("golden: %zu lines, %u booleans, boolean wall time %.3f s\n", lines.size(), corpus.GetBooleanCount(),
        corpus.GetBooleanSeconds());
    std::printf("golden: failedSplits %d, failedRegionFits %d\n", failedSplits, failedRegionFits);
    REQUIRE(corpus.GetDuplicateCount() == 0);

    const std::filesystem::path directory = GetEnv("CSG_GOLDEN_DIR", DEFAULT_GOLDEN_DIR);
    std::filesystem::create_directories(directory);
    const std::string baseName = std::string("golden-") + MODE_NAME;
    const std::filesystem::path baselinePath = directory / (baseName + ".txt");

    if (GetEnv("CSG_GOLDEN_MODE", "compare") == "write")
    {
        WriteLines(baselinePath, lines);
        return;
    }

    WriteLines(directory / (baseName + "-current.txt"), lines);

    std::ifstream baselineFile(baselinePath, std::ios::binary);
    INFO("baseline " << baselinePath.string());
    REQUIRE(baselineFile.good());
    std::vector<std::string> baselineLines;
    std::map<std::string, std::string> baseline;
    for (std::string line; std::getline(baselineFile, line);)
    {
        baseline[LineName(line)] = line;
        baselineLines.push_back(line);
    }

    unsigned diffCount = 0;
    std::set<std::string> currentNames;
    for (const std::string& line : lines)
    {
        const std::string name = LineName(line);
        currentNames.insert(name);
        const auto found = baseline.find(name);
        if (found == baseline.end())
        {
            ++diffCount;
            UNSCOPED_INFO("missing in baseline: " << line);
        }
        else if (found->second != line)
        {
            ++diffCount;
            UNSCOPED_INFO("baseline: " << found->second << "\ncurrent:  " << line);
        }
    }
    for (const auto& [name, line] : baseline)
    {
        if (currentNames.count(name) == 0)
        {
            ++diffCount;
            UNSCOPED_INFO("missing in current: " << line);
        }
    }

    const unsigned openNow = CountOpen(lines);
    const unsigned openBaseline = CountOpen(baselineLines);
    INFO("open cases now " << openNow << ", in baseline " << openBaseline << ", diffs " << diffCount);
    // Catch attaches the unscoped messages to the next assertion only. The diff check comes first for this reason.
    CHECK(diffCount == 0);
    CHECK(openNow <= openBaseline);
    // The pipeline must not fail on the corpus.
    CHECK(failedSplits == 0);
    CHECK(failedRegionFits == 0);
}
