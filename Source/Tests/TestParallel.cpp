// Copyright (c) 2025-2026 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#include "Urho3D/CSG/Csg.h"
#include "Urho3D/CSG/CsgBoolean.h"
#include "Urho3D/CSG/CsgFaceBvh.h"
#include "Urho3D/CSG/CsgSolid.h"

#include "TestGeometry.h"

#include <Urho3D/Core/Context.h>
#include <Urho3D/Core/WorkQueue.h>

#include <catch2/catch_amalgamated.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <limits>
#include <random>

using namespace Urho3D;
using namespace Urho3D::CsgInternal;

namespace
{

using SolidPtr = SharedPtr<const CsgSolid>;
using Pairs = ea::vector<CsgFaceBvh::ItemPair>;

constexpr unsigned WORKER_COUNT = 4;

SharedPtr<Context> MakeParallelContext(unsigned threads)
{
    auto context = MakeShared<Context>();
    context->RegisterSubsystem<WorkQueue>();
    if (threads > 1)
        context->GetSubsystem<WorkQueue>()->Initialize(threads);
    return context;
}

SolidPtr MakeSphere(const Vector3& center, float radius, unsigned rings, unsigned segments)
{
    return SolidPtr(CsgSolid::FromOperand(MakeSpherePolygons(center, radius, rings, segments, 1)));
}

/// Two spheres with different tessellations. Both have more faces than the parallel threshold.
SolidPtr MakeBigA()
{
    return MakeSphere(Vector3(0.0f, 0.0f, 0.0f), 1.0f, 48, 96);
}

SolidPtr MakeBigB()
{
    return MakeSphere(Vector3(0.6f, 0.3f, 0.2f), 0.9f, 56, 80);
}

Pairs Collect(const CsgFaceBvh& a, const CsgFaceBvh& b, const CsgParallelOptions& options)
{
    Pairs output;
    Pairs scratch;
    a.CollectOverlaps(b, 1e-5f, output, scratch, options);
    return output;
}

CsgParallelOptions Options(WorkQueue* queue, CsgParallelMode mode)
{
    CsgParallelOptions options;
    options.workQueue_ = queue;
    options.mode_ = mode;
    return options;
}

bool SameSolid(const CsgSolid& a, const CsgSolid& b)
{
    if (a.FaceCount() != b.FaceCount())
        return false;
    const auto polygonsA = a.ToPolygons();
    const auto polygonsB = b.ToPolygons();
    for (unsigned i = 0; i < polygonsA.size(); ++i)
    {
        if (polygonsA[i].vertices_.size() != polygonsB[i].vertices_.size() || polygonsA[i].shared_ != polygonsB[i].shared_)
            return false;
        for (unsigned j = 0; j < polygonsA[i].vertices_.size(); ++j)
        {
            const Vector4& p = polygonsA[i].vertices_[j].position_;
            const Vector4& q = polygonsB[i].vertices_[j].position_;
            if (std::memcmp(&p, &q, sizeof(Vector4)) != 0)
                return false;
        }
    }
    return true;
}

double Volume(const SolidPtr& solid)
{
    return SignedVolume(solid->ToPolygons());
}

} // namespace

TEST_CASE("Parallel: options decide if tasks run", "[parallel]")
{
    {
        const CsgParallelOptions none;
        CHECK_FALSE(CsgCanRunParallel(none));
    }
    {
        auto context = MakeParallelContext(1);
        auto* queue = context->GetSubsystem<WorkQueue>();
        CHECK_FALSE(CsgCanRunParallel(Options(queue, CsgParallelMode::Parallel)));
    }
    {
        auto context = MakeParallelContext(WORKER_COUNT);
        auto* queue = context->GetSubsystem<WorkQueue>();
        CHECK(CsgCanRunParallel(Options(queue, CsgParallelMode::Auto)));
        CHECK(CsgCanRunParallel(Options(queue, CsgParallelMode::Parallel)));
        CHECK_FALSE(CsgCanRunParallel(Options(queue, CsgParallelMode::Serial)));
    }
}

TEST_CASE("Parallel: overlap pairs equal the serial pairs", "[parallel]")
{
    auto context = MakeParallelContext(WORKER_COUNT);
    auto* queue = context->GetSubsystem<WorkQueue>();
    const SolidPtr a = MakeBigA();
    const SolidPtr b = MakeBigB();
    REQUIRE(a->FaceCount() >= CSG_PARALLEL_MIN_ITEMS);
    REQUIRE(b->FaceCount() >= CSG_PARALLEL_MIN_ITEMS);
    const CsgFaceBvh treeA(a->GetFaceMin(), a->GetFaceMax());
    const CsgFaceBvh treeB(b->GetFaceMin(), b->GetFaceMax());

    const Pairs serial = Collect(treeA, treeB, Options(queue, CsgParallelMode::Serial));
    REQUIRE(serial.size() > 100);

    Pairs plain;
    Pairs scratch;
    treeA.CollectOverlaps(treeB, 1e-5f, plain, scratch);
    CHECK(plain == serial);

    for (int run = 0; run < 20; ++run)
    {
        CHECK(Collect(treeA, treeB, Options(queue, CsgParallelMode::Parallel)) == serial);
        CHECK(Collect(treeA, treeB, Options(queue, CsgParallelMode::Auto)) == serial);
    }
}

TEST_CASE("Parallel: the parallel run appends to the output", "[parallel]")
{
    auto context = MakeParallelContext(WORKER_COUNT);
    auto* queue = context->GetSubsystem<WorkQueue>();
    const SolidPtr a = MakeBigA();
    const SolidPtr b = MakeBigB();
    const CsgFaceBvh treeA(a->GetFaceMin(), a->GetFaceMax());
    const CsgFaceBvh treeB(b->GetFaceMin(), b->GetFaceMax());
    const Pairs serial = Collect(treeA, treeB, Options(queue, CsgParallelMode::Serial));

    Pairs output;
    output.push_back({-1, -1});
    Pairs scratch;
    treeA.CollectOverlaps(treeB, 1e-5f, output, scratch, Options(queue, CsgParallelMode::Parallel));
    REQUIRE(output.size() == serial.size() + 1);
    CHECK(output[0] == CsgFaceBvh::ItemPair(-1, -1));
    CHECK(Pairs(output.begin() + 1, output.end()) == serial);
}

TEST_CASE("Parallel: boxes with NaN and infinity do not hang", "[parallel]")
{
    auto context = MakeParallelContext(WORKER_COUNT);
    auto* queue = context->GetSubsystem<WorkQueue>();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    const auto makeBoxes = [&](unsigned seed, ea::vector<Vector3>& mins, ea::vector<Vector3>& maxs)
    {
        std::mt19937 random(seed);
        std::uniform_real_distribution<float> position(0.0f, 10.0f);
        std::uniform_real_distribution<float> size(0.0f, 0.5f);
        for (unsigned i = 0; i < 3000; ++i)
        {
            Vector3 min(position(random), position(random), position(random));
            Vector3 max = min + Vector3(size(random), size(random), size(random));
            if (i % 211 == 0)
                min.x_ = nan;
            if (i % 307 == 0)
                max = Vector3(inf, inf, inf);
            if (i % 401 == 0)
                min = Vector3(-inf, -inf, -inf);
            mins.push_back(min);
            maxs.push_back(max);
        }
    };
    ea::vector<Vector3> minA, maxA, minB, maxB;
    makeBoxes(1, minA, maxA);
    makeBoxes(2, minB, maxB);
    const CsgFaceBvh treeA(minA, maxA);
    const CsgFaceBvh treeB(minB, maxB);

    const Pairs serial = Collect(treeA, treeB, Options(queue, CsgParallelMode::Serial));
    CHECK(serial.size() > 0);
    CHECK(Collect(treeA, treeB, Options(queue, CsgParallelMode::Parallel)) == serial);
}

TEST_CASE("Parallel: boolean result equals the serial result", "[parallel]")
{
    auto context = MakeParallelContext(WORKER_COUNT);
    auto* queue = context->GetSubsystem<WorkQueue>();
    const SolidPtr a = MakeBigA();
    const SolidPtr b = MakeBigB();

    const CsgOperation operations[] = {CsgOperation::Union, CsgOperation::DifferenceAB, CsgOperation::Intersection};
    for (const CsgOperation operation : operations)
    {
        const SolidPtr serial =
            Boolean(a, b, operation, CSG_DEFAULT_EPSILON, Options(queue, CsgParallelMode::Serial));
        REQUIRE(serial->FaceCount() > 0);
        for (int run = 0; run < 20; ++run)
        {
            const SolidPtr parallel =
                Boolean(a, b, operation, CSG_DEFAULT_EPSILON, Options(queue, CsgParallelMode::Parallel));
            REQUIRE(parallel->FaceCount() == serial->FaceCount());
            REQUIRE(Volume(parallel) == Volume(serial));
            REQUIRE(SameSolid(*parallel, *serial));
        }
    }

    // The public function with the work queue gives the same result.
    const SolidPtr serial = CsgBooleanOperation(a, b, CsgOperation::Union);
    const SolidPtr parallel = CsgBooleanOperation(a, b, CsgOperation::Union, CSG_DEFAULT_EPSILON, queue);
    CHECK(SameSolid(*parallel, *serial));
}

TEST_CASE("Parallel: a call inside a worker task runs serial and does not deadlock", "[parallel]")
{
    auto context = MakeParallelContext(WORKER_COUNT);
    auto* queue = context->GetSubsystem<WorkQueue>();
    const SolidPtr a = MakeBigA();
    const SolidPtr b = MakeBigB();
    const CsgFaceBvh treeA(a->GetFaceMin(), a->GetFaceMax());
    const CsgFaceBvh treeB(b->GetFaceMin(), b->GetFaceMax());
    const Pairs serial = Collect(treeA, treeB, Options(queue, CsgParallelMode::Serial));
    const SolidPtr serialSolid = Boolean(a, b, CsgOperation::Union, CSG_DEFAULT_EPSILON);

    // The tasks are pinned to the worker threads. They must run serial, also with the forced parallel mode.
    // No foreign task runs on the main thread here, because such a task must not post immediate tasks.
    std::atomic<int> workerAllowed{0};
    std::atomic<int> workerMismatch{0};
    std::atomic<int> done{0};
    for (unsigned thread = 1; thread < WORKER_COUNT; ++thread)
    {
        queue->PostTaskForThread(
            [&](unsigned threadIndex, WorkQueue*)
            {
                const CsgParallelOptions options = Options(queue, CsgParallelMode::Parallel);
                if (threadIndex != 0 && CsgCanRunParallel(options))
                    ++workerAllowed;
                if (Collect(treeA, treeB, options) != serial)
                    ++workerMismatch;
                const SolidPtr result = Boolean(a, b, CsgOperation::Union, CSG_DEFAULT_EPSILON, options);
                if (!SameSolid(*result, *serialSolid))
                    ++workerMismatch;
                ++done;
            },
            TaskPriority::Medium, thread);
    }
    queue->CompleteAll();

    CHECK(done == static_cast<int>(WORKER_COUNT - 1));
    CHECK(workerAllowed == 0);
    CHECK(workerMismatch == 0);
}

TEST_CASE("Parallel: timing of the overlap collection", "[parallel][.timing]")
{
    auto context = MakeParallelContext(WORKER_COUNT);
    auto* queue = context->GetSubsystem<WorkQueue>();
    const SolidPtr a = MakeSphere(Vector3(0, 0, 0), 1.0f, 200, 400);
    const SolidPtr b = MakeSphere(Vector3(0.6f, 0.3f, 0.2f), 0.9f, 220, 360);
    const CsgFaceBvh treeA(a->GetFaceMin(), a->GetFaceMax());
    const CsgFaceBvh treeB(b->GetFaceMin(), b->GetFaceMax());
    for (const CsgParallelMode mode : {CsgParallelMode::Serial, CsgParallelMode::Parallel})
    {
        const auto start = std::chrono::steady_clock::now();
        size_t count = 0;
        for (int i = 0; i < 20; ++i)
            count += Collect(treeA, treeB, Options(queue, mode)).size();
        const auto end = std::chrono::steady_clock::now();
        std::printf("mode %d: %.2f ms, pairs %zu\n", static_cast<int>(mode),
            std::chrono::duration<double, std::milli>(end - start).count() / 20.0, count / 20);
    }
}
