/**
 * Parallel Frame Start Benchmark
 *
 * Measures whether parallelising internalUpdate + predictUnconstraintMotion
 * is worth the scheduling overhead in the NOCUDA path.
 *
 * Methodology
 * -----------
 * Uses std::async as a PESSIMISTIC proxy for enkiTS hdt_parallel_invoke.
 * std::async overhead: ~50-100us (may create new threads)
 * enkiTS overhead:     ~1-5us   (pre-warmed thread pool)
 *
 * If parallel wins here, it definitely wins with enkiTS.
 * If parallel loses here, use the Tracy in-game trace to decide.
 *
 * Work models
 * -----------
 * predictMotionWork(N)  -- N bodies * (applyDamping + predictIntegratedTransform)
 *                          ~= 6 float muls + 6 adds per body, plus one branch
 * internalUpdateWork(N, V) -- N bodies * V vertices * weighted bone skinning
 *                          ~= 16 float muls + 12 adds per vertex (4 bones, 1 weight each)
 *
 * These are synthetic but calibrated to match the real operation counts.
 * Absolute timings will differ from in-game (cache pressure, bone hierarchy,
 * NiTransform reads) but relative scaling is representative.
 */

#include "../include/catch.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <numeric>
#include <random>
#include <vector>

namespace
{

	using Clock = std::chrono::high_resolution_clock;
	using Duration = std::chrono::duration<double, std::micro>;

	// ---------------------------------------------------------------------------
	// Work simulators
	// ---------------------------------------------------------------------------

	// Simulates predictUnconstraintMotion(nBodies):
	//   for each body: applyDamping (2 muls), predictIntegratedTransform (4 muls+adds)
	//   Plus one branch. Uses volatile to prevent optimizer elision.
	double predictMotionWork(int nBodies, float timeStep)
	{
		// Fake rigid body state (pos, vel, angVel)
		// Laid out as a flat array to be cache-realistic
		struct Body
		{
			float px, py, pz; // position
			float vx, vy, vz; // linear velocity
			float ax, ay, az; // angular velocity
			float linearDamping;
			float angularDamping;
		};

		static thread_local std::vector<Body> bodies;
		if (static_cast<int>(bodies.size()) != nBodies) {
			std::mt19937 rng(42);
			std::uniform_real_distribution<float> dist(-10.f, 10.f);
			bodies.resize(nBodies);
			for (auto& b : bodies) {
				b.px = dist(rng);
				b.py = dist(rng);
				b.pz = dist(rng);
				b.vx = dist(rng);
				b.vy = dist(rng);
				b.vz = dist(rng);
				b.ax = dist(rng);
				b.ay = dist(rng);
				b.az = dist(rng);
				b.linearDamping = 0.02f;
				b.angularDamping = 0.02f;
			}
		}

		double checksum = 0.0; // prevent dead-code elimination
		for (int i = 0; i < nBodies; ++i) {
			auto& b = bodies[i];
			// applyDamping equivalent
			float dampL = std::pow(1.f - b.linearDamping, timeStep);
			float dampA = std::pow(1.f - b.angularDamping, timeStep);
			b.vx *= dampL;
			b.vy *= dampL;
			b.vz *= dampL;
			b.ax *= dampA;
			b.ay *= dampA;
			b.az *= dampA;
			// predictIntegratedTransform equivalent
			b.px += b.vx * timeStep;
			b.py += b.vy * timeStep;
			b.pz += b.vz * timeStep;
			checksum += b.px;
		}
		return checksum;
	}

	// Simulates SkinnedMeshBody::internalUpdate() for nBodies bodies,
	// each with nVerticesPerBody vertices, using 4-bone weighted skinning.
	// This is the dominant cost in the NOCUDA internalUpdate loop.
	double internalUpdateWork(int nBodies, int nVerticesPerBody)
	{
		struct BoneMatrix
		{
			float m[16];
		}; // 4x4 column-major
		struct Vertex
		{
			float x, y, z;
			float w[4];
			int bone[4];
		};

		// Static storage so repeated calls reuse the same allocation (realistic)
		static thread_local std::vector<BoneMatrix> bones;
		static thread_local std::vector<Vertex> vertices;

		int totalVerts = nBodies * nVerticesPerBody;
		int totalBones = nBodies * 4; // ~4 bones per body is typical

		if (static_cast<int>(vertices.size()) != totalVerts) {
			std::mt19937 rng(99);
			std::uniform_real_distribution<float> fDist(-1.f, 1.f);
			bones.resize(totalBones);
			vertices.resize(totalVerts);
			for (auto& b : bones)
				for (float& f : b.m)
					f = fDist(rng);
			for (int i = 0; i < totalVerts; ++i) {
				auto& v = vertices[i];
				v.x = fDist(rng);
				v.y = fDist(rng);
				v.z = fDist(rng);
				v.w[0] = 0.5f;
				v.w[1] = 0.3f;
				v.w[2] = 0.2f;
				v.w[3] = 0.0f;
				int base = (i / nVerticesPerBody) * 4;
				v.bone[0] = base;
				v.bone[1] = base + 1;
				v.bone[2] = base + 2;
				v.bone[3] = base + 3;
			}
		}

		double checksum = 0.0;
		for (auto& v : vertices) {
			float ox = 0.f, oy = 0.f, oz = 0.f;
			for (int b = 0; b < 3; ++b) // 3 active bones (w[3]=0)
			{
				const auto& m = bones[v.bone[b]];
				float w = v.w[b];
				ox += w * (m.m[0] * v.x + m.m[4] * v.y + m.m[8] * v.z + m.m[12]);
				oy += w * (m.m[1] * v.x + m.m[5] * v.y + m.m[9] * v.z + m.m[13]);
				oz += w * (m.m[2] * v.x + m.m[6] * v.y + m.m[10] * v.z + m.m[14]);
			}
			checksum += ox + oy + oz;
		}
		return checksum;
	}

	// ---------------------------------------------------------------------------
	// Benchmark harness
	// ---------------------------------------------------------------------------

	struct Result
	{
		double meanUs;
		double stddevUs;
		double minUs;
		double maxUs;
	};

	Result bench(std::function<void()> fn, int warmup, int runs)
	{
		for (int i = 0; i < warmup; ++i)
			fn();

		std::vector<double> times;
		times.reserve(runs);
		for (int i = 0; i < runs; ++i) {
			auto t0 = Clock::now();
			fn();
			auto t1 = Clock::now();
			times.push_back(Duration(t1 - t0).count());
		}

		double mean = std::accumulate(times.begin(), times.end(), 0.0) / runs;
		double var = 0.0;
		for (double t : times)
			var += (t - mean) * (t - mean);
		auto [mn, mx] = std::minmax_element(times.begin(), times.end());
		return {mean, std::sqrt(var / runs), *mn, *mx};
	}

	// Run workA and workB via std::async (pessimistic overhead proxy for enkiTS)
	double parallelInvoke(std::function<double()> workA, std::function<double()> workB)
	{
		auto fa = std::async(std::launch::async, workA);
		auto fb = std::async(std::launch::async, workB);
		return fa.get() + fb.get(); // synchronisation point
	}

} // namespace


// =============================================================================
// TEST 1: Scheduling overhead characterization
//
// Answers: how much does std::async cost for tasks of duration X?
// This is the baseline — if the task is shorter than this, parallel never wins.
// =============================================================================

TEST_CASE("Parallel scheduling overhead vs task duration", "[parallel][benchmark]")
{
	// Measure overhead using tasks that spin for known durations
	// If sequential time ≈ Tseq and parallel time ≈ max(TA,TB) + overhead,
	// breakeven is when Tseq - max(TA,TB) > overhead.

	SECTION("Overhead characterization across task durations")
	{
		// Duration of each task in microseconds
		std::vector<int> taskDurationsUs = {10, 25, 50, 100, 200, 500, 1000};

		INFO("=== SCHEDULING OVERHEAD (std::async, pessimistic proxy for enkiTS) ===");
		INFO("Note: enkiTS is ~10-50x cheaper than std::async for pre-warmed pool.");
		INFO("");
		INFO("| TaskDur (us) | Sequential | Parallel | Overhead | Worth it? |");
		INFO("|--------------|------------|----------|----------|-----------|");

		for (int durUs : taskDurationsUs) {
			// Make tasks of known duration using busy work
			auto taskA = [durUs]() -> double {
				auto end = Clock::now() + std::chrono::microseconds(durUs);
				volatile double x = 1.0;
				while (Clock::now() < end)
					x *= 1.0000001;
				return x;
			};
			auto taskB = [durUs]() -> double {
				auto end = Clock::now() + std::chrono::microseconds(durUs);
				volatile double x = 2.0;
				while (Clock::now() < end)
					x *= 1.0000001;
				return x;
			};

			auto seqResult = bench(
				[&]() {
					volatile double a = taskA();
					volatile double b = taskB();
					(void)a;
					(void)b;
				},
				3, 20);

			auto parResult = bench(
				[&]() {
					volatile double r = parallelInvoke(taskA, taskB);
					(void)r;
				},
				3, 20);

			double overheadUs = parResult.meanUs - (double)durUs;
			bool worthIt = parResult.meanUs < seqResult.meanUs * 0.85; // >15% savings

			char buf[256];
			snprintf(buf, sizeof(buf), "| %12d | %8.1f us | %6.1f us | %6.1f us | %-9s |", durUs, seqResult.meanUs,
					 parResult.meanUs, overheadUs, worthIt ? "YES" : "no");
			INFO(buf);

			REQUIRE(seqResult.meanUs > 0);
			REQUIRE(parResult.meanUs > 0);
		}
	}
}


// =============================================================================
// TEST 2: NPC-scale work timing (sequential baseline)
//
// Answers: how long does each operation take at realistic NPC counts?
// These are the numbers to compare against the overhead from TEST 1.
// =============================================================================

TEST_CASE("NPC-scale operation timing", "[parallel][benchmark]")
{
	// NPC count scenarios
	// Vertices per body based on typical SMP mesh:
	//   Hair strand:  ~200 verts
	//   Cloth piece:  ~500 verts
	//   Mixed (avg):  ~300 verts
	struct Scenario
	{
		const char* name;
		int npcs;
		int vertsPerBody;
	};
	std::vector<Scenario> scenarios = {
		{"1 NPC  (single player)", 1, 300}, {"5 NPCs (small encounter)", 5, 300}, {"10 NPCs (combat)", 10, 300},
		{"20 NPCs (crowd)", 20, 300},		{"50 NPCs (stress)", 50, 300},		  {"10 NPCs (heavy cloth)", 10, 800},
		{"20 NPCs (heavy cloth)", 20, 800},
	};

	const float timeStep = 1.f / 60.f;

	SECTION("Individual operation costs")
	{
		INFO("=== PER-OPERATION TIMING (sequential, no parallelism overhead) ===");
		INFO("");
		INFO("| Scenario                  | predictMotion | internalUpdate | Total seq |");
		INFO("|---------------------------|---------------|----------------|-----------|");

		for (auto& s : scenarios) {
			auto pmResult = bench(
				[&]() {
					volatile double r = predictMotionWork(s.npcs, timeStep);
					(void)r;
				},
				5, 50);

			auto iuResult = bench(
				[&]() {
					volatile double r = internalUpdateWork(s.npcs, s.vertsPerBody);
					(void)r;
				},
				5, 50);

			double totalSeq = pmResult.meanUs + iuResult.meanUs;

			char buf[256];
			snprintf(buf, sizeof(buf), "| %-25s | %9.1f us | %10.1f us | %7.1f us |", s.name, pmResult.meanUs,
					 iuResult.meanUs, totalSeq);
			INFO(buf);

			REQUIRE(pmResult.meanUs > 0);
			REQUIRE(iuResult.meanUs > 0);
		}
	}
}


// =============================================================================
// TEST 3: Sequential vs parallel comparison at NPC scale
//
// This is the money test.
// Reports actual savings (or costs) of parallelising the two operations.
// =============================================================================

TEST_CASE("Sequential vs parallel frame start", "[parallel][benchmark]")
{
	struct Scenario
	{
		const char* name;
		int npcs;
		int vertsPerBody;
	};
	std::vector<Scenario> scenarios = {
		{"5 NPCs,  300 verts", 5, 300},	 {"10 NPCs, 300 verts", 10, 300}, {"20 NPCs, 300 verts", 20, 300},
		{"10 NPCs, 800 verts", 10, 800}, {"20 NPCs, 800 verts", 20, 800},
	};

	const float timeStep = 1.f / 60.f;

	SECTION("Parallel vs sequential speedup")
	{
		INFO("=== SEQUENTIAL vs PARALLEL (std::async proxy) ===");
		INFO("Parallel overhead here is PESSIMISTIC vs enkiTS (~10-50x lower in prod).");
		INFO("");
		INFO("| Scenario             | Sequential | Parallel | Speedup | Saves    |");
		INFO("|----------------------|------------|----------|---------|----------|");

		for (auto& s : scenarios) {
			auto seqResult = bench(
				[&]() {
					volatile double a = predictMotionWork(s.npcs, timeStep);
					volatile double b = internalUpdateWork(s.npcs, s.vertsPerBody);
					(void)a;
					(void)b;
				},
				5, 30);

			auto parResult = bench(
				[&]() {
					volatile double r = parallelInvoke([&]() { return predictMotionWork(s.npcs, timeStep); },
													   [&]() { return internalUpdateWork(s.npcs, s.vertsPerBody); });
					(void)r;
				},
				5, 30);

			double speedup = seqResult.meanUs / parResult.meanUs;
			double savingsUs = seqResult.meanUs - parResult.meanUs;

			char buf[256];
			snprintf(buf, sizeof(buf), "| %-20s | %8.1f us | %6.1f us | %5.2fx   | %+6.1f us |", s.name,
					 seqResult.meanUs, parResult.meanUs, speedup, savingsUs);
			INFO(buf);

			REQUIRE(seqResult.meanUs > 0);
			REQUIRE(parResult.meanUs > 0);
		}

		INFO("");
		INFO("Interpretation:");
		INFO("  Speedup > 1.0x = parallel wins (even with pessimistic overhead)");
		INFO("  Speedup < 1.0x = parallel loses even here; needs Tracy data to decide");
		INFO("");
		INFO("Next step: compare savingsUs against Tracy internalUpdate zone timing.");
		INFO("  If Tracy shows internalUpdate >> savingsUs, parallel wins in prod.");
		INFO("  If Tracy shows internalUpdate ~= 0, the work is trivial, skip.");
	}
}


// =============================================================================
// TEST 4: Breakeven analysis
//
// Given a fixed scheduling overhead (from TEST 1), what internalUpdate duration
// is needed before parallelism pays off?
// =============================================================================

TEST_CASE("Breakeven analysis for parallel frame start", "[parallel][benchmark]")
{
	SECTION("Compute breakeven threshold")
	{
		// Measure scheduling overhead directly with near-zero work
		auto noopResult = bench(
			[&]() {
				volatile double r = parallelInvoke([]() { return 0.0; }, []() { return 0.0; });
				(void)r;
			},
			10, 100);

		double overheadUs = noopResult.meanUs;

		INFO("=== BREAKEVEN ANALYSIS ===");
		INFO("");

		char buf[256];
		snprintf(buf, sizeof(buf), "std::async scheduling overhead: %.1f us (pessimistic)", overheadUs);
		INFO(buf);

		// enkiTS is typically 10-50x cheaper — estimate best case
		double enkitsOverheadUs = overheadUs / 20.0; // conservative 20x estimate
		snprintf(buf, sizeof(buf), "enkiTS estimated overhead:      %.1f us (estimated, ~1/20 of std::async)",
				 enkitsOverheadUs);
		INFO(buf);

		INFO("");
		INFO("For parallel to be worth it, the SHORTER task must exceed the overhead.");
		INFO("predictMotion is always shorter than internalUpdate.");
		INFO("");

		// Measure predictMotion at various NPC counts
		const float dt = 1.f / 60.f;
		std::vector<int> npcCounts = {1, 5, 10, 20, 50};

		INFO("| NPCs | predictMotion | Beats std::async? | Beats enkiTS? |");
		INFO("|------|---------------|-------------------|----------------|");

		for (int n : npcCounts) {
			auto r = bench(
				[&]() {
					volatile double x = predictMotionWork(n, dt);
					(void)x;
				},
				5, 50);

			bool beatsAsync = r.meanUs > overheadUs;
			bool beatsEnkiTS = r.meanUs > enkitsOverheadUs;

			snprintf(buf, sizeof(buf), "| %4d | %9.1f us | %-17s | %-14s |", n, r.meanUs,
					 beatsAsync ? "YES (parallel wins)" : "no", beatsEnkiTS ? "YES (parallel wins)" : "no");
			INFO(buf);
		}

		INFO("");
		INFO("Action: if predictMotion beats enkiTS overhead at your target NPC count,");
		INFO("        implementing NOCUDA parallel frame start will show measurable gain.");

		REQUIRE(overheadUs > 0);
	}
}
