/**
 * Real-world Solver Configuration Benchmarks
 *
 * Tests actual performance tradeoffs between different solver configurations:
 * - numIterations: global solver iterations
 * - groupIterations: constraint group iterations
 * - groupEnableMLCP: Dantzig vs Sequential Impulse
 *
 * Uses synthetic physics simulations that model Bullet's behavior patterns
 * without requiring the actual Bullet library.
 *
 * These benchmarks simulate:
 * - Hair chain (linear constraint sequence with poor propagation)
 * - Cloth patch (2D grid of interconnected constraints)
 * - Mixed scenarios (multiple NPCs with different physics systems)
 */

#include "../include/catch.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

namespace
{

	using Clock = std::chrono::high_resolution_clock;
	using Duration = std::chrono::duration<double, std::micro>;

	// Configuration preset matching configs.xml options
	struct SolverConfig
	{
		const char* name;
		int numIterations;	 // btContactSolverInfo::m_numIterations
		int groupIterations; // How many times to run the solver per group
		bool enableMLCP;	 // Use MLCP solver vs SI

		// Derived effective iterations
		int effectiveIterations() const { return numIterations * groupIterations; }
	};

	// Standard presets matching configs.xml options
	const std::vector<SolverConfig> PRESETS = {
		{"Minimum", 4, 1, false},	{"Low", 6, 2, false},	  {"Default", 10, 2, true},
		{"Balanced", 10, 4, false}, {"Quality", 16, 4, true}, {"Maximum", 16, 8, true},
	};

	struct BenchmarkResult
	{
		double solveTimeUs;		// Microseconds per frame
		double constraintError; // Average constraint violation
		double energyDrift;		// Energy stability (0 = perfect)
		double convergenceRate; // How fast error decreases
	};

	// ============================================================================
	// CONSTRAINT SYSTEM SIMULATION
	// ============================================================================

	// Simulates a single constraint between two bodies
	struct Constraint
	{
		int bodyA, bodyB; // Indices into body array
		float restLength; // Target distance
		float compliance; // Inverse stiffness (XPBD-style)
	};

	// Simulates a rigid body (simplified)
	struct Body
	{
		float x, y, z;	  // Position
		float vx, vy, vz; // Velocity
		float invMass;	  // 0 = static

		void applyImpulse(float ix, float iy, float iz)
		{
			if (invMass > 0) {
				vx += ix * invMass;
				vy += iy * invMass;
				vz += iz * invMass;
			}
		}

		void integrate(float dt)
		{
			if (invMass > 0) {
				x += vx * dt;
				y += vy * dt;
				z += vz * dt;
				// Gravity
				vy -= 9.81f * dt;
			}
		}
	};

	class ConstraintSystem
	{
	public:
		std::vector<Body> bodies;
		std::vector<Constraint> constraints;

		// Create a hair-like chain
		void createChain(int numBones, float boneLength = 0.1f)
		{
			bodies.clear();
			constraints.clear();

			for (int i = 0; i < numBones; i++) {
				Body b;
				b.x = 0;
				b.y = 1.0f - i * boneLength;
				b.z = 0;
				b.vx = b.vy = b.vz = 0;
				b.invMass = (i == 0) ? 0.0f : 100.0f; // First is static
				bodies.push_back(b);

				if (i > 0) {
					Constraint c;
					c.bodyA = i - 1;
					c.bodyB = i;
					c.restLength = boneLength;
					c.compliance = 0.0001f;
					constraints.push_back(c);
				}
			}
		}

		// Create a cloth-like grid
		void createGrid(int size, float spacing = 0.1f)
		{
			bodies.clear();
			constraints.clear();

			for (int i = 0; i < size; i++) {
				for (int j = 0; j < size; j++) {
					Body b;
					b.x = j * spacing;
					b.y = 1.0f - i * spacing;
					b.z = 0;
					b.vx = b.vy = b.vz = 0;
					b.invMass = (i == 0) ? 0.0f : 100.0f; // Top row static
					bodies.push_back(b);
				}
			}

			// Horizontal constraints
			for (int i = 0; i < size; i++) {
				for (int j = 0; j < size - 1; j++) {
					Constraint c;
					c.bodyA = i * size + j;
					c.bodyB = i * size + j + 1;
					c.restLength = spacing;
					c.compliance = 0.0001f;
					constraints.push_back(c);
				}
			}

			// Vertical constraints
			for (int i = 0; i < size - 1; i++) {
				for (int j = 0; j < size; j++) {
					Constraint c;
					c.bodyA = i * size + j;
					c.bodyB = (i + 1) * size + j;
					c.restLength = spacing;
					c.compliance = 0.0001f;
					constraints.push_back(c);
				}
			}
		}

		// Gauss-Seidel solver (Sequential Impulse style)
		void solveGaussSeidel(int iterations)
		{
			for (int iter = 0; iter < iterations; iter++) {
				for (auto& c : constraints) {
					Body& a = bodies[c.bodyA];
					Body& b = bodies[c.bodyB];

					float dx = b.x - a.x;
					float dy = b.y - a.y;
					float dz = b.z - a.z;
					float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

					if (dist < 0.0001f)
						continue;

					float error = dist - c.restLength;
					float totalMass = a.invMass + b.invMass;
					if (totalMass < 0.0001f)
						continue;

					float correction = error / totalMass;
					float nx = dx / dist;
					float ny = dy / dist;
					float nz = dz / dist;

					// Position correction
					a.x += nx * correction * a.invMass;
					a.y += ny * correction * a.invMass;
					a.z += nz * correction * a.invMass;
					b.x -= nx * correction * b.invMass;
					b.y -= ny * correction * b.invMass;
					b.z -= nz * correction * b.invMass;
				}
			}
		}

		// MLCP-style solver (direct, more expensive but exact)
		void solveMLCP(int iterations)
		{
			// Simulate MLCP's higher per-iteration cost but better convergence
			// MLCP is typically O(n^3) for building LCP, vs O(n) for GS iteration

			// First pass: Build approximate Jacobian (expensive)
			std::vector<float> jacobian(constraints.size() * 6, 0.0f);
			for (size_t i = 0; i < constraints.size(); i++) {
				auto& c = constraints[i];
				Body& a = bodies[c.bodyA];
				Body& b = bodies[c.bodyB];

				float dx = b.x - a.x;
				float dy = b.y - a.y;
				float dz = b.z - a.z;
				float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
				if (dist > 0.0001f) {
					jacobian[i * 6 + 0] = -dx / dist;
					jacobian[i * 6 + 1] = -dy / dist;
					jacobian[i * 6 + 2] = -dz / dist;
					jacobian[i * 6 + 3] = dx / dist;
					jacobian[i * 6 + 4] = dy / dist;
					jacobian[i * 6 + 5] = dz / dist;
				}
			}

			// Simulate Dantzig's O(n^2) per-iteration cost with better accuracy
			// In practice, MLCP converges faster but each iteration is more expensive
			for (int iter = 0; iter < iterations; iter++) {
				// Additional matrix operations to simulate MLCP overhead
				float dummy = 0.0f;
				for (size_t i = 0; i < jacobian.size(); i++) {
					dummy += jacobian[i] * jacobian[i];
				}
				(void)dummy; // Prevent optimization

				// Still do position correction
				for (auto& c : constraints) {
					Body& a = bodies[c.bodyA];
					Body& b = bodies[c.bodyB];

					float dx = b.x - a.x;
					float dy = b.y - a.y;
					float dz = b.z - a.z;
					float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

					if (dist < 0.0001f)
						continue;

					// MLCP applies stronger correction per iteration
					float error = dist - c.restLength;
					float totalMass = a.invMass + b.invMass;
					if (totalMass < 0.0001f)
						continue;

					float correction = error / totalMass * 1.2f; // Over-relaxation
					float nx = dx / dist;
					float ny = dy / dist;
					float nz = dz / dist;

					a.x += nx * correction * a.invMass;
					a.y += ny * correction * a.invMass;
					a.z += nz * correction * a.invMass;
					b.x -= nx * correction * b.invMass;
					b.y -= ny * correction * b.invMass;
					b.z -= nz * correction * b.invMass;
				}
			}
		}

		void integrate(float dt)
		{
			for (auto& b : bodies) {
				b.integrate(dt);
			}
		}

		double measureError() const
		{
			double totalError = 0.0;
			for (const auto& c : constraints) {
				const Body& a = bodies[c.bodyA];
				const Body& b = bodies[c.bodyB];

				float dx = b.x - a.x;
				float dy = b.y - a.y;
				float dz = b.z - a.z;
				float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

				totalError += std::abs(dist - c.restLength);
			}
			return constraints.empty() ? 0.0 : totalError / constraints.size();
		}

		double measureEnergy() const
		{
			double energy = 0.0;
			for (const auto& b : bodies) {
				if (b.invMass > 0) {
					float mass = 1.0f / b.invMass;
					energy += 0.5 * mass * (b.vx * b.vx + b.vy * b.vy + b.vz * b.vz);
					energy += mass * 9.81 * b.y;
				}
			}
			return energy;
		}
	};

	// Run a simulation scenario and measure performance
	BenchmarkResult runScenario(ConstraintSystem& system, const SolverConfig& config, int warmupFrames,
								int measureFrames, float dt = 1.0f / 60.0f)
	{
		double initialEnergy = system.measureEnergy();

		// Warmup
		for (int i = 0; i < warmupFrames; i++) {
			system.integrate(dt);
			for (int g = 0; g < config.groupIterations; g++) {
				if (config.enableMLCP) {
					system.solveMLCP(config.numIterations);
				}
				else {
					system.solveGaussSeidel(config.numIterations);
				}
			}
		}

		// Measure
		double totalTime = 0.0;
		double totalError = 0.0;

		for (int i = 0; i < measureFrames; i++) {
			auto start = Clock::now();

			system.integrate(dt);
			for (int g = 0; g < config.groupIterations; g++) {
				if (config.enableMLCP) {
					system.solveMLCP(config.numIterations);
				}
				else {
					system.solveGaussSeidel(config.numIterations);
				}
			}

			auto end = Clock::now();
			totalTime += Duration(end - start).count();
			totalError += system.measureError();
		}

		double finalEnergy = system.measureEnergy();

		return {
			totalTime / measureFrames, totalError / measureFrames,
			std::abs(finalEnergy - initialEnergy) / std::max(1.0, std::abs(initialEnergy)),
			0.0 // Convergence rate not measured
		};
	}

} // anonymous namespace


// =============================================================================
// REAL-WORLD BENCHMARK TESTS
// =============================================================================

TEST_CASE("Hair chain config comparison", "[bullet][benchmark][config]")
{
	const int WARMUP = 30;
	const int MEASURE = 100;

	std::vector<int> chainLengths = {10, 25, 50};

	SECTION("Compare presets across chain lengths")
	{
		for (int length : chainLengths) {
			INFO("=== Hair Chain: " << length << " bones ===");

			for (const auto& config : PRESETS) {
				ConstraintSystem system;
				system.createChain(length);

				auto result = runScenario(system, config, WARMUP, MEASURE);

				INFO(config.name << " (eff=" << config.effectiveIterations() << "): " << result.solveTimeUs
								 << " us/frame, " << "error=" << result.constraintError);

				REQUIRE(result.solveTimeUs > 0);
			}
		}
	}
}

TEST_CASE("Cloth patch config comparison", "[bullet][benchmark][config]")
{
	const int WARMUP = 30;
	const int MEASURE = 100;

	std::vector<int> gridSizes = {5, 8, 10};

	SECTION("Compare presets across grid sizes")
	{
		for (int size : gridSizes) {
			int numConstraints = 2 * size * (size - 1);
			INFO("=== Cloth Patch: " << size << "x" << size << " (" << numConstraints << " constraints) ===");

			for (const auto& config : PRESETS) {
				ConstraintSystem system;
				system.createGrid(size);

				auto result = runScenario(system, config, WARMUP, MEASURE);

				INFO(config.name << ": " << result.solveTimeUs << " us/frame, " << "error=" << result.constraintError);

				REQUIRE(result.solveTimeUs > 0);
			}
		}
	}
}

TEST_CASE("Iteration count scaling", "[bullet][benchmark][config]")
{
	const int WARMUP = 30;
	const int MEASURE = 100;

	std::vector<int> iterCounts = {2, 4, 6, 8, 10, 16, 32};

	SECTION("Hair chain - 25 bones")
	{
		INFO("=== Iteration Count Scaling on 25-bone Hair ===");
		INFO("| Iters | Time (us) | Error    | Time/Iter |");
		INFO("|-------|-----------|----------|-----------|");

		for (int iters : iterCounts) {
			SolverConfig config = {"Custom", iters, 1, false};

			ConstraintSystem system;
			system.createChain(25);
			auto result = runScenario(system, config, WARMUP, MEASURE);

			char buffer[128];
			snprintf(buffer, sizeof(buffer), "| %5d | %9.2f | %8.6f | %9.3f |", iters, result.solveTimeUs,
					 result.constraintError, result.solveTimeUs / iters);
			INFO(buffer);

			REQUIRE(result.solveTimeUs > 0);
		}
	}

	SECTION("Cloth patch - 8x8")
	{
		INFO("=== Iteration Count Scaling on 8x8 Cloth ===");
		INFO("| Iters | Time (us) | Error    | Time/Iter |");
		INFO("|-------|-----------|----------|-----------|");

		for (int iters : iterCounts) {
			SolverConfig config = {"Custom", iters, 1, false};

			ConstraintSystem system;
			system.createGrid(8);
			auto result = runScenario(system, config, WARMUP, MEASURE);

			char buffer[128];
			snprintf(buffer, sizeof(buffer), "| %5d | %9.2f | %8.6f | %9.3f |", iters, result.solveTimeUs,
					 result.constraintError, result.solveTimeUs / iters);
			INFO(buffer);

			REQUIRE(result.solveTimeUs > 0);
		}
	}
}

TEST_CASE("MLCP vs Sequential Impulse comparison", "[bullet][benchmark][config]")
{
	const int WARMUP = 30;
	const int MEASURE = 100;

	SECTION("Direct comparison - same effective iterations")
	{
		std::vector<int> iterCounts = {4, 8, 16};

		for (int iters : iterCounts) {
			INFO("=== " << iters << " iterations: MLCP vs SI ===");

			SolverConfig siConfig = {"SI", iters, 1, false};
			SolverConfig mlcpConfig = {"MLCP", iters, 1, true};

			// Hair chain
			{
				ConstraintSystem siSystem, mlcpSystem;
				siSystem.createChain(25);
				mlcpSystem.createChain(25);

				auto siResult = runScenario(siSystem, siConfig, WARMUP, MEASURE);
				auto mlcpResult = runScenario(mlcpSystem, mlcpConfig, WARMUP, MEASURE);

				INFO("Hair (25 bones):");
				INFO("  SI:   " << siResult.solveTimeUs << " us, error=" << siResult.constraintError);
				INFO("  MLCP: " << mlcpResult.solveTimeUs << " us, error=" << mlcpResult.constraintError);
				INFO("  MLCP/SI time ratio: " << (mlcpResult.solveTimeUs / siResult.solveTimeUs));

				// MLCP should be slower but potentially more accurate
				REQUIRE(siResult.solveTimeUs > 0);
				REQUIRE(mlcpResult.solveTimeUs > 0);
			}

			// Cloth patch
			{
				ConstraintSystem siSystem, mlcpSystem;
				siSystem.createGrid(8);
				mlcpSystem.createGrid(8);

				auto siResult = runScenario(siSystem, siConfig, WARMUP, MEASURE);
				auto mlcpResult = runScenario(mlcpSystem, mlcpConfig, WARMUP, MEASURE);

				INFO("Cloth (8x8):");
				INFO("  SI:   " << siResult.solveTimeUs << " us, error=" << siResult.constraintError);
				INFO("  MLCP: " << mlcpResult.solveTimeUs << " us, error=" << mlcpResult.constraintError);
				INFO("  MLCP/SI time ratio: " << (mlcpResult.solveTimeUs / siResult.solveTimeUs));
			}
		}
	}
}

TEST_CASE("groupIterations effectiveness", "[bullet][benchmark][config]")
{
	const int WARMUP = 30;
	const int MEASURE = 100;

	SECTION("Same effective iterations, different groupIterations")
	{
		// Compare: 16 iterations vs 8 iterations x 2 groups vs 4 iterations x 4 groups
		struct TestCase
		{
			int numIter;
			int groupIter;
		};
		std::vector<TestCase> cases = {
			{16, 1},
			{8, 2},
			{4, 4},
			{2, 8},
		};

		INFO("=== 16 Effective Iterations: Different Distributions ===");
		INFO("All have 16 effective iterations (numIter * groupIter)");
		INFO("");
		INFO("| NumIter | GroupIter | Hair Time | Hair Error | Cloth Time | Cloth Err |");
		INFO("|---------|-----------|-----------|------------|------------|-----------|");

		for (const auto& tc : cases) {
			SolverConfig config = {"Test", tc.numIter, tc.groupIter, false};

			ConstraintSystem hairSystem, clothSystem;
			hairSystem.createChain(25);
			clothSystem.createGrid(8);

			auto hairResult = runScenario(hairSystem, config, WARMUP, MEASURE);
			auto clothResult = runScenario(clothSystem, config, WARMUP, MEASURE);

			char buffer[256];
			snprintf(buffer, sizeof(buffer), "| %7d | %9d | %9.2f | %10.6f | %10.2f | %9.6f |", tc.numIter,
					 tc.groupIter, hairResult.solveTimeUs, hairResult.constraintError, clothResult.solveTimeUs,
					 clothResult.constraintError);
			INFO(buffer);

			REQUIRE(hairResult.solveTimeUs > 0);
		}
	}
}

TEST_CASE("Config tradeoff summary table", "[bullet][benchmark][config][!mayfail]")
{
	const int WARMUP = 20;
	const int MEASURE = 50;

	INFO("==========================================================");
	INFO("CONFIGURATION PERFORMANCE TRADEOFF SUMMARY");
	INFO("==========================================================");
	INFO("");
	INFO("| Preset    | EffIter | Hair us | Cloth us | Hair Err | Cloth Err |");
	INFO("|-----------|---------|---------|----------|----------|-----------|");

	for (const auto& config : PRESETS) {
		ConstraintSystem hairSystem, clothSystem;
		hairSystem.createChain(25);
		clothSystem.createGrid(8);

		auto hairResult = runScenario(hairSystem, config, WARMUP, MEASURE);
		auto clothResult = runScenario(clothSystem, config, WARMUP, MEASURE);

		char buffer[256];
		snprintf(buffer, sizeof(buffer), "| %-9s | %7d | %7.1f | %8.1f | %8.6f | %9.6f |", config.name,
				 config.effectiveIterations(), hairResult.solveTimeUs, clothResult.solveTimeUs,
				 hairResult.constraintError, clothResult.constraintError);
		INFO(buffer);
	}

	INFO("");
	INFO("EffIter = numIterations * groupIterations");
	INFO("Lower time = faster, Lower error = more accurate");
	INFO("==========================================================");

	REQUIRE(true); // Informational test
}

TEST_CASE("Realistic game scenario benchmarks", "[bullet][benchmark][config]")
{
	const int WARMUP = 30;
	const int MEASURE = 100;

	SECTION("5 NPCs with 20-bone hair each")
	{
		INFO("=== 5 NPCs with 20-bone hair (100 total bones, 95 constraints) ===");

		for (const auto& config : PRESETS) {
			// Create 5 separate hair chains
			std::vector<ConstraintSystem> npcs(5);
			for (auto& npc : npcs) {
				npc.createChain(20);
			}

			// Measure combined simulation time
			double totalTime = 0.0;
			double totalError = 0.0;

			for (int frame = 0; frame < MEASURE; frame++) {
				auto start = Clock::now();

				for (auto& npc : npcs) {
					npc.integrate(1.0f / 60.0f);
					for (int g = 0; g < config.groupIterations; g++) {
						if (config.enableMLCP) {
							npc.solveMLCP(config.numIterations);
						}
						else {
							npc.solveGaussSeidel(config.numIterations);
						}
					}
				}

				auto end = Clock::now();
				totalTime += Duration(end - start).count();

				for (const auto& npc : npcs) {
					totalError += npc.measureError();
				}
			}

			double avgTime = totalTime / MEASURE;
			double avgError = totalError / (MEASURE * 5);
			double fps60Budget = 16666.0; // 16.6ms in us
			double percentOfBudget = (avgTime / fps60Budget) * 100;

			INFO(config.name << ": " << avgTime << " us/frame (" << percentOfBudget
							 << "% of 60fps), error=" << avgError);

			REQUIRE(avgTime > 0);
		}
	}

	SECTION("Mixed: 3 NPCs hair + 1 cloth cape")
	{
		INFO("=== 3 NPCs hair + 1 cloth cape ===");

		for (const auto& config : PRESETS) {
			std::vector<ConstraintSystem> npcs(3);
			for (auto& npc : npcs) {
				npc.createChain(15);
			}
			ConstraintSystem cloth;
			cloth.createGrid(6);

			double totalTime = 0.0;

			for (int frame = 0; frame < MEASURE; frame++) {
				auto start = Clock::now();

				for (auto& npc : npcs) {
					npc.integrate(1.0f / 60.0f);
					for (int g = 0; g < config.groupIterations; g++) {
						if (config.enableMLCP) {
							npc.solveMLCP(config.numIterations);
						}
						else {
							npc.solveGaussSeidel(config.numIterations);
						}
					}
				}

				cloth.integrate(1.0f / 60.0f);
				for (int g = 0; g < config.groupIterations; g++) {
					if (config.enableMLCP) {
						cloth.solveMLCP(config.numIterations);
					}
					else {
						cloth.solveGaussSeidel(config.numIterations);
					}
				}

				auto end = Clock::now();
				totalTime += Duration(end - start).count();
			}

			double avgTime = totalTime / MEASURE;
			double fps60Budget = 16666.0;
			double percentOfBudget = (avgTime / fps60Budget) * 100;

			INFO(config.name << ": " << avgTime << " us/frame (" << percentOfBudget << "% of 60fps budget)");

			REQUIRE(avgTime > 0);
		}
	}
}

TEST_CASE("Performance scaling with NPC count", "[bullet][benchmark][config]")
{
	const int WARMUP = 10;
	const int MEASURE = 50;

	std::vector<int> npcCounts = {1, 2, 5, 10, 20};
	SolverConfig defaultConfig = {"Default", 10, 2, false};

	INFO("=== Performance Scaling with NPC Count (Default config) ===");
	INFO("| NPCs | Bones | Constraints | Time (us) | us/NPC  | 60fps % |");
	INFO("|------|-------|-------------|-----------|---------|---------|");

	for (int count : npcCounts) {
		std::vector<ConstraintSystem> npcs(count);
		for (auto& npc : npcs) {
			npc.createChain(20);
		}

		double totalTime = 0.0;
		for (int frame = 0; frame < MEASURE; frame++) {
			auto start = Clock::now();

			for (auto& npc : npcs) {
				npc.integrate(1.0f / 60.0f);
				for (int g = 0; g < defaultConfig.groupIterations; g++) {
					npc.solveGaussSeidel(defaultConfig.numIterations);
				}
			}

			auto end = Clock::now();
			totalTime += Duration(end - start).count();
		}

		double avgTime = totalTime / MEASURE;
		double fps60Budget = 16666.0;

		char buffer[256];
		snprintf(buffer, sizeof(buffer), "| %4d | %5d | %11d | %9.1f | %7.2f | %7.2f |", count, count * 20, count * 19,
				 avgTime, avgTime / count, (avgTime / fps60Budget) * 100);
		INFO(buffer);

		REQUIRE(avgTime > 0);
	}
}
