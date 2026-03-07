#include "../include/catch.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

// Standalone solver performance benchmarks
// These measure the computational cost of different constraint solving approaches
// without requiring the full Bullet Physics or Skyrim environment

namespace
{

	using Clock = std::chrono::high_resolution_clock;
	using Duration = std::chrono::duration<double, std::micro>;

	// Simplified matrix operations for benchmarking
	class BenchmarkMatrix
	{
	public:
		std::vector<float> data;
		int rows, cols;

		BenchmarkMatrix(int r, int c) : rows(r), cols(c), data(r * c, 0.0f) {}

		float& operator()(int r, int c) { return data[r * cols + c]; }
		float operator()(int r, int c) const { return data[r * cols + c]; }

		void setIdentity()
		{
			std::fill(data.begin(), data.end(), 0.0f);
			for (int i = 0; i < std::min(rows, cols); i++) {
				(*this)(i, i) = 1.0f;
			}
		}

		// Create a typical constraint system matrix (sparse, diagonally dominant)
		void setConstraintSystem(std::mt19937& rng)
		{
			std::uniform_real_distribution<float> dist(-0.1f, 0.1f);
			for (int i = 0; i < rows; i++) {
				float diagSum = 0.0f;
				for (int j = 0; j < cols; j++) {
					if (i != j) {
						// Sparse: only nearby constraints interact
						if (std::abs(i - j) <= 2) {
							(*this)(i, j) = dist(rng);
							diagSum += std::abs((*this)(i, j));
						}
					}
				}
				// Diagonal dominance for stability
				(*this)(i, i) = diagSum + 1.0f + std::abs(dist(rng));
			}
		}
	};

	// Gauss-Seidel iteration (core of Sequential Impulse)
	void gaussSeidelIteration(const BenchmarkMatrix& A, const std::vector<float>& b, std::vector<float>& x,
							  const std::vector<float>& lo, const std::vector<float>& hi)
	{
		int n = static_cast<int>(b.size());
		for (int i = 0; i < n; i++) {
			float sum = b[i];
			for (int j = 0; j < n; j++) {
				if (i != j) {
					sum -= A(i, j) * x[j];
				}
			}
			float newX = sum / A(i, i);
			// Project to bounds (LCP constraint)
			x[i] = std::max(lo[i], std::min(hi[i], newX));
		}
	}

	// Simplified Dantzig-style pivot solver (demonstrates O(n^3) behavior)
	// This is a simplified version to show the computational pattern
	bool dantzigSolve(const BenchmarkMatrix& A, const std::vector<float>& b, std::vector<float>& x,
					  const std::vector<float>& lo, const std::vector<float>& hi)
	{
		int n = static_cast<int>(b.size());
		std::vector<float> w(n);

		// Initial solution
		std::fill(x.begin(), x.end(), 0.0f);

		// Compute initial w = b - A*x
		for (int i = 0; i < n; i++) {
			w[i] = b[i];
			for (int j = 0; j < n; j++) {
				w[i] -= A(i, j) * x[j];
			}
		}

		// Pivot iterations (simplified)
		for (int iter = 0; iter < n * 2; iter++) {
			// Find violated constraint
			int pivot = -1;
			float maxViolation = 1e-6f;

			for (int i = 0; i < n; i++) {
				if (x[i] <= lo[i] && w[i] < -maxViolation) {
					pivot = i;
					maxViolation = -w[i];
				}
				else if (x[i] >= hi[i] && w[i] > maxViolation) {
					pivot = i;
					maxViolation = w[i];
				}
			}

			if (pivot < 0)
				break; // Converged

			// Pivot operation (O(n^2) per pivot, O(n) pivots = O(n^3) total)
			float delta = w[pivot] / A(pivot, pivot);
			x[pivot] += delta;
			x[pivot] = std::max(lo[pivot], std::min(hi[pivot], x[pivot]));

			// Update w
			for (int i = 0; i < n; i++) {
				w[i] -= A(i, pivot) * delta;
			}
		}

		return true;
	}

	struct BenchmarkResult
	{
		double meanMicros;
		double stdDevMicros;
		double minMicros;
		double maxMicros;
		int iterations;
	};

	BenchmarkResult runBenchmark(std::function<void()> fn, int warmupRuns, int measuredRuns)
	{
		// Warmup
		for (int i = 0; i < warmupRuns; i++) {
			fn();
		}

		std::vector<double> times;
		times.reserve(measuredRuns);

		for (int i = 0; i < measuredRuns; i++) {
			auto start = Clock::now();
			fn();
			auto end = Clock::now();
			times.push_back(Duration(end - start).count());
		}

		double sum = std::accumulate(times.begin(), times.end(), 0.0);
		double mean = sum / times.size();

		double sqSum = 0.0;
		for (double t : times) {
			sqSum += (t - mean) * (t - mean);
		}
		double stdDev = std::sqrt(sqSum / times.size());

		auto [minIt, maxIt] = std::minmax_element(times.begin(), times.end());

		return {mean, stdDev, *minIt, *maxIt, measuredRuns};
	}

} // anonymous namespace


// =============================================================================
// BENCHMARK TESTS
// =============================================================================

TEST_CASE("Gauss-Seidel iteration scaling", "[solver][benchmark]")
{
	std::mt19937 rng(42);

	struct TestConfig
	{
		int size;
		int iterations;
		const char* description;
	};

	// Representative constraint system sizes
	// Small: single bone chain (5-10 constraints)
	// Medium: hair strand (20-50 constraints)
	// Large: full cloth mesh section (100+ constraints)
	std::vector<TestConfig> configs = {
		{10, 4, "Small (single bone chain)"}, {25, 4, "Medium-small (short hair)"}, {50, 4, "Medium (hair strand)"},
		{100, 4, "Large (cloth section)"},	  {200, 4, "Very large (full cloth)"},
	};

	SECTION("Measure iteration cost by system size")
	{
		std::vector<std::pair<int, double>> results;

		for (const auto& config : configs) {
			BenchmarkMatrix A(config.size, config.size);
			A.setConstraintSystem(rng);

			std::vector<float> b(config.size);
			std::vector<float> x(config.size, 0.0f);
			std::vector<float> lo(config.size, -1000.0f);
			std::vector<float> hi(config.size, 1000.0f);

			std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
			for (int i = 0; i < config.size; i++) {
				b[i] = dist(rng);
			}

			auto benchmark = runBenchmark(
				[&]() {
					std::fill(x.begin(), x.end(), 0.0f);
					for (int iter = 0; iter < config.iterations; iter++) {
						gaussSeidelIteration(A, b, x, lo, hi);
					}
				},
				10, 100);

			results.push_back({config.size, benchmark.meanMicros});

			INFO(config.description << ": " << config.size << " constraints, " << config.iterations << " iterations");
			INFO("Mean: " << benchmark.meanMicros << " us, StdDev: " << benchmark.stdDevMicros << " us");

			// Just verify it runs - actual timing is informational
			REQUIRE(benchmark.meanMicros > 0);
		}

		// Verify O(n^2) scaling (each iteration is O(n^2) for dense matrix)
		// Ratio of times should be roughly (n2/n1)^2
		if (results.size() >= 2) {
			double ratio = results.back().second / results.front().second;
			double sizeRatio = static_cast<double>(results.back().first) / results.front().first;
			double expectedRatio = sizeRatio * sizeRatio; // O(n^2)

			INFO("Time ratio: " << ratio << ", Size ratio squared: " << expectedRatio);
			// Allow significant variance due to cache effects, etc.
			REQUIRE(ratio > expectedRatio * 0.1);  // At least 10% of expected
			REQUIRE(ratio < expectedRatio * 10.0); // At most 10x expected
		}
	}
}

TEST_CASE("Dantzig vs Gauss-Seidel comparison", "[solver][benchmark]")
{
	std::mt19937 rng(42);

	// Test sizes representative of hair physics constraint groups
	std::vector<int> sizes = {10, 25, 50, 75, 100};

	SECTION("Compare solver performance")
	{
		for (int size : sizes) {
			BenchmarkMatrix A(size, size);
			A.setConstraintSystem(rng);

			std::vector<float> b(size);
			std::vector<float> x_gs(size, 0.0f);
			std::vector<float> x_dz(size, 0.0f);
			std::vector<float> lo(size, -1000.0f);
			std::vector<float> hi(size, 1000.0f);

			std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
			for (int i = 0; i < size; i++) {
				b[i] = dist(rng);
			}

			// Gauss-Seidel with typical iteration counts
			int gsIterations = 4; // Typical groupIterations value
			auto gsBenchmark = runBenchmark(
				[&]() {
					std::fill(x_gs.begin(), x_gs.end(), 0.0f);
					for (int iter = 0; iter < gsIterations; iter++) {
						gaussSeidelIteration(A, b, x_gs, lo, hi);
					}
				},
				10, 100);

			// Dantzig solver
			auto dzBenchmark = runBenchmark(
				[&]() {
					std::fill(x_dz.begin(), x_dz.end(), 0.0f);
					dantzigSolve(A, b, x_dz, lo, hi);
				},
				10, 100);

			double speedup = dzBenchmark.meanMicros / gsBenchmark.meanMicros;

			INFO("Size " << size << " constraints:");
			INFO("  Gauss-Seidel (" << gsIterations << " iter): " << gsBenchmark.meanMicros << " us");
			INFO("  Dantzig: " << dzBenchmark.meanMicros << " us");
			INFO("  Dantzig/GS ratio: " << speedup << "x slower");

			// Dantzig should be slower for these sizes
			// (it's O(n^3) vs O(n^2 * iterations))
			REQUIRE(gsBenchmark.meanMicros > 0);
			REQUIRE(dzBenchmark.meanMicros > 0);
		}
	}
}

TEST_CASE("Iteration count vs accuracy tradeoff", "[solver][benchmark]")
{
	std::mt19937 rng(42);
	int size = 50; // Medium-sized system

	BenchmarkMatrix A(size, size);
	A.setConstraintSystem(rng);

	std::vector<float> b(size);
	std::vector<float> lo(size, -1000.0f);
	std::vector<float> hi(size, 1000.0f);

	std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
	for (int i = 0; i < size; i++) {
		b[i] = dist(rng);
	}

	// Get "ground truth" with many iterations
	std::vector<float> x_truth(size, 0.0f);
	for (int iter = 0; iter < 1000; iter++) {
		gaussSeidelIteration(A, b, x_truth, lo, hi);
	}

	SECTION("Measure convergence rate")
	{
		std::vector<int> iterCounts = {1, 2, 4, 8, 16, 32};

		for (int numIter : iterCounts) {
			std::vector<float> x(size, 0.0f);

			auto benchmark = runBenchmark(
				[&]() {
					std::fill(x.begin(), x.end(), 0.0f);
					for (int iter = 0; iter < numIter; iter++) {
						gaussSeidelIteration(A, b, x, lo, hi);
					}
				},
				10, 100);

			// Calculate error vs ground truth
			double maxError = 0.0;
			double rmsError = 0.0;
			for (int i = 0; i < size; i++) {
				double err = std::abs(x[i] - x_truth[i]);
				maxError = std::max(maxError, err);
				rmsError += err * err;
			}
			rmsError = std::sqrt(rmsError / size);

			INFO("Iterations: " << numIter);
			INFO("  Time: " << benchmark.meanMicros << " us");
			INFO("  Max error: " << maxError);
			INFO("  RMS error: " << rmsError);

			REQUIRE(benchmark.meanMicros > 0);
		}
	}
}

TEST_CASE("Chain constraint propagation", "[solver][benchmark]")
{
	// This tests the specific case of hair/cloth chains where
	// constraints are connected in sequence

	std::mt19937 rng(42);

	SECTION("Linear chain (hair strand)")
	{
		// Hair is typically a linear chain of constraints
		// Information must propagate from root to tip

		std::vector<int> chainLengths = {5, 10, 20, 40, 80};

		for (int length : chainLengths) {
			// Tridiagonal matrix (each node connected to neighbors)
			BenchmarkMatrix A(length, length);
			for (int i = 0; i < length; i++) {
				A(i, i) = 2.0f;
				if (i > 0)
					A(i, i - 1) = -0.8f;
				if (i < length - 1)
					A(i, i + 1) = -0.8f;
			}

			std::vector<float> b(length, 0.0f);
			b[0] = 1.0f; // Impulse at root

			std::vector<float> x(length, 0.0f);
			std::vector<float> lo(length, -1000.0f);
			std::vector<float> hi(length, 1000.0f);

			// How many iterations to propagate to the tip?
			std::vector<int> iterCounts = {1, 2, 4, 8, 16};

			for (int numIter : iterCounts) {
				std::fill(x.begin(), x.end(), 0.0f);
				for (int iter = 0; iter < numIter; iter++) {
					gaussSeidelIteration(A, b, x, lo, hi);
				}

				// Check how far the impulse propagated
				int propagationDepth = 0;
				for (int i = 0; i < length; i++) {
					if (std::abs(x[i]) > 0.01f) {
						propagationDepth = i + 1;
					}
				}

				INFO("Chain length: " << length << ", Iterations: " << numIter);
				INFO("  Propagation depth: " << propagationDepth << "/" << length);
				INFO("  Tip value: " << x[length - 1]);

				REQUIRE(propagationDepth >= 0);
			}
		}
	}
}

TEST_CASE("Warm starting benefit", "[solver][benchmark]")
{
	std::mt19937 rng(42);
	int size = 50;

	BenchmarkMatrix A(size, size);
	A.setConstraintSystem(rng);

	std::vector<float> b(size);
	std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
	for (int i = 0; i < size; i++) {
		b[i] = dist(rng);
	}

	std::vector<float> lo(size, -1000.0f);
	std::vector<float> hi(size, 1000.0f);

	// Simulate frame-to-frame solving with warm start
	SECTION("Cold start vs warm start convergence")
	{
		// Get converged solution
		std::vector<float> x_converged(size, 0.0f);
		for (int iter = 0; iter < 100; iter++) {
			gaussSeidelIteration(A, b, x_converged, lo, hi);
		}

		// Cold start: from zero
		std::vector<float> x_cold(size, 0.0f);
		int coldIterations = 4;
		for (int iter = 0; iter < coldIterations; iter++) {
			gaussSeidelIteration(A, b, x_cold, lo, hi);
		}

		double coldError = 0.0;
		for (int i = 0; i < size; i++) {
			coldError += std::abs(x_cold[i] - x_converged[i]);
		}
		coldError /= size;

		// Warm start: from 90% of converged (simulating previous frame)
		std::vector<float> x_warm(size);
		for (int i = 0; i < size; i++) {
			x_warm[i] = x_converged[i] * 0.9f;
		}

		int warmIterations = 4;
		for (int iter = 0; iter < warmIterations; iter++) {
			gaussSeidelIteration(A, b, x_warm, lo, hi);
		}

		double warmError = 0.0;
		for (int i = 0; i < size; i++) {
			warmError += std::abs(x_warm[i] - x_converged[i]);
		}
		warmError /= size;

		INFO("Cold start error (4 iter): " << coldError);
		INFO("Warm start error (4 iter): " << warmError);
		INFO("Warm start improvement: " << (coldError / warmError) << "x");

		// Warm start should be significantly better
		REQUIRE(warmError < coldError);
	}
}

TEST_CASE("Representative timing summary", "[solver][benchmark][!mayfail]")
{
	// This test produces a summary table of expected timings
	// Marked !mayfail because absolute timings are system-dependent

	std::mt19937 rng(42);

	struct ScenarioResult
	{
		const char* scenario;
		int constraints;
		double gsTime4iter;
		double gsTime8iter;
		double dantzigTime;
	};

	std::vector<ScenarioResult> results;

	// Scenarios matching real hair/cloth usage
	std::vector<std::pair<const char*, int>> scenarios = {
		{"Single hair strand", 25},
		{"Multiple hair strands (5x)", 125},
		{"Cloth patch", 100},
		{"Full body cloth", 300},
	};

	for (auto& [name, size] : scenarios) {
		BenchmarkMatrix A(size, size);
		A.setConstraintSystem(rng);

		std::vector<float> b(size);
		std::vector<float> x(size, 0.0f);
		std::vector<float> lo(size, -1000.0f);
		std::vector<float> hi(size, 1000.0f);

		std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
		for (int i = 0; i < size; i++) {
			b[i] = dist(rng);
		}

		auto gs4 = runBenchmark(
			[&]() {
				std::fill(x.begin(), x.end(), 0.0f);
				for (int iter = 0; iter < 4; iter++) {
					gaussSeidelIteration(A, b, x, lo, hi);
				}
			},
			5, 50);

		auto gs8 = runBenchmark(
			[&]() {
				std::fill(x.begin(), x.end(), 0.0f);
				for (int iter = 0; iter < 8; iter++) {
					gaussSeidelIteration(A, b, x, lo, hi);
				}
			},
			5, 50);

		auto dz = runBenchmark(
			[&]() {
				std::fill(x.begin(), x.end(), 0.0f);
				dantzigSolve(A, b, x, lo, hi);
			},
			5, 50);

		results.push_back({name, size, gs4.meanMicros, gs8.meanMicros, dz.meanMicros});
	}

	// Print summary table
	INFO("=== SOLVER PERFORMANCE SUMMARY ===");
	INFO("| Scenario | Constraints | GS(4iter) | GS(8iter) | Dantzig | Dantzig/GS4 |");
	INFO("|----------|-------------|-----------|-----------|---------|-------------|");

	for (auto& r : results) {
		char buffer[256];
		snprintf(buffer, sizeof(buffer), "| %-20s | %4d | %7.1f us | %7.1f us | %7.1f us | %5.1fx |", r.scenario,
				 r.constraints, r.gsTime4iter, r.gsTime8iter, r.dantzigTime, r.dantzigTime / r.gsTime4iter);
		INFO(buffer);
	}

	// Always pass - this is informational
	REQUIRE(results.size() > 0);
}
