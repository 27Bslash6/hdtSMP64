#pragma once

// enkiTS-based parallel primitives - drop-in replacements for PPL
// This replaces Microsoft Parallel Patterns Library (PPL) with enkiTS
// to avoid thread pool over-subscription and reduce per-task overhead.
//
// enkiTS: 0.5-1us per task vs PPL: 2-5us per task
// Used in: Doom (2016), Doom Eternal, Rage 2, Saints Row

// Prevent Windows min/max macros from conflicting with std::min/max in enkiTS
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "../../external/enkiTS/src/TaskScheduler.h"
#include "../hdtPrefix.h" // For _MESSAGE logging
#include "../hdtTracy.h"
#include "../LinearMath/btThreads.h" // For btGetCurrentThreadIndex

#include <atomic>
#include <functional>
#include <type_traits>
#include <vector>

namespace hdt
{
	// Global enkiTS scheduler singleton
	class EnkiTSScheduler
	{
	public:
		static EnkiTSScheduler& get()
		{
			static EnkiTSScheduler instance;
			return instance;
		}

		enki::TaskScheduler& scheduler()
		{
			// Defensive check - ensure scheduler is valid before use
			if (!m_initialized) {
				// This should never happen with proper singleton usage
				// If it does, it indicates a serious bug (use-after-destroy or before-init)
				__debugbreak();
			}
			return m_scheduler;
		}

		void shutdown()
		{
			if (m_initialized) {
				m_scheduler.WaitforAllAndShutdown();
				m_initialized = false;
			}
		}

		bool isInitialized() const { return m_initialized; }
		uint32_t getNumThreads() const { return m_scheduler.GetNumTaskThreads(); }

	private:
		EnkiTSScheduler() : m_initialized(false) // Initialize first for safety
		{
			// CRITICAL: Register main thread index BEFORE enkiTS spawns workers.
			// btGetCurrentThreadIndex() assigns indices sequentially - first caller gets 0.
			// If worker threads call it first, main thread won't be index 0, and
			// btSetTaskScheduler() will silently fail (it requires threadId == 0).
			(void)btGetCurrentThreadIndex();

			// Check hardware thread detection BEFORE initializing
			uint32_t hwThreads = enki::GetNumHardwareThreads();

			// Initialize with hardware thread count
			m_scheduler.Initialize();

			// Verify initialization succeeded
			uint32_t taskThreads = m_scheduler.GetNumTaskThreads();

			// Log the actual values we got
			_MESSAGE("[ENKITS-INIT] hwThreads=%u, taskThreads=%u (after Initialize)", hwThreads, taskThreads);

			if (hwThreads == 0 || taskThreads == 0) {
				_ERROR("[ENKITS-INIT] FATAL: Thread initialization failed! hwThreads=%u taskThreads=%u", hwThreads,
					   taskThreads);
				__debugbreak();
			}

			m_initialized = true;
		}

		~EnkiTSScheduler() { shutdown(); }

		EnkiTSScheduler(const EnkiTSScheduler&) = delete;
		EnkiTSScheduler& operator=(const EnkiTSScheduler&) = delete;

		enki::TaskScheduler m_scheduler;
		volatile bool m_initialized; // volatile to prevent optimization removing checks
	};

	// ============================================================================
	// hdt_parallel_for - Drop-in replacement for concurrency::parallel_for
	// ============================================================================
	template<typename Func>
	void hdt_parallel_for(int begin, int end, Func&& func)
	{
		HDT_ZONE_SCOPED_N("hdt_parallel_for");

		if (begin >= end) {
			return;
		}

		const int count = end - begin;

		// For very small ranges, just run sequentially
		if (count <= 1) {
			for (int i = begin; i < end; ++i) {
				func(i);
			}
			return;
		}

		// Use enkiTS TaskSet with lambda wrapper
		struct ParallelForTask : public enki::ITaskSet
		{
			int m_begin;
			std::function<void(int)> m_func;

			ParallelForTask(int begin, int count, std::function<void(int)> func)
				: enki::ITaskSet(static_cast<uint32_t>(count)), m_begin(begin), m_func(std::move(func))
			{
				// Set minimum range to avoid excessive task splitting
				m_MinRange = (std::max)(1u, static_cast<uint32_t>(count) / (enki::GetNumHardwareThreads() * 4));
			}

			void ExecuteRange(enki::TaskSetPartition range, uint32_t /*threadnum*/) override
			{
				for (uint32_t i = range.start; i < range.end; ++i) {
					m_func(m_begin + static_cast<int>(i));
				}
			}
		};

		ParallelForTask task(begin, count, std::forward<Func>(func));
		EnkiTSScheduler::get().scheduler().AddTaskSetToPipe(&task);
		EnkiTSScheduler::get().scheduler().WaitforTask(&task);
	}

	// ============================================================================
	// hdt_parallel_for_each - Drop-in replacement for concurrency::parallel_for_each
	// ============================================================================
	template<typename Iterator, typename Func>
	void hdt_parallel_for_each(Iterator begin, Iterator end, Func&& func)
	{
		HDT_ZONE_SCOPED_N("hdt_parallel_for_each");

		using ValueType = typename std::iterator_traits<Iterator>::value_type;

		// For random access iterators, we can use parallel_for directly
		if constexpr (std::is_same_v<typename std::iterator_traits<Iterator>::iterator_category,
									 std::random_access_iterator_tag>)
		{
			const auto count = std::distance(begin, end);
			if (count <= 0) {
				return;
			}

			// For very small ranges, just run sequentially
			if (count <= 1) {
				for (auto it = begin; it != end; ++it) {
					func(*it);
				}
				return;
			}

			struct ParallelForEachTask : public enki::ITaskSet
			{
				Iterator m_begin;
				std::function<void(ValueType&)> m_func;

				ParallelForEachTask(Iterator begin, size_t count, std::function<void(ValueType&)> func)
					: enki::ITaskSet(static_cast<uint32_t>(count)), m_begin(begin), m_func(std::move(func))
				{
					// Set minimum range to avoid excessive task splitting
					m_MinRange = (std::max)(1u, static_cast<uint32_t>(count) / (enki::GetNumHardwareThreads() * 4));
				}

				void ExecuteRange(enki::TaskSetPartition range, uint32_t /*threadnum*/) override
				{
					for (uint32_t i = range.start; i < range.end; ++i) {
						m_func(*(m_begin + i));
					}
				}
			};

			ParallelForEachTask task(begin, static_cast<size_t>(count), std::forward<Func>(func));
			EnkiTSScheduler::get().scheduler().AddTaskSetToPipe(&task);
			EnkiTSScheduler::get().scheduler().WaitforTask(&task);
		}
		else {
			// For non-random-access iterators, copy values to vector first
			// This handles const_iterators correctly by copying the values
			std::vector<ValueType> items;
			for (auto it = begin; it != end; ++it) {
				items.push_back(*it);
			}

			if (items.empty()) {
				return;
			}

			struct ParallelForEachCopyTask : public enki::ITaskSet
			{
				std::vector<ValueType>* m_items;
				std::function<void(ValueType&)> m_func;

				ParallelForEachCopyTask(std::vector<ValueType>* items, std::function<void(ValueType&)> func)
					: enki::ITaskSet(static_cast<uint32_t>(items->size())), m_items(items), m_func(std::move(func))
				{
					m_MinRange =
						(std::max)(1u, static_cast<uint32_t>(items->size()) / (enki::GetNumHardwareThreads() * 4));
				}

				void ExecuteRange(enki::TaskSetPartition range, uint32_t /*threadnum*/) override
				{
					for (uint32_t i = range.start; i < range.end; ++i) {
						m_func((*m_items)[i]);
					}
				}
			};

			ParallelForEachCopyTask task(&items, std::forward<Func>(func));
			EnkiTSScheduler::get().scheduler().AddTaskSetToPipe(&task);
			EnkiTSScheduler::get().scheduler().WaitforTask(&task);
		}
	}

	// Container-based overload (more convenient for most call sites)
	template<typename Container, typename Func>
	void hdt_parallel_for_each(Container& c, Func&& func)
	{
		hdt_parallel_for_each(std::begin(c), std::end(c), std::forward<Func>(func));
	}

	// ============================================================================
	// hdt_parallel_invoke - Drop-in replacement for concurrency::parallel_invoke
	// Runs multiple functions in parallel and waits for all to complete
	// ============================================================================

	namespace detail
	{
		// Helper to create task for each function
		template<typename Func>
		struct InvokeTask : public enki::ITaskSet
		{
			Func m_func;

			explicit InvokeTask(Func&& func) : enki::ITaskSet(1), m_func(std::forward<Func>(func)) {}

			void ExecuteRange(enki::TaskSetPartition /*range*/, uint32_t /*threadnum*/) override { m_func(); }
		};

		// Base case for recursive template expansion
		inline void add_invoke_tasks(enki::TaskScheduler& /*scheduler*/) {}

		// Recursive helper to add all tasks to scheduler
		template<typename Task, typename... Tasks>
		void add_invoke_tasks(enki::TaskScheduler& scheduler, Task* first, Tasks*... rest)
		{
			scheduler.AddTaskSetToPipe(first);
			add_invoke_tasks(scheduler, rest...);
		}

		// Base case for wait
		inline void wait_invoke_tasks(enki::TaskScheduler& /*scheduler*/) {}

		// Recursive helper to wait for all tasks
		template<typename Task, typename... Tasks>
		void wait_invoke_tasks(enki::TaskScheduler& scheduler, Task* first, Tasks*... rest)
		{
			scheduler.WaitforTask(first);
			wait_invoke_tasks(scheduler, rest...);
		}
	} // namespace detail

	// Two function overload
	template<typename Func1, typename Func2>
	void hdt_parallel_invoke(Func1&& func1, Func2&& func2)
	{
		HDT_ZONE_SCOPED_N("hdt_parallel_invoke");

		detail::InvokeTask<Func1> task1(std::forward<Func1>(func1));
		detail::InvokeTask<Func2> task2(std::forward<Func2>(func2));

		auto& scheduler = EnkiTSScheduler::get().scheduler();
		scheduler.AddTaskSetToPipe(&task1);
		scheduler.AddTaskSetToPipe(&task2);

		scheduler.WaitforTask(&task1);
		scheduler.WaitforTask(&task2);
	}

	// Three function overload
	template<typename Func1, typename Func2, typename Func3>
	void hdt_parallel_invoke(Func1&& func1, Func2&& func2, Func3&& func3)
	{
		HDT_ZONE_SCOPED_N("hdt_parallel_invoke");

		detail::InvokeTask<Func1> task1(std::forward<Func1>(func1));
		detail::InvokeTask<Func2> task2(std::forward<Func2>(func2));
		detail::InvokeTask<Func3> task3(std::forward<Func3>(func3));

		auto& scheduler = EnkiTSScheduler::get().scheduler();
		scheduler.AddTaskSetToPipe(&task1);
		scheduler.AddTaskSetToPipe(&task2);
		scheduler.AddTaskSetToPipe(&task3);

		scheduler.WaitforTask(&task1);
		scheduler.WaitforTask(&task2);
		scheduler.WaitforTask(&task3);
	}

	// Four function overload
	template<typename Func1, typename Func2, typename Func3, typename Func4>
	void hdt_parallel_invoke(Func1&& func1, Func2&& func2, Func3&& func3, Func4&& func4)
	{
		HDT_ZONE_SCOPED_N("hdt_parallel_invoke");

		detail::InvokeTask<Func1> task1(std::forward<Func1>(func1));
		detail::InvokeTask<Func2> task2(std::forward<Func2>(func2));
		detail::InvokeTask<Func3> task3(std::forward<Func3>(func3));
		detail::InvokeTask<Func4> task4(std::forward<Func4>(func4));

		auto& scheduler = EnkiTSScheduler::get().scheduler();
		scheduler.AddTaskSetToPipe(&task1);
		scheduler.AddTaskSetToPipe(&task2);
		scheduler.AddTaskSetToPipe(&task3);
		scheduler.AddTaskSetToPipe(&task4);

		scheduler.WaitforTask(&task1);
		scheduler.WaitforTask(&task2);
		scheduler.WaitforTask(&task3);
		scheduler.WaitforTask(&task4);
	}

	// ============================================================================
	// AsyncTaskGroup - Drop-in replacement for concurrency::task_group
	// Allows launching tasks asynchronously and waiting for them later
	// ============================================================================
	class AsyncTaskGroup
	{
	public:
		AsyncTaskGroup() = default;

		~AsyncTaskGroup()
		{
			// Wait for any pending tasks on destruction
			wait();
		}

		// Non-copyable
		AsyncTaskGroup(const AsyncTaskGroup&) = delete;
		AsyncTaskGroup& operator=(const AsyncTaskGroup&) = delete;

		// Move semantics
		AsyncTaskGroup(AsyncTaskGroup&& other) noexcept : m_pendingTasks(std::move(other.m_pendingTasks)) {}

		AsyncTaskGroup& operator=(AsyncTaskGroup&& other) noexcept
		{
			if (this != &other) {
				wait(); // Wait for our tasks first
				m_pendingTasks = std::move(other.m_pendingTasks);
			}
			return *this;
		}

		// Launch a task asynchronously
		template<typename Func>
		void run(Func&& func)
		{
			HDT_ZONE_SCOPED_N("AsyncTaskGroup::run");

			auto task = std::make_unique<AsyncTask>(std::forward<Func>(func));
			EnkiTSScheduler::get().scheduler().AddTaskSetToPipe(task.get());
			m_pendingTasks.push_back(std::move(task));
		}

		// Wait for all pending tasks to complete
		void wait()
		{
			HDT_ZONE_SCOPED_N("AsyncTaskGroup::wait");

			auto& scheduler = EnkiTSScheduler::get().scheduler();
			for (auto& task : m_pendingTasks) {
				scheduler.WaitforTask(task.get());
			}
			m_pendingTasks.clear();
		}

		// Check if there are any pending tasks
		bool has_pending() const { return !m_pendingTasks.empty(); }

	private:
		struct AsyncTask : public enki::ITaskSet
		{
			std::function<void()> m_func;

			explicit AsyncTask(std::function<void()> func) : enki::ITaskSet(1), m_func(std::move(func)) {}

			void ExecuteRange(enki::TaskSetPartition /*range*/, uint32_t /*threadnum*/) override { m_func(); }
		};

		std::vector<std::unique_ptr<AsyncTask>> m_pendingTasks;
	};

} // namespace hdt
