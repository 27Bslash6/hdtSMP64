#pragma once

#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h>

namespace hdt
{
	// Simplified GroupConstraintSolver - provides AVX-optimized constraint row solvers.
	// The batched parallel solving is now handled by btSequentialImpulseConstraintSolverMt
	// via btConstraintSolverPoolMt with s_minimumContactManifoldsForBatching = 8.
	//
	// The previous lock-based parallel solver (SolverBodyMt, SolverTask, etc.) was dead code
	// that was never executed because btConstraintSolverPoolMt dispatches to its internal
	// solver instances, not to this class's solveSingleIteration override.
	class GroupConstraintSolver : public btSequentialImpulseConstraintSolverMt
	{
		typedef btSequentialImpulseConstraintSolverMt Base;

	public:
		GroupConstraintSolver();

		static btSingleConstraintRowSolver getResolveSingleConstraintRowGenericAVX();
		static btSingleConstraintRowSolver getResolveSingleConstraintRowLowerLimitAVX();
	};
} // namespace hdt
