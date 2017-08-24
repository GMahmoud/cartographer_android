#ifndef SRC_CARTOGRAPHER_GENERIC_TEST_H_
#define SRC_CARTOGRAPHER_GENERIC_TEST_H_

#include "cartographer_generic/test_heritage.h"
#include <cmath>
#include <string>
#include <vector>
#include "ceres/crs_matrix.h"
#include "ceres/internal/macros.h"
#include "ceres/internal/port.h"
#include "ceres/iteration_callback.h"
#include "ceres/ordered_groups.h"
#include "ceres/types.h"
#include "ceres/internal/disable_warnings.h"
#include "ceres/problem.h"
#include "ceres/solver.h"
//#include "cartographer_generic/test/problem_test.h"


namespace ceres{

// Interface for non-linear least squares solvers.
class  test {
 public:
  virtual ~test();
  struct  Options {
    Options() {
      minimizer_type = TRUST_REGION;
      line_search_direction_type = LBFGS;
      line_search_type = WOLFE;
      nonlinear_conjugate_gradient_type = FLETCHER_REEVES;
      max_lbfgs_rank = 20;
      use_approximate_eigenvalue_bfgs_scaling = false;
      line_search_interpolation_type = CUBIC;
      min_line_search_step_size = 1e-9;
      line_search_sufficient_function_decrease = 1e-4;
      max_line_search_step_contraction = 1e-3;
      min_line_search_step_contraction = 0.6;
      max_num_line_search_step_size_iterations = 20;
      max_num_line_search_direction_restarts = 5;
      line_search_sufficient_curvature_decrease = 0.9;
      max_line_search_step_expansion = 10.0;
      trust_region_strategy_type = LEVENBERG_MARQUARDT;
      dogleg_type = TRADITIONAL_DOGLEG;
      use_nonmonotonic_steps = false;
      max_consecutive_nonmonotonic_steps = 5;
      max_num_iterations = 50;
      max_solver_time_in_seconds = 1e9;
      num_threads = 1;
      initial_trust_region_radius = 1e4;
      max_trust_region_radius = 1e16;
      min_trust_region_radius = 1e-32;
      min_relative_decrease = 1e-3;
      min_lm_diagonal = 1e-6;
      max_lm_diagonal = 1e32;
      max_num_consecutive_invalid_steps = 5;
      function_tolerance = 1e-6;
      gradient_tolerance = 1e-10;
      parameter_tolerance = 1e-8;

#if defined(CERES_NO_SUITESPARSE) && defined(CERES_NO_CXSPARSE) && !defined(CERES_ENABLE_LGPL_CODE)  // NOLINT
      linear_solver_type = DENSE_QR;
#else
      linear_solver_type = SPARSE_NORMAL_CHOLESKY;
#endif

      preconditioner_type = JACOBI;
      visibility_clustering_type = CANONICAL_VIEWS;
      dense_linear_algebra_library_type = EIGEN;

      // Choose a default sparse linear algebra library in the order:
      //
      //   SUITE_SPARSE > CX_SPARSE > EIGEN_SPARSE > NO_SPARSE
      sparse_linear_algebra_library_type = NO_SPARSE;
#if !defined(CERES_NO_SUITESPARSE)
      sparse_linear_algebra_library_type = SUITE_SPARSE;
#else
  #if !defined(CERES_NO_CXSPARSE)
      sparse_linear_algebra_library_type = CX_SPARSE;
  #else
    #if defined(CERES_USE_EIGEN_SPARSE)
      sparse_linear_algebra_library_type = EIGEN_SPARSE;
    #endif
  #endif
#endif

      num_linear_solver_threads = 1;
      use_explicit_schur_complement = false;
      use_postordering = false;
      dynamic_sparsity = false;
      min_linear_solver_iterations = 0;
      max_linear_solver_iterations = 500;
      eta = 1e-1;
      jacobi_scaling = true;
      use_inner_iterations = false;
      inner_iteration_tolerance = 1e-3;
      logging_type = PER_MINIMIZER_ITERATION;
      minimizer_progress_to_stdout = false;
      trust_region_problem_dump_directory = "/tmp";
      trust_region_problem_dump_format_type = TEXTFILE;
      check_gradients = false;
      gradient_check_relative_precision = 1e-8;
      gradient_check_numeric_derivative_relative_step_size = 1e-6;
      update_state_every_iteration = false;
    }

    bool IsValid(std::string* error) const;
    MinimizerType minimizer_type;
    LineSearchDirectionType line_search_direction_type;
    LineSearchType line_search_type;
    NonlinearConjugateGradientType nonlinear_conjugate_gradient_type;
    int max_lbfgs_rank;
    bool use_approximate_eigenvalue_bfgs_scaling;
    LineSearchInterpolationType line_search_interpolation_type;
    double min_line_search_step_size;
    double line_search_sufficient_function_decrease;
    double max_line_search_step_contraction;
    double min_line_search_step_contraction;
    int max_num_line_search_step_size_iterations;
    int max_num_line_search_direction_restarts;
    double line_search_sufficient_curvature_decrease;
    double max_line_search_step_expansion;
    TrustRegionStrategyType trust_region_strategy_type;
    DoglegType dogleg_type;
    bool use_nonmonotonic_steps;
    int max_consecutive_nonmonotonic_steps;
    int max_num_iterations;
    double max_solver_time_in_seconds;
    int num_threads;
    double initial_trust_region_radius;
    double max_trust_region_radius;
    double min_trust_region_radius;
    double min_relative_decrease;
    double min_lm_diagonal;
    double max_lm_diagonal;
    int max_num_consecutive_invalid_steps;
    double function_tolerance;
    double gradient_tolerance;
    double parameter_tolerance;
    LinearSolverType linear_solver_type;
    PreconditionerType preconditioner_type;
    VisibilityClusteringType visibility_clustering_type;
    DenseLinearAlgebraLibraryType dense_linear_algebra_library_type;
    SparseLinearAlgebraLibraryType sparse_linear_algebra_library_type;
    int num_linear_solver_threads;
    shared_ptr<ParameterBlockOrdering> linear_solver_ordering;
    bool use_explicit_schur_complement;
    bool use_postordering;
    bool dynamic_sparsity;
    bool use_inner_iterations;
    shared_ptr<ParameterBlockOrdering> inner_iteration_ordering;
    double inner_iteration_tolerance;
    int min_linear_solver_iterations;
    int max_linear_solver_iterations;
    double eta;
    bool jacobi_scaling;
    LoggingType logging_type;
    bool minimizer_progress_to_stdout;
    std::vector<int> trust_region_minimizer_iterations_to_dump;
    std::string trust_region_problem_dump_directory;
    DumpFormatType trust_region_problem_dump_format_type;
    bool check_gradients;

    double gradient_check_relative_precision;
    double gradient_check_numeric_derivative_relative_step_size;

    bool update_state_every_iteration;
    std::vector<IterationCallback*> callbacks;
    Solver::Options solver_option_;
  };

  struct  Summary {
    Summary();
    std::string BriefReport() const;
    std::string FullReport() const;
    bool IsSolutionUsable() const;
    MinimizerType minimizer_type;
    TerminationType termination_type;
    std::string message;
    double initial_cost;
    double final_cost;
    double fixed_cost;
    std::vector<IterationSummary> iterations;
    int num_successful_steps;
    int num_unsuccessful_steps;
    int num_inner_iteration_steps;
    int num_line_search_steps;
    double preprocessor_time_in_seconds;
   double minimizer_time_in_seconds;
    double postprocessor_time_in_seconds;
    double total_time_in_seconds;
    double linear_solver_time_in_seconds;
    double residual_evaluation_time_in_seconds;
    double jacobian_evaluation_time_in_seconds;
    double inner_iteration_time_in_seconds;
    double line_search_cost_evaluation_time_in_seconds;
    double line_search_gradient_evaluation_time_in_seconds;
    double line_search_polynomial_minimization_time_in_seconds;
    double line_search_total_time_in_seconds;
    int num_parameter_blocks;
    int num_parameters;
    int num_effective_parameters;
    int num_residual_blocks;
    int num_residuals;
    int num_parameter_blocks_reduced;
    int num_parameters_reduced;
    int num_effective_parameters_reduced;
    int num_residual_blocks_reduced;
    int num_residuals_reduced;
    bool is_constrained;
    int num_threads_given;
    int num_threads_used;
    int num_linear_solver_threads_given;
    int num_linear_solver_threads_used;
    LinearSolverType linear_solver_type_given;
    LinearSolverType linear_solver_type_used;
    std::vector<int> linear_solver_ordering_given;
    std::vector<int> linear_solver_ordering_used;
    std::string schur_structure_given;
    std::string schur_structure_used;
    bool inner_iterations_given;
    bool inner_iterations_used;
    std::vector<int> inner_iteration_ordering_given;
    std::vector<int> inner_iteration_ordering_used;
    PreconditionerType preconditioner_type_given;
    PreconditionerType preconditioner_type_used;
    VisibilityClusteringType visibility_clustering_type;
    TrustRegionStrategyType trust_region_strategy_type;
    DoglegType dogleg_type;
    DenseLinearAlgebraLibraryType dense_linear_algebra_library_type;
    SparseLinearAlgebraLibraryType sparse_linear_algebra_library_type;
    LineSearchDirectionType line_search_direction_type;
    LineSearchType line_search_type;
    LineSearchInterpolationType line_search_interpolation_type;
    NonlinearConjugateGradientType nonlinear_conjugate_gradient_type;
    int max_lbfgs_rank;

  };

 virtual void Solve(const test::Options& options,
                     Problem* problem,
                     test::Summary* summary);
};

 void Solve(const test::Options& options,
           Problem* problem,
           test::Summary* summary);
}  // namespace ceres

#include "ceres/internal/reenable_warnings.h"

#endif  // CERES_PUBLIC_SOLVER_H_
