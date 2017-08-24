/*
 * test_heritage.cc
 *
 *  Created on: Aug 7, 2017
 *      Author: maghob
 */
#include "cartographer_generic/test_heritage.h"

#include <cerrno>
#include <cstdarg>  // For va_list and related operations
#include <cstdio>   // MSVC requires this for _vsnprintf
#include <string>
#include <vector>

#include "ceres/internal/port.h"

#include "ceres/graph.h"
#include "ceres/graph_algorithms.h"
#include "ceres/internal/scoped_ptr.h"
#include "ceres/map_util.h"
#include "ceres/parameter_block.h"
#include "ceres/program.h"
#include "ceres/residual_block.h"
#include "ceres/wall_time.h"
#include "glog/logging.h"



namespace ceres {
namespace carto {

using std::string;
using namespace std;
using namespace ceres::internal;
// va_copy() was defined in the C99 standard.  However, it did not appear in the
// C++ standard until C++11.  This means that if Ceres is being compiled with a
// strict pre-C++11 standard (e.g. -std=c++03), va_copy() will NOT be defined,
// as we are using the C++ compiler (it would however be defined if we were
// using the C compiler).  Note however that both GCC & Clang will in fact
// define va_copy() when compiling for C++ if the C++ standard is not explicitly
// specified (i.e. no -std=c++<XX> arg), even though it should not strictly be
// defined unless -std=c++11 (or greater) was passed.
#if !defined(va_copy)
#if defined (__GNUC__)
// On GCC/Clang, if va_copy() is not defined (C++ standard < C++11 explicitly
// specified), use the internal __va_copy() version, which should be present
// in even very old GCC versions.
#define va_copy(d, s) __va_copy(d, s)
#else
// Some older versions of MSVC do not have va_copy(), in which case define it.
// Although this is required for older MSVC versions, it should also work for
// other non-GCC/Clang compilers which also do not defined va_copy().
#define va_copy(d, s) ((d) = (s))
#endif  // defined (__GNUC__)
#endif  // !defined(va_copy)

void StringAppendV(string* dst, const char* format, va_list ap) {
	// First try with a small fixed size buffer
	char space[1024];

	// It's possible for methods that use a va_list to invalidate
	// the data in it upon use.  The fix is to make a copy
	// of the structure before using it and use that copy instead.
	va_list backup_ap;
	va_copy(backup_ap, ap);
	int result = vsnprintf(space, sizeof(space), format, backup_ap);
	va_end(backup_ap);

	if (result < sizeof(space)) {
		if (result >= 0) {
			// Normal case -- everything fit.
			dst->append(space, result);
			return;
		}

#if defined (_MSC_VER)
		// Error or MSVC running out of space.  MSVC 8.0 and higher
		// can be asked about space needed with the special idiom below:
		va_copy(backup_ap, ap);
		result = vsnprintf(NULL, 0, format, backup_ap);
		va_end(backup_ap);
#endif

		if (result < 0) {
			// Just an error.
			return;
		}
	}

	// Increase the buffer size to the size requested by vsnprintf,
	// plus one for the closing \0.
	int length = result+1;
	char* buf = new char[length];

	// Restore the va_list before we use it again
	va_copy(backup_ap, ap);
	result = vsnprintf(buf, length, format, backup_ap);
	va_end(backup_ap);

	if (result >= 0 && result < length) {
		// It fit
		dst->append(buf, result);
	}
	delete[] buf;
}


string StringPrintf(const char* format, ...) {
	va_list ap;
	va_start(ap, format);
	string result;
	StringAppendV(&result, format, ap);
	va_end(ap);
	return result;
}

const string& SStringPrintf(string* dst, const char* format, ...) {
	va_list ap;
	va_start(ap, format);
	dst->clear();
	StringAppendV(dst, format, ap);
	va_end(ap);
	return *dst;
}

void StringAppendF(string* dst, const char* format, ...) {
	va_list ap;
	va_start(ap, format);
	StringAppendV(dst, format, ap);
	va_end(ap);
}

void OrderingToGroupSizes(const ParameterBlockOrdering* ordering,
                          vector<int>* group_sizes) {
  CHECK_NOTNULL(group_sizes)->clear();
  if (ordering == NULL) {
    return;
  }

  const map<int, set<double*> >& group_to_elements =
      ordering->group_to_elements();
  for (map<int, set<double*> >::const_iterator it = group_to_elements.begin();
       it != group_to_elements.end();
       ++it) {
    group_sizes->push_back(it->second.size());
  }
}

//Program
bool ParameterBlocksAreFinite(const Program& program,string* message){
  CHECK_NOTNULL(message);
  for (int i = 0; i < program.parameter_blocks_.size(); ++i) {
    const ParameterBlock* parameter_block = program.parameter_blocks_[i];
    const double* array = parameter_block->user_state();
    const int size = parameter_block->Size();
    const int invalid_index = FindInvalidValue(size, array);
    if (invalid_index != size) {
      *message = StringPrintf(
          "ParameterBlock: %p with size %d has at least one invalid value.\n"
          "First invalid value is at index: %d.\n"
          "Parameter block values: ",
          array, size, invalid_index);
      AppendArrayToString(size, array, message);
      return false;
    }
  }
  return true;
}

int FindInvalidValue(const int size, const double* x) {
  if (x == NULL) {
    return size;
  }
}

void AppendArrayToString(const int size, const double* x, string* result) {
  for (int i = 0; i < size; ++i) {
    if (x == NULL) {
      StringAppendF(result, "Not Computed  ");
    } else {
      if (x[i] == kImpossibleValue) {
        StringAppendF(result, "Uninitialized ");
      } else {
        StringAppendF(result, "%12g ", x[i]);
      }
    }
  }
}

bool IsFeasible(const Program& program, string* message) {
  CHECK_NOTNULL(message);
  for (int i = 0; i < program.parameter_blocks_.size(); ++i) {
    const ParameterBlock* parameter_block = program.parameter_blocks_[i];
    const double* parameters = parameter_block->user_state();
    const int size = parameter_block->Size();
    if (parameter_block->IsConstant()) {
      // Constant parameter blocks must start in the feasible region
      // to ultimately produce a feasible solution, since Ceres cannot
      // change them.
      for (int j = 0; j < size; ++j) {
        const double lower_bound = parameter_block->LowerBoundForParameter(j);
        const double upper_bound = parameter_block->UpperBoundForParameter(j);
        if (parameters[j] < lower_bound || parameters[j] > upper_bound) {
          *message = StringPrintf(
              "ParameterBlock: %p with size %d has at least one infeasible "
              "value."
              "\nFirst infeasible value is at index: %d."
              "\nLower bound: %e, value: %e, upper bound: %e"
              "\nParameter block values: ",
              parameters, size, j, lower_bound, parameters[j], upper_bound);
          AppendArrayToString(size, parameters, message);
          return false;
        }
      }
    } else {
      // Variable parameter blocks must have non-empty feasible
      // regions, otherwise there is no way to produce a feasible
      // solution.
      for (int j = 0; j < size; ++j) {
        const double lower_bound = parameter_block->LowerBoundForParameter(j);
        const double upper_bound = parameter_block->UpperBoundForParameter(j);
        if (lower_bound >= upper_bound) {
          *message = StringPrintf(
              "ParameterBlock: %p with size %d has at least one infeasible "
              "bound."
              "\nFirst infeasible bound is at index: %d."
              "\nLower bound: %e, upper bound: %e"
              "\nParameter block values: ",
              parameters, size, j, lower_bound, upper_bound);
          AppendArrayToString(size, parameters, message);
          return false;
        }
      }
    }
  }

  return true;
}


//Program* CreateReducedProgram(Program* program,
//    vector<double*>* removed_parameter_blocks,
//    double* fixed_cost,
//    string* error) {
//  CHECK_NOTNULL(removed_parameter_blocks);
//  CHECK_NOTNULL(fixed_cost);
//  CHECK_NOTNULL(error);
//
//  scoped_ptr<Program> reduced_program(new Program(*program));
//  if (!reduced_program->RemoveFixedBlocks(removed_parameter_blocks,
//                                          fixed_cost,
//                                          error)) {
//    return NULL;
//  }
//
//  reduced_program->SetParameterOffsetsAndIndex();
//  return reduced_program.release();
//}












}  // namespace internal



}  // namespace ceres
