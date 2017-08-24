/*
 * test_header.h
 *
 *  Created on: Aug 7, 2017
 *      Author: maghob
 */

#ifndef SRC_CARTOGRAPHER_GENERIC_TEST_HERITAGE_H_
#define SRC_CARTOGRAPHER_GENERIC_TEST_HERITAGE_H_

#include <cstdarg>
#include <string>

#include "ceres/internal/port.h"

#include <vector>
#include "ceres/ordered_groups.h"
#include "ceres/graph.h"
#include "ceres/types.h"
#include "ceres/program.h"

namespace ceres {
namespace carto {
using std::string;
using namespace std;
using namespace ceres::internal;
#if (defined(__GNUC__) || defined(__clang__))
// Tell the compiler to do printf format string checking if the compiler
// supports it; see the 'format' attribute in
// <http://gcc.gnu.org/onlinedocs/gcc-4.3.0/gcc/Function-Attributes.html>.
//
// N.B.: As the GCC manual states, "[s]ince non-static C++ methods
// have an implicit 'this' argument, the arguments of such methods
// should be counted from two, not one."
#define CERES_PRINTF_ATTRIBUTE(string_index, first_to_check) \
    __attribute__((__format__ (__printf__, string_index, first_to_check)))
#define CERES_SCANF_ATTRIBUTE(string_index, first_to_check) \
    __attribute__((__format__ (__scanf__, string_index, first_to_check)))
#else
#define CERES_PRINTF_ATTRIBUTE(string_index, first_to_check)
#endif

// Return a C++ string.
extern std::string StringPrintf(const char* format, ...)
    // Tell the compiler to do printf format string checking.
    CERES_PRINTF_ATTRIBUTE(1, 2);

// Store result into a supplied string and return it.
extern const std::string& SStringPrintf(std::string* dst, const char* format, ...)
    // Tell the compiler to do printf format string checking.
    CERES_PRINTF_ATTRIBUTE(2, 3);

// Append result to a supplied string.
extern void StringAppendF(std::string* dst, const char* format, ...)
    // Tell the compiler to do printf format string checking.
    CERES_PRINTF_ATTRIBUTE(2, 3);

// Lower-level routine that takes a va_list and appends to a specified string.
// All other routines are just convenience wrappers around it.
extern void StringAppendV(std::string* dst, const char* format, va_list ap);


#undef CERES_PRINTF_ATTRIBUTE

void OrderingToGroupSizes(const ParameterBlockOrdering* ordering,
                          std::vector<int>* group_sizes);

bool ParameterBlocksAreFinite(const Program& program,string* message);

int FindInvalidValue(const int size, const double* x);


void AppendArrayToString(const int size, const double* x, string* result);


bool IsFeasible(const Program& program, string* message);

//Program* CreateReducedProgram(Program* program, vector<double*>* removed_parameter_blocks, double* fixed_cost,string* error);


}  // namespace internal
}  // namespace ceres



#endif /* SRC_CARTOGRAPHER_GENERIC_TEST_HERITAGE_H_ */
