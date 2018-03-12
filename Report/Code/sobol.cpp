// These matrices are based on the following publication:
//
// S. Joe and F. Y. Kuo: "Constructing Sobol sequences with better
// two-dimensional projections", SIAM J. Sci. Comput. 30, 2635-2654 (2008).
//
// The tabulated direction numbers are available here:
// http://web.maths.unsw.edu.au/~fkuo/sobol/new-joe-kuo-6.21201

#include "sobol.h"

namespace sobol {

const unsigned Matrices::num_dimensions;
const unsigned Matrices::size;

const unsigned Matrices::matrices[Matrices::num_dimensions * Matrices::size] =
{
    0x80000000U,
    0x40000000U,
    0x20000000U,
    0x10000000U,
    0x8000000U,
    0x4000000U,
    0x2000000U,
    0x1000000U,
    0x800000U,
    0x400000U,
    0x200000U,
    0x100000U,
    0x80000U,
    0x40000U,
    0x20000U,
    0x10000U,
    0x8000U,
    0x4000U,
    0x2000U,
//    ...
    0x59c0d35aU,
    0x34a32b93U,
    0x1397876eU,
};

} // namespace sobol

