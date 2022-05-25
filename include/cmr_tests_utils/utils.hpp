#ifndef CMR_TESTS_UTILS__UTILS_HPP
#define CMR_TESTS_UTILS__UTILS_HPP
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

namespace cmr_tests_utils {

// Note that this method introduces a **small** bias
// See https://www.pcg-random.org/posts/bounded-rands.html
// And https://hal.archives-ouvertes.fr/hal-03282794/document
double randMToN(double M, double N)
{
  srand ( time(NULL) );
  return M + (rand() / (RAND_MAX / (N-M) ));  
}

}

#endif