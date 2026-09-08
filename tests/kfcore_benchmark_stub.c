/** @file kfcore_benchmark_stub.c
 * @author Jan Zwiener (jan@zwiener.org)
 *
 * Stub for KFCore/tests/benchmark/benchmark.h's benchmark(). The real
 * implementation (KFCore/tests/benchmark/benchmark.cpp) times
 * kalman_takasu() against an Eigen reference. Pulling in a C++ compiler
 * and Eigen just for a coverage run buys nothing: we only care about
 * testlinalg()/testnavtoolbox() (correctness) from KFCore/tests/test.c. */

#include "benchmark/benchmark.h"

void benchmark(void) {}
