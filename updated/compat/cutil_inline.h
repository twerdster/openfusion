#pragma once
// Updated compatibility layer: only the runtime checks used by the replay core.
// This does not reproduce the obsolete CUDA SDK cutil library.
#include <cuda_runtime.h>
#include <cstdio>
#include <stdexcept>
#include <string>
inline void openfusionCudaCheck(cudaError_t error, const char* expression,
                               const char* file, int line) {
    if (error == cudaSuccess) return;
    const std::string message = std::string(file) + ":" + std::to_string(line) +
        " " + expression + ": " + cudaGetErrorString(error);
    throw std::runtime_error(message);
}
#define cutilSafeCall(call) openfusionCudaCheck((call), #call, __FILE__, __LINE__)
#define cutilCheckMsg(message) openfusionCudaCheck(cudaGetLastError(), (message), __FILE__, __LINE__)
