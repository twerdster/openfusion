// Updated, checked reader for the original Parameters.txt / recordedN format.
// Derived from source/Source/src/CameraSupport/CameraFromFile.cpp.
#include "CameraFromFile.h"
#include <cutil_inline.h>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>

CameraFromFile::CameraFromFile(char* path)
    : m_currentFrameNumber(1), m_inputDataDepth(nullptr), m_inputDataRgb(nullptr) {
    m_path[0] = '\0';
    if (!path || std::strlen(path) >= sizeof(m_path)) return;
    std::strcpy(m_path, path);
    m_isInitialized = Init();
}
CameraFromFile::~CameraFromFile() {
    delete[] m_inputDataDepth;
    delete[] m_inputDataRgb;
}
bool CameraFromFile::Init() {
    std::ifstream settings(std::string(m_path) + "/Parameters.txt");
    m_params = CameraParams{};
    if (!(settings >> m_params.m_width >> m_params.m_height
          >> m_params.m_intrinsic.m_fx >> m_params.m_intrinsic.m_fy
          >> m_params.m_intrinsic.m_cx >> m_params.m_intrinsic.m_cy
          >> m_params.m_z_offset >> m_params.m_min_depth >> m_params.m_max_depth))
        return false;
    const auto& k = m_params.m_intrinsic;
    if (!m_params.m_width || !m_params.m_height ||
        m_params.m_width > 4096 || m_params.m_height > 4096 ||
        !std::isfinite(k.m_fx) || !std::isfinite(k.m_fy) ||
        k.m_fx == 0 || k.m_fy == 0 || !std::isfinite(k.m_cx) ||
        !std::isfinite(k.m_cy) || !std::isfinite(m_params.m_min_depth) ||
        !std::isfinite(m_params.m_max_depth) ||
        m_params.m_min_depth <= 0 || m_params.m_max_depth <= m_params.m_min_depth ||
        m_params.m_max_depth > std::numeric_limits<CAMERA_DEPTH_TYPE>::max() ||
        !std::isfinite(m_params.m_z_offset) || m_params.m_z_offset != 0)
        return false; // Original pipeline never applies z_offset; reject it explicitly.
    m_params.m_invIntrinsic.Set(1 / k.m_fx, 1 / k.m_fy,
                                -k.m_cx / k.m_fx, -k.m_cy / k.m_fy);
    const size_t pixels = size_t(m_params.m_width) * m_params.m_height;
    m_inputDataDepth = new CAMERA_DEPTH_TYPE[pixels];
    m_inputDataRgb = new CAMERA_RGB_TYPE[pixels];
    return CameraAbstract::Init();
}
namespace {
bool readExact(const std::string& path, void* destination, size_t bytes) {
    std::ifstream input(path, std::ios::binary | std::ios::ate);
    if (!input || input.tellg() != std::streamoff(bytes)) return false;
    input.seekg(0);
    return bool(input.read(static_cast<char*>(destination), bytes));
}
}
bool CameraFromFile::GetDepthFrame(CAMERA_DEPTH_TYPE* output) {
    if (!m_isInitialized || !readExact(std::string(m_path) + "/recorded" +
        std::to_string(m_currentFrameNumber), m_inputDataDepth, m_depthSizeBytes))
        return false;
    if (output) cutilSafeCall(cudaMemcpy(output, m_inputDataDepth,
                                        m_depthSizeBytes, cudaMemcpyHostToDevice));
    ++m_currentFrameNumber;
    return true;
}
bool CameraFromFile::GetRgbFrame(CAMERA_RGB_TYPE* output) {
    if (!m_isInitialized || !readExact(std::string(m_path) + "/recorded-rgb" +
        std::to_string(m_currentFrameNumber - 1), m_inputDataRgb, m_rgbSizeBytes))
        return false;
    if (output) cutilSafeCall(cudaMemcpy(output, m_inputDataRgb,
                                        m_rgbSizeBytes, cudaMemcpyHostToDevice));
    return true;
}
