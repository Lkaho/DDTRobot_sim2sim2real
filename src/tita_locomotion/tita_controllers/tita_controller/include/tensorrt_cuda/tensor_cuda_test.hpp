#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

class Logger : public nvinfer1::ILogger
{
  void log(Severity severity, const char * msg) noexcept override
  {
    (void)severity;
    (void)msg;
  }
};

class CudaTest
{
private:
  bool cuda_init = false;
  nvinfer1::ICudaEngine * get_engine(const std::string & engine_file_path);
  void cuda_memory_init(void);
  // Cuda pipeline config: 输入obs + 输出actions + 可选estimated_velocity
  float * buffers[3] = {nullptr, nullptr, nullptr};
  size_t input_size = 31 * 10 * sizeof(float);  // 启动后会按engine里的obs维度更新
  size_t actions_output_size = 8 * sizeof(float);
  size_t estimated_velocity_output_size = 0;
  cudaStream_t stream = nullptr;
  nvinfer1::ICudaEngine * engine_ = nullptr;
  nvinfer1::IExecutionContext * context = nullptr;
  bool has_estimated_velocity_output_ = false;
  Logger gLogger;

public:
  // 直接传入按配置组装好的完整历史输入
  void do_inference(float * obs_history, float * actions, float * estimated_velocity = nullptr);
  bool get_cuda_init(void);
  size_t get_input_dim(void) const { return input_size / sizeof(float); }
  bool has_estimated_velocity_output(void) const { return has_estimated_velocity_output_; }
  explicit CudaTest(const std::string & engine_file_path);
  ~CudaTest();
};
