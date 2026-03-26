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
  // Cuda pipeline config: 输入obs(310) + 输出actions(8)
  float * buffers[2];
  size_t input_size = 31 * 10 * sizeof(float);  // 310维历史观测
  size_t output_size = 8 * sizeof(float);
  cudaStream_t stream;
  nvinfer1::ICudaEngine * engine_;
  nvinfer1::IExecutionContext * context;
  Logger gLogger;

public:
  // 只传一个310维的输入（包含历史）
  // 直接传入 310 维组装好的输入
  void do_inference(float * obs_history, float * actions);
  bool get_cuda_init(void);
  explicit CudaTest(const std::string & engine_file_path);
  ~CudaTest();
};
