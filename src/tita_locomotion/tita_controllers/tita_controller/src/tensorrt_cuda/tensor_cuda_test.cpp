#include "tensorrt_cuda/tensor_cuda_test.hpp"


nvinfer1::ICudaEngine * CudaTest::get_engine(const std::string & engine_file_path)
{
  std::ifstream file(engine_file_path, std::ios::binary);
  if (!file.good()) {
    return nullptr;
  }
  std::vector<char> engine_data(
    (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  file.close();
  nvinfer1::IRuntime * runtime = nvinfer1::createInferRuntime(gLogger);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#if NV_TENSORRT_MAJOR >= 10
  return runtime->deserializeCudaEngine(engine_data.data(), engine_data.size());
#else
  return runtime->deserializeCudaEngine(engine_data.data(), engine_data.size(), nullptr);
#endif

#pragma GCC diagnostic pop
}

void CudaTest::cuda_memory_init(void)
{
  cudaStreamCreate(&stream);
  // buffers[0]: 输入 obs (310维)
  // buffers[1]: 输出 actions (8维)
  cudaMalloc(reinterpret_cast<void **>(&buffers[0]), input_size);
  cudaMalloc(reinterpret_cast<void **>(&buffers[1]), output_size);
}

// Function to do inference
// obs: 310维输入 (包含31维当前观测 + 历史)
// actions: 8维输出
void CudaTest::do_inference(float * obs, float * actions)
{
  // 拷贝输入到GPU
  cudaMemcpyAsync(buffers[0], obs, input_size, cudaMemcpyHostToDevice, stream);
  
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#if NV_TENSORRT_MAJOR >= 10
  // TensorRT 10.x: 设置tensor地址
  context->setTensorAddress("obs", buffers[0]);
  context->setTensorAddress("actions", buffers[1]);
  context->enqueueV3(stream);
#else
  context->enqueueV2(reinterpret_cast<void **>(buffers), stream, nullptr);
#endif
#pragma GCC diagnostic pop

  // 拷贝输出回CPU
  cudaMemcpyAsync(actions, buffers[1], output_size, cudaMemcpyDeviceToHost, stream);
  cudaStreamSynchronize(stream);
}

bool CudaTest::get_cuda_init(void) { return cuda_init; }

CudaTest::CudaTest(const std::string & engine_file_path)
{
  engine_ = get_engine(engine_file_path);
  if (engine_ != nullptr) {
    context = engine_->createExecutionContext();
    
    // Debug: 打印 engine 输入输出维度
    std::cout << "[DEBUG] Engine bindings: " << engine_->getNbIOTensors() << std::endl;
    for (int i = 0; i < engine_->getNbIOTensors(); i++) {
      const char* name = engine_->getIOTensorName(i);
      auto dims = engine_->getTensorShape(name);
      std::cout << "[DEBUG] Tensor " << i << " (" << name << "): dims=";
      for (int j = 0; j < dims.nbDims; j++) {
        std::cout << dims.d[j] << " ";
      }
      std::cout << std::endl;
    }
    
    cuda_memory_init();
    cuda_init = true;
  } else {
    cuda_init = false;
  }
}

CudaTest::~CudaTest()
{
  cudaStreamDestroy(stream);
  for (void * buf : buffers) {
    cudaFree(buf);
  }
}
