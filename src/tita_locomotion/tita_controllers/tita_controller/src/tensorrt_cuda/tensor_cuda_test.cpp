#include "tensorrt_cuda/tensor_cuda_test.hpp"

namespace
{
size_t tensor_numel(const nvinfer1::Dims & dims)
{
  size_t numel = 1;
  for (int i = 0; i < dims.nbDims; i++) {
    numel *= static_cast<size_t>(dims.d[i]);
  }
  return numel;
}
}  // namespace


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
  // buffers[0]: 输入 obs
  // buffers[1]: 输出 actions
  // buffers[2]: 可选输出 estimated_velocity
  cudaMalloc(reinterpret_cast<void **>(&buffers[0]), input_size);
  cudaMalloc(reinterpret_cast<void **>(&buffers[1]), actions_output_size);
  if (has_estimated_velocity_output_) {
    cudaMalloc(reinterpret_cast<void **>(&buffers[2]), estimated_velocity_output_size);
  }
}

// Function to do inference
// obs: 按engine要求的完整历史输入
// actions: 8维输出
void CudaTest::do_inference(float * obs, float * actions, float * estimated_velocity)
{
  // 拷贝输入到GPU
  cudaMemcpyAsync(buffers[0], obs, input_size, cudaMemcpyHostToDevice, stream);
  
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#if NV_TENSORRT_MAJOR >= 10
  // TensorRT 10.x: 设置tensor地址
  context->setTensorAddress("obs", buffers[0]);
  context->setTensorAddress("actions", buffers[1]);
  if (has_estimated_velocity_output_) {
    context->setTensorAddress("estimated_velocity", buffers[2]);
  }
  context->enqueueV3(stream);
#else
  context->enqueueV2(reinterpret_cast<void **>(buffers), stream, nullptr);
#endif
#pragma GCC diagnostic pop

  // 拷贝输出回CPU
  cudaMemcpyAsync(actions, buffers[1], actions_output_size, cudaMemcpyDeviceToHost, stream);
  if (has_estimated_velocity_output_ && estimated_velocity != nullptr) {
    cudaMemcpyAsync(
      estimated_velocity, buffers[2], estimated_velocity_output_size, cudaMemcpyDeviceToHost, stream);
  } else if (estimated_velocity != nullptr) {
    estimated_velocity[0] = 0.0f;
    estimated_velocity[1] = 0.0f;
  }
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

      const size_t tensor_size = tensor_numel(dims) * sizeof(float);
      if (std::string(name) == "obs") {
        input_size = tensor_size;
      } else if (std::string(name) == "actions") {
        actions_output_size = tensor_size;
      } else if (std::string(name) == "estimated_velocity") {
        estimated_velocity_output_size = tensor_size;
        has_estimated_velocity_output_ = true;
      }
    }
    
    cuda_memory_init();
    cuda_init = true;
  } else {
    cuda_init = false;
  }
}

CudaTest::~CudaTest()
{
  if (stream != nullptr) {
    cudaStreamDestroy(stream);
  }
  for (void * buf : buffers) {
    if (buf != nullptr) {
      cudaFree(buf);
    }
  }
}
