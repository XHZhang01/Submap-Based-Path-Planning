#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <torch/torch.h>


namespace srl {

/// Convert a grayscale OpenCV image to a torch (byte) tensor.
/**
 * \param img   OpenCV image as cv::Mat.
 * \return      Corresponding torch (byte) tensor.
 */
torch::Tensor cvMatToTensor(const cv::Mat& img) {
  return torch::from_blob(img.data, {img.rows, img.cols}, at::kByte);
}

/// Convert a torch (byte) tensor to a grayscale OpenCV image.
/**
 * \param tensor  Torch tensor to convert.
 * \return        Corresponding OpenCV image as cv::Mat.
 */
cv::Mat tensorToCvMatByte(const torch::Tensor& tensor) {
  int width = tensor.sizes()[0];
  int height = tensor.sizes()[1];
  cv::Mat output_mat(cv::Size{height, width}, CV_8UC1, tensor.data_ptr<uchar>());  
  return output_mat.clone();
}

/// Convert a torch (float) tensor to a grayscale OpenCV image.
/**
 * \param tensor  Torch tensor to convert.
 * \return        Corresponding OpenCV image as cv::Mat.
 */
cv::Mat tensorToCvMatFloat(const torch::Tensor& tensor) {
  int width = tensor.sizes()[0];
  int height = tensor.sizes()[1];
  cv::Mat output_mat(cv::Size{height, width}, CV_32F, tensor.data_ptr<float>());  
  return output_mat.clone();
}

}
