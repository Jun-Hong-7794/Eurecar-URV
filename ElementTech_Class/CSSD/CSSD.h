#ifndef CSSD_H
#define CSSD_H

#include "opencv_header.h"
#include "../../Caffe_Header.h"

class CSSD {

 public:
    CSSD();

  void NetInitialize(const string& model_file,
           const string& weights_file,
           const string& mean_value);

  vector<vector<int>> GetSSDImage(cv::Mat& _org_image);

 private:
  std::string mstr_model_path;
  std::string mstr_weight_path;
  std::string mstr_mean_file;
  std::string mstr_mean_value;

 private:
  void SetMean(const string& mean_value);

  void WrapInputLayer(std::vector<cv::Mat>* input_channels);

  void Preprocess(const cv::Mat& img,
                  std::vector<cv::Mat>* input_channels);

 private:
  caffe::shared_ptr<Net<float> > net_;
  cv::Size input_geometry_;
  int num_channels_;
  cv::Mat mean_;
};



#endif // CSSD_H
