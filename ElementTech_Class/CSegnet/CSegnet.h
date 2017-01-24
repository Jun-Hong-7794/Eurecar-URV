#ifndef CSEGNET_H
#define CSEGNET_H

#include "opencv_header.h"
#include "../../Caffe_Header.h"

class CSegnet
{
public:
    CSegnet();

public:
    void NetInitialize(const string& _model_file, const string& _trained_file);
    cv::Mat GetSegnetImage(cv::Mat _org_image);

private:
    cv::Mat Predict(const cv::Mat& _img);

    void WrapInputLayer(std::vector<cv::Mat>* _input_channels);

    void Preprocess(const cv::Mat& _img, std::vector<cv::Mat>* _input_channels);

private:
    caffe::shared_ptr<Net<float> > net_;
    cv::Size input_geometry_;
    int num_channels_;
    cv::Mat mean_;
    std::vector<string> labels_;
};

#endif // CSEGNET_H
