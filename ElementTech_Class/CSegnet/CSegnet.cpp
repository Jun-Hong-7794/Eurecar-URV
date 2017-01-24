#include "CSegnet.h"

CSegnet::CSegnet()
{

}

void CSegnet::NetInitialize(const string &_model_file, const string &_trained_file){

    Caffe::set_mode(Caffe::GPU);

    // Load the network
    net_.reset(new Net<float>(_model_file, TEST));
    net_->CopyTrainedLayersFrom(_trained_file);

    Blob<float>* input_layer = net_->input_blobs()[0];

    num_channels_ = input_layer->channels();
    input_geometry_.width = input_layer->width();
    input_geometry_.height = input_layer->height();
}

cv::Mat CSegnet::GetSegnetImage(cv::Mat _org_image){

    cv::Mat segnet_image;
    segnet_image = Predict(_org_image);
    return segnet_image;
}

cv::Mat CSegnet::Predict(const cv::Mat &_img) {

    Blob<float>* input_layer = net_->input_blobs()[0];
    input_layer->Reshape(1, num_channels_,input_geometry_.height, input_geometry_.width);

    // Forward dimension change to all layers
    net_->Reshape();
    std::vector<cv::Mat> input_channels;
    WrapInputLayer(&input_channels);
    Preprocess(_img, &input_channels);
    net_->Forward();

    float* data_addr = input_layer->mutable_cpu_data();
    vector<cv::Mat> img_RGB_channel;
    for(int channel=0;channel < num_channels_;channel++)
    {
        cv::Mat img_f(input_geometry_.height,input_geometry_.width, CV_32FC1, data_addr);
        cv::Mat img_single_channel;
        img_f.convertTo(img_single_channel,CV_8UC3);
        img_RGB_channel.push_back(img_single_channel);
        data_addr += input_geometry_.width*input_geometry_.height;
    }

    cv::Mat color_img;

    cv::merge(img_RGB_channel, color_img);

    const caffe::shared_ptr<Blob<float> > prob = net_->blob_by_name("prob");
    float* prob_addr = prob->mutable_cpu_data();

    vector<int* > label;

    int Unknown[3]={0,0,0};
    int Valve[3]={255,0,0};
    int Valve_holder[3]={247,191,155};
    int panel[3]={0,255,0};
    int rench[3]={0,0,255};
    int rench_holder[3]={127,205,236};

    label.push_back(Unknown);
    label.push_back(Valve);
    label.push_back(Valve_holder);
    label.push_back(panel);
    label.push_back(rench);
    label.push_back(rench_holder);

    cv::Mat result_img(input_geometry_.height,input_geometry_.width,CV_8UC3);

    cv::Mat prob_max = cv::Mat::zeros(input_geometry_.height,input_geometry_.width,CV_32FC2);

    for(int channel=0;channel<prob->channels();channel++)
    {
      cv::Mat prob_f(input_geometry_.height,input_geometry_.width,CV_32FC1,prob_addr);
      for(int x=0;x<prob_f.size().width;x++)
      {

          for(int y=0;y<prob_f.size().height;y++)
          {
              if (prob_f.at<float>(y,x) > prob_max.at<cv::Vec2f>(y,x)[0])
              {
                  prob_max.at<cv::Vec2f>(y,x)[0] = prob_f.at<float>(y,x);
                  prob_max.at<cv::Vec2f>(y,x)[1] = channel;
              }
              if(channel == prob->channels()-1)
              {
                  result_img.at<cv::Vec3b>(y,x)[0] = label.at(prob_max.at<cv::Vec2f>(y,x)[1])[2];
                  result_img.at<cv::Vec3b>(y,x)[1] = label.at(prob_max.at<cv::Vec2f>(y,x)[1])[1];
                  result_img.at<cv::Vec3b>(y,x)[2] = label.at(prob_max.at<cv::Vec2f>(y,x)[1])[0];
              }
          }
      }
      prob_addr += input_geometry_.width*input_geometry_.height;
    }
    return result_img;
}

void CSegnet::WrapInputLayer(std::vector<cv::Mat> *_input_channels){

    Blob<float>* input_layer = net_->input_blobs()[0];

    int width = input_layer->width();
    int height = input_layer->height();
    float* input_data = input_layer->mutable_cpu_data();
    for(int i = 0; i < input_layer->channels();++i) {
        cv::Mat channel(height, width, CV_32FC1, input_data);
        _input_channels->push_back(channel);
        input_data += width * height;
    }
}

void CSegnet::Preprocess(const cv::Mat &_img, std::vector<cv::Mat> *_input_channels){
    /* Convert the input image to the input image format of the network. */
    cv::Mat sample;
    if (_img.channels() == 3 && num_channels_ == 1)
      cv::cvtColor(_img, sample, cv::COLOR_BGR2GRAY);
    else if (_img.channels() == 4 && num_channels_ == 1)
      cv::cvtColor(_img, sample, cv::COLOR_BGRA2GRAY);
    else if (_img.channels() == 4 && num_channels_ == 3)
      cv::cvtColor(_img, sample, cv::COLOR_BGRA2BGR);
    else if (_img.channels() == 1 && num_channels_ == 3)
      cv::cvtColor(_img, sample, cv::COLOR_GRAY2BGR);
    else
      sample = _img;

    cv::Mat sample_resized;
    if (sample.size() != input_geometry_)
      cv::resize(sample, sample_resized, input_geometry_);
    else
      sample_resized = sample;

    cv::Mat sample_float;
    if (num_channels_ == 3)
      sample_resized.convertTo(sample_float, CV_32FC3);
    else
      sample_resized.convertTo(sample_float, CV_32FC1);

    cv::split(sample_float, *_input_channels);

    CHECK(reinterpret_cast<float*>(_input_channels->at(0).data)
          == net_->input_blobs()[0]->cpu_data())
      << "Input channels are not wrapping the input layer of the network.";
}
