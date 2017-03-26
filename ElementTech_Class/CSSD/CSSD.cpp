#include "CSSD.h"
#include "QTime"
#include <iostream>
CSSD::CSSD()
{
    mstr_model_path = "/home/winner/caffe-ssd/models/VGGNet/mbzirc/SSD_300x300/deploy.prototxt";
    mstr_weight_path = "/home/winner/caffe-ssd/models/VGGNet/mbzirc/SSD_300x300/VGG_mbzirc_SSD_300x300_iter_41662.caffemodel";
    mstr_mean_value =  "104,117,123";

    NetInitialize(mstr_model_path, mstr_weight_path, mstr_mean_value);
}


void CSSD::NetInitialize(const string& model_file,
                   const string& weights_file,
                   const string& mean_value) {
#ifdef CPU_ONLY
  Caffe::set_mode(Caffe::CPU);
#else

#endif

    Caffe::set_mode(Caffe::GPU);
    bool gpu_available = Caffe::CheckDevice(0);
    if(gpu_available)
    {
        Caffe::SetDevice(0);
    }

  /* Load the network. */
  net_.reset(new Net<float>(model_file, TEST));
  net_->CopyTrainedLayersFrom(weights_file);

  CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
  CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

  Blob<float>* input_layer = net_->input_blobs()[0];
  num_channels_ = input_layer->channels();
  CHECK(num_channels_ == 3 || num_channels_ == 1)
    << "Input layer should have 1 or 3 channels.";
  input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

  /* Load the binaryproto mean file. */
  SetMean(mean_value);
}

vector<vector<int>> CSSD::GetSSDImage(cv::Mat& _org_image) {
  mtx_ssd.lock();
  Blob<float>* input_layer = net_->input_blobs()[0];
  input_layer->Reshape(1, num_channels_,
                       input_geometry_.height, input_geometry_.width);
  /* Forward dimension change to all layers. */
  net_->Reshape();

  std::vector<cv::Mat> input_channels;
  WrapInputLayer(&input_channels);

  Preprocess(_org_image, &input_channels);

  Caffe::set_mode(Caffe::GPU);
  net_->Forward();

  /* Copy the output layer to a std::vector */
  Blob<float>* result_blob = net_->output_blobs()[0];
  const float* result = result_blob->cpu_data();
  const int num_det = result_blob->height();
  vector<vector<float> > detections;
  for (int k = 0; k < num_det; ++k) {
    if (result[0] == -1) {
      // Skip invalid detection.
      result += 7;
      continue;
    }
    vector<float> detection(result, result + 7);
    detections.push_back(detection);
    result += 7;
  }

  vector<vector<int>> bb_info;
  rench_loc_list.clear();
  /* Print the detection results. */
  for (unsigned int i = 0; i < detections.size(); ++i) {
    const vector<float>& d = detections[i];
    // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
    CHECK_EQ(d.size(), 7);
    const float score = d[2];
    if (score >= 0.25) {
      if(!((d[1] == 1)|| (d[1] == 2)))
            continue;
      cv::Point2d tl,br;
      tl.x = d[3] * _org_image.cols;
      tl.y = d[4] * _org_image.rows;
      br.x = d[5] * _org_image.cols;
      br.y = d[6] * _org_image.rows;

      vector<cv::Point> points;
      points.push_back(tl);
      points.push_back(br);
      cv::Rect boundRect = cv::boundingRect(points);
      string labelstring;
      cv::Scalar box_color;
      vector<int> bb_info_xy;
      double bb_box_mean;
      switch(int(d[1]))
      {
      case 1:
          labelstring = "Rench";
          box_color = cv::Scalar(255,171,0);
          bb_info_xy.push_back(br.x);
          bb_info_xy.push_back(br.y);
          bb_info.push_back(bb_info_xy);
          bb_box_mean= 0.5*(tl.x + br.x);
          rench_loc_list.push_back((int)bb_box_mean);

          break;
      case 2:
          labelstring = "Valve";
          box_color = cv::Scalar(0,0,255);
          break;
      default:

          break;
      }
      cv::rectangle(_org_image,boundRect,box_color,2);
      cv::putText(_org_image,labelstring,tl,cv::FONT_HERSHEY_SIMPLEX,0.7,CV_RGB(255,255,255),2);
    }
  }

  if(rench_loc_list.size() != 0)
  {

      std::sort(rench_loc_list.begin(),rench_loc_list.end());

      for(unsigned int i = 0; i < rench_loc_list.size();i++)
      {
          rench_loc_list_arr[i] = (int)rench_loc_list.at(i);
      }
//      std::cout << "Localization List: " << rench_loc_list.at(rench_loc_list.size() - 1)<< std::endl;
  }
  mtx_ssd.unlock();

  return bb_info;
}

vector<int> CSSD::GetRenchLocList()
{
    mtx_ssd.lock();
    vector<int> rench_loc_return;

    rench_loc_return = rench_loc_list;

    mtx_ssd.unlock();
    return rench_loc_return;

}

int* CSSD::GetRenchLocListArr()
{
    return rench_loc_list_arr;
}

/* Load the mean file in binaryproto format. */
void CSSD::SetMean(const string& mean_value) {
  if (!mean_value.empty()) {
    stringstream ss(mean_value);
    vector<float> values;
    string item;
    while (getline(ss, item, ',')) {
      float value = std::atof(item.c_str());
      values.push_back(value);
    }
    CHECK(values.size() == 1 || values.size() == num_channels_) <<
      "Specify either 1 mean_value or as many as channels: " << num_channels_;

    std::vector<cv::Mat> channels;
    for (int i = 0; i < num_channels_; ++i) {
      /* Extract an individual channel. */
      cv::Mat channel(input_geometry_.height, input_geometry_.width, CV_32FC1,
          cv::Scalar(values[i]));
      channels.push_back(channel);
    }
    cv::merge(channels, mean_);
  }
}

/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */
void CSSD::WrapInputLayer(std::vector<cv::Mat>* input_channels) {
  Blob<float>* input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i) {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}

void CSSD::Preprocess(const cv::Mat& img,
                            std::vector<cv::Mat>* input_channels) {
  /* Convert the input image to the input image format of the network. */
  cv::Mat sample;
  if (img.channels() == 3 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
  else if (img.channels() == 4 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
  else if (img.channels() == 4 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
  else if (img.channels() == 1 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
  else
    sample = img;

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

  cv::Mat sample_normalized;
  cv::subtract(sample_float, mean_, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_normalized, *input_channels);

  CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
        == net_->input_blobs()[0]->cpu_data())
    << "Input channels are not wrapping the input layer of the network.";
}
