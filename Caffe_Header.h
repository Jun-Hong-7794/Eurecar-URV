#ifndef CAFFE_HEADER_H
#define CAFFE_HEADER_H
#include <caffe/caffe.hpp>
#include <caffe/proto/caffe.pb.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <typeinfo>
#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <boost/array.hpp>
#include <time.h>
#include <limits.h>
#define THRES_NUM 0.7

using namespace caffe;  // NOLINT(build/namespaces)
using std::string;
using namespace std;


/* Pair (label, confidence) representing a prediction. */
typedef std::pair<string, float> Prediction;
#endif // CAFFE_HEADER_H
