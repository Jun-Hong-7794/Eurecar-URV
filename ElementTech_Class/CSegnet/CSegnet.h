#ifndef CSEGNET_H
#define CSEGNET_H

#include "opencv_header.h"
#include "../../Caffe_Header.h"

class CSegnet
{
public:
    CSegnet();

public:
    cv::Mat GetSegnetImage(cv::Mat _org_image);

};

#endif // CSEGNET_H
