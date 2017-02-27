#-------------------------------------------------
#
# Project created by QtCreator 2017-01-18T15:38:40
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = Eurecar-URV
TEMPLATE = app


SOURCES += main.cpp\
        EurecarURV_Dlg.cpp \
    Driving_Dlg.cpp \
    Manipulation_Dlg.cpp \
    Mission_Class/CDriving/CDriving.cpp \
    Mission_Class/CManipulation/CManipulation.cpp \
    Mission_Class/CScript/CScript.cpp \
    Device_Class/CCamera/CCamera.cpp \
    Device_Class/CLRF/CLRF.cpp \
    Device_Class/CKinova/CKinova.cpp \
    Device_Class/CVehicle/CVehicle.cpp \
    ElementTech_Class/CNavigation/CNavigation.cpp \
    ElementTech_Class/CRGBD/CRGBD.cpp \
    ElementTech_Class/CRLearning/CRLearning.cpp \
    Device_Class/CGPS/CGPS.cpp \
    ElementTech_Class/CSegnet/CSegnet.cpp \
    Device_Class/CVelodyne/CVelodyne.cpp \
    Device_Class/CVehicle/RoboteqDevice.cpp \
    Device_Class/CUDP/CUDP.cpp \
    Device_Class/CGripper/CGripper.cpp \
    qcustomplot.cpp \
    ElementTech_Class/CSSD/CSSD.cpp \
    Device_Class/CIMU/3DM-GX/byteswap_utilities.cpp \
    Device_Class/CIMU/3DM-GX/mip_sdk_3dm.cpp \
    Device_Class/CIMU/3DM-GX/mip_sdk_ahrs.cpp \
    Device_Class/CIMU/3DM-GX/mip_sdk_base.cpp \
    Device_Class/CIMU/3DM-GX/mip_sdk_filter.cpp \
    Device_Class/CIMU/3DM-GX/mip_sdk_gps.cpp \
    Device_Class/CIMU/3DM-GX/mip_sdk_inteface.cpp \
    Device_Class/CIMU/3DM-GX/mip_sdk_system.cpp \
    Device_Class/CIMU/3DM-GX/mip_sdk_user_functions.cpp \
    Device_Class/CIMU/3DM-GX/mip.cpp \
    Device_Class/CIMU/3DM-GX/ring_buffer.cpp \
    Device_Class/CIMU/CIMU.cpp \
    Device_Class/CLMS511/CLMS511.cpp


HEADERS  += EurecarURV_Dlg.h \
    Driving_Dlg.h \
    Manipulation_Dlg.h \
    Mission_Class/CDriving/CDriving.h \
    Mission_Class/CManipulation/CManipulation.h \
    Mission_Class/CScript/CScript.h \
    Device_Class/CCamera/CCamera.h \
    Device_Class/CLRF/CLRF.h \
    Device_Class/CKinova/CKinova.h \
    opencv_header.h \
    Device_Class/CVehicle/CVehicle.h \
    ElementTech_Class/CNavigation/CNavigation.h \
    ElementTech_Class/CRGBD/CRGBD.h \
    ElementTech_Class/CRLearning/CRLearning.h \
    Device_Class/CGPS/CGPS.h \
    ElementTech_Class/CSegnet/CSegnet.h \
    Caffe_Header.h \
    Device_Class/CVelodyne/CVelodyne.h \
    Device_Class/CVehicle/Constants.h \
    Device_Class/CVehicle/ErrorCodes.h \
    Device_Class/CVehicle/RoboteqDevice.h \
    Mission_Class/CDriving/Def_Driving.h \
    Device_Class/CUDP/CUDP.h \
    Device_Class/CVelodyne/CPCL.h \
    ElementTech_Class/CRGBD/RGBD_Data.h \
    Mission_Class/CManipulation/Def_Manipulation.h \
    Device_Class/CGripper/CGripper.h \
    Mission_Class/CScript/Def_Script.h \
    qcustomplot.h \
    ElementTech_Class/CSSD/CSSD.h \
    Device_Class/CGPS/gps_struct.h \
    Device_Class/CIMU/3DM-GX/byteswap_utilities.h \
    Device_Class/CIMU/3DM-GX/mainpage.h \
    Device_Class/CIMU/3DM-GX/mip_gx3_35.h \
    Device_Class/CIMU/3DM-GX/mip_gx3_45.h \
    Device_Class/CIMU/3DM-GX/mip_gx4_15.h \
    Device_Class/CIMU/3DM-GX/mip_gx4_25.h \
    Device_Class/CIMU/3DM-GX/mip_gx4_45.h \
    Device_Class/CIMU/3DM-GX/mip_gx4_imu.h \
    Device_Class/CIMU/3DM-GX/mip_rq1_imu.h \
    Device_Class/CIMU/3DM-GX/mip_rq1.h \
    Device_Class/CIMU/3DM-GX/mip_sdk_3dm.h \
    Device_Class/CIMU/3DM-GX/mip_sdk_ahrs.h \
    Device_Class/CIMU/3DM-GX/mip_sdk_base.h \
    Device_Class/CIMU/3DM-GX/mip_sdk_config.h \
    Device_Class/CIMU/3DM-GX/mip_sdk_filter.h \
    Device_Class/CIMU/3DM-GX/mip_sdk_gps.h \
    Device_Class/CIMU/3DM-GX/mip_sdk_interface.h \
    Device_Class/CIMU/3DM-GX/mip_sdk_system.h \
    Device_Class/CIMU/3DM-GX/mip_sdk_user_functions.h \
    Device_Class/CIMU/3DM-GX/mip_sdk.h \
    Device_Class/CIMU/3DM-GX/mip_types.h \
    Device_Class/CIMU/3DM-GX/mip.h \
    Device_Class/CIMU/3DM-GX/ring_buffer.h \
    Device_Class/CIMU/CIMU.h \
    Device_Class/CVelodyne/velodyne_parser.h \
    Device_Class/CLMS511/CLMS511.h \
    Device_Class/CLMS511/LMS511_parser.h

FORMS    += EurecarURV_Dlg.ui \
    Driving_Dlg.ui \
    Manipulation_Dlg.ui

#-------------------------------------------------
#
# Dynamixel Configuration
#
#-------------------------------------------------

INCLUDEPATH += /home/winner/Downloads/DynamixelSDK-3.4.1/c++/include
LIBS += -L/usr/local/lib

LIBS  += -ldxl_x64_cpp
LIBS  += -lrt
#-------------------------------------------------

#-------------------------------------------------
#
# OpenCv Configuration
#
#-------------------------------------------------
INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/lib/x86_64-linux-gnu/hdf5/serial/include
INCLUDEPATH += /usr/local/cuda/include

LIBS += -L/usr/local/lib

LIBS += -lopencv_core
LIBS += -lopencv_videoio
LIBS += -lopencv_highgui
LIBS += -lopencv_imgproc
LIBS += -lopencv_imgcodecs

LIBS += -lopencv_calib3d
LIBS += -lopencv_core
LIBS += -lopencv_features2d
LIBS += -lopencv_flann
LIBS += -lopencv_highgui
LIBS += -lopencv_imgcodecs
LIBS += -lopencv_imgproc
LIBS += -lopencv_ml
LIBS += -lopencv_objdetect
LIBS += -lopencv_photo
LIBS += -lopencv_shape
LIBS += -lopencv_superres
LIBS += -lopencv_video
LIBS += -lopencv_videoio
LIBS += -lopencv_videostab

#-------------------------------------------------
#
# Caffe Configuration
#
#-------------------------------------------------
INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/lib/x86_64-linux-gnu/hdf5/serial/include
INCLUDEPATH += /usr/local/cuda/include
INCLUDEPATH += /home/winner/caffe-ssd/include

LIBS += -L/usr/local/lib
LIBS += -lopencv_core
LIBS += -lopencv_videoio
LIBS += -lopencv_highgui
LIBS += -lopencv_imgproc
LIBS += -lopencv_imgcodecs

LIBS += -L/usr/lib/x86_64-linux-gnu/hdf5/serial
LIBS += -lhdf5

LIBS += -L/home/winner/caffe-ssd/build/lib
LIBS += -lcaffe

LIBS += -L/usr/lib/x86_64-linux-gnu
LIBS += -lboost_system
LIBS += -lglog
LIBS += -lgflags

#-------------------------------------------------
#
# Velodyne - PCL Configuration
#
#-------------------------------------------------
INCLUDEPATH += /usr/local/include/vtk-7.1  \
    /usr/local/include/pcl-1.8 \
    /usr/include/eigen3

LIBS += -L/usr/local/lib \
    -lvtkpng-7.1 \
    -lvtksys-7.1 \
    -lvtkglew-7.1 \
    -lvtkhdf5-7.1 \
    -lvtkjpeg-7.1 \
    -lvtktiff-7.1 \
    -lvtkzlib-7.1 \
    -lvtkexpat-7.1 \
    -lvtkexpat-7.1 \
    -lvtkgl2ps-7.1 \
    -lvtkIOAMR-7.1 \
    -lvtkIOPLY-7.1 \
    -lvtkIOSQL-7.1 \
    -lvtkIOXML-7.1 \
    -lvtkproj4-7.1 \
    -lvtkalglib-7.1 \
    -lvtkIOCore-7.1 \
    -lvtkIOMINC-7.1 \
    -lvtkmetaio-7.1 \
    -lvtkNetCDF-7.1 \
    -lvtksqlite-7.1 \
    -lvtkhdf5_hl-7.1 \
    -lvtkIOImage-7.1 \
    -lvtkIOMovie-7.1 \
    -lvtkIOVideo-7.1 \
    -lvtkIOXML-7.1 \
    -lvtkjsoncpp-7.1 \
    -lvtkCommonSystem-7.1 \
    -lvtkCommonCore-7.1 \
    -lvtkCommonDataModel-7.1 \
    -lvtkCommonMath-7.1 \
    -lvtkGUISupportQt-7.1 \
    -lvtkRenderingQt-7.1 \
    -lvtkGUISupportQtSQL-7.1 \
    -lvtkRenderingCore-7.1 \
    -lvtkRenderingOpenGL2-7.1 \
    -lvtkIOParallel-7.1 \
    -lvtkParallelCore-7.1 \
    -lvtkCommonExecutionModel-7.1 \
    -lvtkIOGeometry-7.1 \
    -lvtkRenderingAnnotation-7.1 \
    -lpcl_io \
    -lpcl_ml \
    -lpcl_common \
    -lpcl_io_ply \
    -lpcl_kdtree \
    -lpcl_ml \
    -lpcl_octree \
    -lpcl_people \
    -lpcl_search \
    -lpcl_stereo \
    -lpcl_filters \
    -lpcl_surface \
    -lpcl_features \
    -lpcl_tracking \
    -lpcl_keypoints \
    -lpcl_outofcore \
    -lpcl_recognition \
    -lpcl_registration \
    -lpcl_segmentation \
    -lpcl_visualization \
    -lpcl_sample_consensus \
    -lboost_system \
    -lgomp
#-------------------------------------------------

#-------------------------------------------------
#
# KINOVA Configuration
#
#-------------------------------------------------
INCLUDEPATH += /opt/kinova/API

LIBS += -L /opt/kinova/API
LIBS += -ldl
#-------------------------------------------------

#-------------------------------------------------
#
# UST-20LX Configuration
#
#-------------------------------------------------

INCLUDEPATH += /usr/local/include/urg_c
INCLUDEPATH += /usr/local/include/urg_cpp

LIBS += -L/usr/local/lib

LIBS += -lurg_c
LIBS += -lurg_cpp
#-------------------------------------------------
