# 2017 MBZIRC Challenge 2 Project(Eurecar-URV)
![KAIST Logo](http://www.kaist.ac.kr/Img/kr/kaist/sym_new_01.gif)
![USRG Background](http://unmanned.kaist.ac.kr/student/photo/all2015.jpg)

##1. Caffe
###1) Cuda 8.0(or Latest Version)
* Download and install Cuda8.0(Latest Version) [here](https://developer.nvidia.com/cuda-downloads)

###2) Cudnn 5.1(or Latest Version)
* Download and install Cudnn5.1(Latest Version) [here](https://developer.nvidia.com/cudnn)

###3) Opencv 3.2(Download [here](https://github.com/opencv/opencv))
* `$ cd opencv-master`
* `$ mkdir build`
* `$ cd build`
* `$ cmake -DWITH_CUDA=ON -DWITH_CUBLAS=ON -DPYTHON_INCLUDE_DIRS=/usr/include/python2.7 -DPYTHON_LIBRARY=/usr/lib/python2.7/config-x86_64-linux-gnu/libpython2.7.so -DWITH_TBB=ON -DWITH_EXAMPLES=ON -DBUILD_EXAMPLES=ON -DWITH_DOC=ON ..`
* `$ make & sudo make install -j8`

###4) Caffe Installation 
* Edit "MkaeConfig" file.(#USE_CUDNN and #OPENCV_VERSION:=3 => comment out)
* `$ make & sudo make install -j8`

##2. VTK & PCL installation for Velodyne
###1) VTK-7.1.0 Installatoin
* `$ sudo apt-get install libeigen3-dev libflann-dev`
* Download VTK-7.1.0 [here](http://www.vtk.org/download/)
* Extract
* `$ cd VTK-7.1.0`
* `$ mkdir build`
* `$ cmake`
* `$ make & sudo make install`
* _Copy_  ~/build/lib/libQVTKWidgetplugin.so   _To_   ~/Qt5.7/Tools/QtCreator/lib/Qt/plugins/designer

###2) PCL-1.8.0 Installation
* Download PCL-1.8.0 [here](https://github.com/PointCloudLibrary/pcl)
* Extract
* `$ cd pcl-pcl-1.8.0`
* `$ mkdir build`
* `$ cmake`
* Add "VTK_Module_INIT(VtkRenderingOpenGL2);" in your source code.

##3. KINOVA(MICO2)
###1) Download SDK and install [here](https://drive.google.com/file/d/0B5d8FVDq3A-XOUludDJnM3ppM28/view)
* `$ cd ~/KINOVA SDK MICO2/Ubuntu/64bits`
* `$ chmod a+x *`
* `$ ./installSDK64.sh`

#Trouble Shooting
### Access to /dev/ttyACM0 with no super user.
* `$ sudo apt-get remove modemmanager`
* `$ sudo usermode -a -G dialout [user name]`
* (Reboot) or (Logout and Login)

