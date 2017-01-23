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
* Edit "Makefile.config" file.(#USE_CUDNN and #OPENCV_VERSION:=3 => comment out)
* `$ sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libhdf5-serial-dev protobuf-compiler`
* `$ sudo apt-get install --no-install-recommends libboost-all-dev`
* `$ sudo apt-get install libatlas-base-dev`
* `$ sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev`
* `$ mkdir build`
* `$ cd build `
* `$ cmake ..`
* `$ make -j8`
* `$ make py`
* `$ make install`


##2. VTK & PCL installation for Velodyne
###1) VTK-7.1.0 Installatoin
* `$ sudo apt-get install libeigen3-dev libflann-dev`
* Download VTK-7.1.0 [here](http://www.vtk.org/download/)
* Extract
* `$ cd VTK-7.1.0`
* `$ mkdir build`
* `$ cd build`
* `$ cmake -DCMAKE_BUILD_TYPE:STRING=Release \ 
   -DVTK_QT_VERSION:STRING=5 \
   -DQT_QMAKE_EXECUTABLE:PATH=/home/winner/Qt5.7.0/5.7/gcc_64/bin/qmake \
   -DVTK_Group_Qt:BOOL=ON \
   -DCMAKE_PREFIX_PATH:PATH=/home/winner/Qt5.7.0/5.7/gcc_64/lib/cmake  \
   -DBUILD_SHARED_LIBS:BOOL=ON ..`
* `$ make & sudo make install`
* _Copy_  ~/build/lib/libQVTKWidgetplugin.so   _To_   ~/Qt5.7/Tools/QtCreator/lib/Qt/plugins/designer

###2) PCL-1.8.0 Installation
* Download PCL-1.8.0 [here](https://github.com/PointCloudLibrary/pcl)
* Extract
* `$ cd pcl-pcl-1.8.0`
* `$ mkdir build`
* `$ cd build`
* `$ cmake -DCMAKE_BUILD_TYPE=Release ..`
* Add "VTK_Module_INIT(VtkRenderingOpenGL2);" in your source code.

##3. KINOVA(MICO2)
###1) Download SDK and install [here](https://drive.google.com/file/d/0B5d8FVDq3A-XOUludDJnM3ppM28/view)
* `$ cd ~/KINOVA SDK MICO2/Ubuntu/64bits`
* `$ chmod a+x *`
* `$ ./installSDK64.sh`

##4. Hokuyo LRF(UTM-30LX)
###1) Download urg_library-1.2.0.zip [here](https://sourceforge.net/projects/urgnetwork/files/urg_library/)
* Extract
* `$ cd ~/urg_library-1.2.0`
* `$ make`
* `$ sudo make install`

#Trouble Shooting
### Access to /dev/ttyACM0 with no super user.
* `$ sudo apt-get remove modemmanager`
* `$ sudo usermode -a -G dialout [user name]`
* (Reboot) or (Logout and Login)

