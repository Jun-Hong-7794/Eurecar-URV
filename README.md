# 2017 MBZIRC Challenge 2 Project(Eurecar-URV)
##1. VTK & PCL installation for Velodyne
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
