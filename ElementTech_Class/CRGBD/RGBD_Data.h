#ifndef RGBD_DATA_H
#define RGBD_DATA_H

#define LRF_TILTING_OFFSET 189 //deg : 189 deg is Center
#define LRF_YAWING__OFFSET 135 //deg : (270/2) is Center

#define FLYING_POINT_THRESHOLD 20//mm

#define NUMBER_OF_LAYS 1081

//#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

typedef struct _WORLD_COORDINATE{

    double X;
    double Y;
    double Z;

}WD_COORDINATE;

typedef struct _DEPTH_COORDINATE{

    int x;
    int y;

    double depth;

}DP_COORDINATE;

typedef struct _INTENSITY_COORDINATE{

    int x;
    int y;

    double intensity;

}IT_COORDINATE;

typedef struct _PIXEL_COORDINATE{

    double a;
    double b;
    double s;

    int x;
    int y;

}PX_COORDINATE;

typedef struct _PLANE_PARAMETER{

    double a;
    double b;
    double c;
    double d;

    double Distance; //Distance Vector Parallel to Y Axis

    double yaw; //Angle Between the Plane and X-Y Plane
    double pitch;  //Angle Between the Plane and X-Z Plane

    int num_inlier;

}PLANE_PARAM;

typedef struct _LINE_PARAMETER{

    double a;
    double b;

    double Distance; //Distance Vector Parallel to Y Axis

    double yaw; //Angle Between the Plane and X-Y Plane

    int num_inlier;

}LINE_PARAM;

typedef struct _POINT_PARAMETER{

    double x;
    double y;

}POINT_PARAM;

typedef struct _POINT_CLOUD_AREA{

    int center_x;
    int center_y;

    int width;

    int height;

}PTC_AREA;


#endif // RGBD_DATA_H
