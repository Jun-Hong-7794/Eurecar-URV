#include "CScript.h"

CScript::CScript(){

}

CScript::CScript(CGPS* _p_gps, CLRF* _p_lrf, CCamera* _p_camera, CKinova* _p_kinova, CVehicle* _p_vehicle, CVelodyne* _p_velodyne, CGripper* _p_gripper){

    mpc_gps = _p_gps;
    mpc_lrf = _p_lrf;
    mpc_camera = _p_camera;
    mpc_kinova = _p_kinova;
    mpc_vehicle = _p_vehicle;
    mpc_velodyne = _p_velodyne;
    mpc_gripper = _p_gripper;

    mpc_drivig = new CDriving(mpc_gps, mpc_lrf, mpc_camera, mpc_kinova, mpc_vehicle, mpc_velodyne);
    mpc_manipulation = new CManipulation(mpc_lrf, mpc_camera, mpc_kinova, mpc_vehicle, mpc_velodyne, mpc_gripper);
}



//----------------------------------------------------------------
//
//                            Run Thread
//
//----------------------------------------------------------------

void CScript::run(){

}
