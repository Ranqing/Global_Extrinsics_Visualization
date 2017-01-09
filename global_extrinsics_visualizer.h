#ifndef GLOBAL_EXTRINSICS_VISUALIZER_H
#define GLOBAL_EXTRINSICS_VISUALIZER_H

#include "../../../Qing/qing_common.h"

//a class to visualize global extrinsics
class Qing_Extrinsics_Visualizer
{
public:
    Qing_Extrinsics_Visualizer() {}
    Qing_Extrinsics_Visualizer(const string& sfm_folder, const string& ocv_folder, const int num, const string * names);
    ~Qing_Extrinsics_Visualizer() ;

    int m_cam_num;
    string m_sfm_folder;                                //folder of sfm_based calibration results
    string m_ocv_folder;                                //folder of opencv_based calibration results
    string * m_cam_names;                               //cameras' names

    //sfm_based
    cv::Mat * m_rotations;                              //rotations from world coord to camera coord
    cv::Mat * m_translations;                           //translations from world coord to camera coord
    cv::Mat * m_cam_poses;                          //cameras' poses in world coord
    cv::Mat * m_cam_orientations;                   //cameras' orientation in world coord

    //opencv_based stereo extrinsics
    cv::Mat * m_stereo_rotations;                       //rotations between stereo rigs
    cv::Mat * m_stereo_translations;                    //translation between stereo rigs

    void read_sfm_extrinsics();
    void calc_camera_poses();
    void calc_camera_orientations();
    void save_camera_poses(const string& filename, int with_orientation = false);

    void read_stereo_extrinsics();


    void visualizer_by_ply();
};

#endif // GLOBAL_EXTRINSICS_VISUALIZER_H
