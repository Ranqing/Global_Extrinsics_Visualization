#ifndef GLOBAL_EXTRINSICS_VISUALIZER_H
#define GLOBAL_EXTRINSICS_VISUALIZER_H

#define EVAL 1                        //evaluation sfm extrinsics

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
    cv::Mat * m_cam_poses;                              //cameras' poses in world coord
    cv::Mat * m_cam_orientations;                       //cameras' orientation in world coord
    cv::Mat * m_cam_intrinsics;                         //cameras' intrinsics
    cv::Mat * m_cam_pmatrices;                          //cameras' pmatrices

    //opencv_based stereo extrinsics
    cv::Mat * m_stereo_rotations;                       //rotations between stereo rigs
    cv::Mat * m_stereo_translations;                    //translation between stereo rigs

    void read_sfm_extrinsics();
    void calc_camera_pmatrices();
    void calc_camera_poses();
    void calc_camera_orientations();
    void save_camera_poses(const string& filename, int with_orientation = false);

    void read_stereo_extrinsics();

    vector<cv::Vec3f> m_xaxis_points, m_yaxis_points, m_zaxis_points;
    void calc_axes_points();
    void visualizer_by_ply(const string& plyname);

# if EVAL
    //reconstruct chessboard which size is 20mm
    //1-demension: stereo idx, 0 ~ stereo_size
    //2-demension: frame idx,  0 ~ frame_size
    //3-demension: point idx,  0 ~ point_size
    Size m_board_size;
    vector<vector<vector<cv::Vec3f> > > m_chessboard_points;
    void sfm_triangulate_chessboard(const string calibfolder);
    void sfm_triangulate_chessboard(const string calibfolder, const int camidx0, const int camidx1, vector<vector<Vec3f> >& total_chessboard);
    void eval_sfm_triangulation(const string evalfile);
#endif
};

#endif // GLOBAL_EXTRINSICS_VISUALIZER_H
