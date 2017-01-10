#include "global_extrinsics_visualizer.h"

#define WITH_ORIENTATION 1

int main(int argc, char * argv[])
{
    cout << "Usage: " << argv[0] << " ../../../HumanDatas_20161224/global_extrinsics/res"
         << " ../../../HumanDatas_20161224/calib_results"<< endl;
    if(argc != 3)
    {
        cerr << "invalid arguments." << endl;
        return -1;
    }

    int num = 60;
    string sfm_folder = argv[1];
    string ocv_folder = argv[2];
    string camnames[60] = { "A-01", "A-02", "A-03", "A-04", "A-05", "A-06", "A-07", "A-08", "A-09", "A-10", "A-11", "A-12", "A-13", "A-14", "A-15", "A-16",
                            "B-01", "B-02", "B-03", "B-04", "B-05", "B-06", "B-07", "B-08", "B-09", "B-10", "B-11", "B-12", "B-13", "B-14", "B-15", "B-16",
                            "C-01", "C-02", "C-03", "C-04", "C-05", "C-06", "C-07", "C-08", "C-09", "C-10", "C-11", "C-12", "C-13", "C-14", "C-15", "C-16",
                            "L-01", "L-02", "L-03", "L-04", "L-05", "L-06", "R-01", "R-02", "R-03", "R-04", "R-05", "R-06"} ;

    Qing_Extrinsics_Visualizer * visualizer = new Qing_Extrinsics_Visualizer(sfm_folder, ocv_folder, num, camnames);

    visualizer->read_stereo_extrinsics();

    visualizer->read_sfm_extrinsics();
    visualizer->calc_camera_poses();
    visualizer->calc_camera_orientations();
    visualizer->calc_camera_pmatrices();

# if 0
    string savefile;
    savefile = "./camera_pose.txt";
    visualizer->save_camera_poses(savefile, WITH_ORIENTATION);
    savefile = "./camera_pose.xyz";
    visualizer->save_camera_poses(savefile);
    savefile = "./camera_pose.ply";
    visualizer->visualizer_by_ply(savefile);
# endif

    string calibfolder = "../../../HumanDatas_20161224/calib_rectified";
    visualizer->triangulate_chessboard(calibfolder);

    return 1;
}

