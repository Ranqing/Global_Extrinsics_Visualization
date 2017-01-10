#include "global_extrinsics_visualizer.h"

#include "../../../Qing/qing_string.h"
#include "../../../Qing/qing_basic.h"
#include "../../../Qing/qing_io.h"
#include "../../../Qing/qing_ply.h"
#include "../../../Qing/qing_dir.h"

void qing_read_sfm_pmatrix(const string filename, Mat& intrinsic, Mat& rotation, Mat& translation)
{
    fstream fin(filename.c_str(), ios::in);
    if(fin.is_open() == false)
    {
        cerr << "failed to open " << filename << endl;
        return ;
    }
    cout << filename << endl;

    double kdata[9] = {0.0};    //intrinsic data
    double rdata[9] = {0.0};    //rotation data
    double tdata[3] = {0.0};    //tranlatio  data

    string line;
    vector<string> words(0);
    while(getline(fin, line))
    {
        if('#' != line[0])  continue;
        words.clear();
        qing_split_a_string_by_space(line.substr(1), words);

        if ("width" == words[0])       {  getline(fin, line);  continue;  }         //width height
        if ("focal" == words[0])       {                                            //focal centerx centery
            double focal, centerx, centery;
            fin >> focal >> centerx >> centery;
            kdata[0] = focal;
            kdata[2] = centerx;
            kdata[4] = focal;
            kdata[5] = centery;
            kdata[8] = 1.0;
            continue;
        }
        if ("rotation" == words[0])    {                                            //rotation
            for(int i = 0; i < 9; ++i)
                fin >> rdata[i];
            continue;
        }
        if ("translation" == words[0]) {                                            //translation
            for(int i = 0; i < 3; ++i)
                fin >> tdata[i];
            continue;
        }
    }
    fin.close();

    intrinsic.create(3, 3, CV_64FC1);
    memcpy(intrinsic.data, kdata, sizeof(double)*9);
    rotation.create(3, 3, CV_64FC1);
    memcpy(rotation.data, rdata, sizeof(double)*9);
    translation.create(3,1,CV_64FC1);
    memcpy(translation.data, tdata, sizeof(double)*3);
}


Mat qing_cam_coord_to_world_coord(const Mat& rotation, const Mat& translation, const Mat& c_coord)
{
    Mat w_coord = rotation.t() * (c_coord - translation);
    return w_coord;
}

#define BOARD_W 14
#define BOARD_H 24
#define SQUARESIZE 20            //mm

void qing_set_chessboard_size(const string camname, Size& boardSize)
{
    if('A' == camname[0] || 'B' == camname[0] || 'C' == camname[0])
        boardSize = Size(BOARD_W, BOARD_H);   //14 * 24
    else
        boardSize = Size(BOARD_H, BOARD_W);   //24 * 14
}

bool qing_extract_corners(const string& imgname, const Size& boardSize, vector<Point2f>& corners)
{
    corners.clear();
    corners.resize(0);

    Mat view, gray_view;
    view = imread(imgname, CV_LOAD_IMAGE_UNCHANGED);
    if(view.data == NULL)
    {
        cerr << "failed to open stereo images " << endl;
        return false;
    }
    cvtColor(view, gray_view, CV_BGR2GRAY);

    bool found = findChessboardCorners(gray_view, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    if(false == found)
    {
        cerr << "failed to find chessboard corners." << endl;
        return false;
    }
    cornerSubPix(gray_view, corners, Size(11, 11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    return true;
}

void qing_calc_recons_error(vector<Vec3f>& object_points, Size board_size, double& total_recons_err, double& max_recons_err)
{
    int w = board_size.width;
    int h = board_size.height;

    //horizon
    for(int i = 0; i < h; ++i)
    {
        for(int j = 0; j < w-1; ++j)
        {
            int idx0 = i * w + j;
            int idx1 = idx0 + 1;

            float dis = qing_euclidean_dis(object_points[idx1] , object_points[idx0]);
            total_recons_err += dis;

            if(dis > max_recons_err)  max_recons_err = dis;
        }
    }

    // vertical
    for(int i = 0; i < h-1; ++i)
    {
        for(int j = 0; j < w; ++j)
        {
            int idx0 = i * w + j;
            int idx1 = idx0 + w;

            float dis = qing_euclidean_dis(object_points[idx1] , object_points[idx0]);
            total_recons_err += dis;

            if(dis > max_recons_err )  max_recons_err = dis;
        }
    }

  //  cout << "max_recons_err = " << max_recons_err << endl;
}


Qing_Extrinsics_Visualizer::Qing_Extrinsics_Visualizer(const string &sfm_folder, const string &ocv_folder, const int num, const string *names):
    m_sfm_folder(sfm_folder), m_ocv_folder(ocv_folder), m_cam_num(num)
{
    cout << "sfm result folder: "    << m_sfm_folder  << endl;
    cout << "ocv result folder: "    << m_ocv_folder   << endl;
    cout << "camera num: "           << m_cam_num     << endl;

    m_cam_names = new string[m_cam_num];
    m_rotations = new Mat[m_cam_num];
    m_translations = new Mat[m_cam_num];
    m_cam_poses = new Mat[m_cam_num];
    m_cam_orientations = new Mat[m_cam_num];
    m_cam_intrinsics = new Mat[m_cam_num];
    m_cam_pmatrices  = new Mat[m_cam_num];

    for(int i = 0; i < m_cam_num; ++i)
    {
        m_cam_names[i] = names[i];
    }

}

Qing_Extrinsics_Visualizer::~Qing_Extrinsics_Visualizer()
{
    if(m_cam_names != NULL)         { delete[] m_cam_names; m_cam_names = NULL;}
    if(m_rotations != NULL)         { delete[] m_rotations; m_rotations = NULL;}
    if(m_translations != NULL)      { delete[] m_translations; m_translations = NULL; }
    if(m_cam_poses != NULL)         { delete[] m_cam_poses; m_cam_poses = NULL;}
    if(m_cam_orientations != NULL)  { delete[] m_cam_orientations; m_cam_orientations = NULL; }

    if(m_cam_intrinsics != NULL)    { delete[] m_cam_intrinsics; m_cam_intrinsics = NULL; }
    if(m_cam_pmatrices != NULL)     { delete[] m_cam_pmatrices;  m_cam_pmatrices = NULL;  }
}


//known: openmvg data: p_world_coord = rotation.inv() * p_cam_coord + translation
//
void Qing_Extrinsics_Visualizer::read_sfm_extrinsics()
{
    for(int i = 0; i < m_cam_num; ++i)
    {
        string filename = m_sfm_folder + "/" + m_cam_names[i] + ".txt";
        Mat rotation, translation;
        Mat intrinsic;
        qing_read_sfm_pmatrix(filename, intrinsic, rotation, translation);

        m_cam_intrinsics[i] = intrinsic;

        //openmvg:
        //p_world_coord = rotation.inv() * p_cam_coord + translation <==> p_cam_coord = rotation * p_world_coord - rotation * translation
        m_rotations[i] = rotation;
        m_translations[i] = -(rotation * translation);

# if 0
        cout << m_cam_names[i] <<" : " << endl ;
        cout << m_cam_intrinsics[i] << endl;
        cout << m_rotations[i] << endl << m_translations[i] << endl;
# endif
    }
}

void Qing_Extrinsics_Visualizer::calc_camera_pmatrices()
{
    for(int i = 0; i < m_cam_num; ++i)
    {
        double p[12];
        double * rdata  = (double *)m_rotations[i].ptr<double>(0);
        double * tdata  = (double *)m_translations[i].ptr<double>(0);

        p[0] = rdata[0] ;   p[1] = rdata[1] ;   p[2] = rdata[2] ;  p[3] = tdata[0];
        p[4] = rdata[3] ;   p[5] = rdata[4] ;   p[6] = rdata[5] ;  p[7] = tdata[1];
        p[8] = rdata[6] ;   p[9] = rdata[7] ;   p[10] = rdata[8] ; p[11] = tdata[2];

        m_cam_pmatrices[i].create(3, 4, CV_64FC1);
        memcpy(m_cam_pmatrices[i].data, p, sizeof(double)*12);

        cout << m_cam_names[i] << " : " << endl << m_cam_pmatrices[i] << endl;
    }
}

void Qing_Extrinsics_Visualizer::calc_camera_poses()
{
    for(int i = 0; i < m_cam_num; ++i)
    {
        double * rdata  = (double *)m_rotations[i].ptr<double>(0);
        double * tdata  = (double *)m_translations[i].ptr<double>(0);
        double * center = qing_calc_cam_center(rdata, tdata);              //center in qing_calc_cam_center must be create by 'new', or data in this memory will be freshed by 'create' then

        m_cam_poses[i].create(3,1,CV_64FC1);
        memcpy(m_cam_poses[i].data, center, sizeof(double)*3);
#if 0
        cout << m_cam_names[i] << " : " << m_cam_poses[i].t() << endl;
#endif
        delete[] center;
    }
}

void Qing_Extrinsics_Visualizer::calc_camera_orientations()
{ 
    for(int i = 0; i < m_cam_num; ++i)
    {
        Mat c_coord = (Mat_<double>(3,1) << 1.0, 0.0, 0.0);                                              //camera_coord
        Mat w_coord = qing_cam_coord_to_world_coord(m_rotations[i], m_translations[i], c_coord) ;        //m_rotations[i].t() * (c_coord - m_translations[i]);

        m_cam_orientations[i] = w_coord.clone();
# if 0
        cout << m_cam_names[i] << " orientation: " << m_cam_orientations[i].t() << endl;
#endif
    }
}

void Qing_Extrinsics_Visualizer::save_camera_poses(const string& filename, int with_orientation /*= true*/)
{
    fstream fout(filename.c_str(), ios::out);
    for(int i = 0; i < m_cam_num; ++i)
    {
        double * pdata = (double *)m_cam_poses[i].ptr<double>(0);
        fout << pdata[0] << ' ' << pdata[1] << ' ' << pdata[2] << ' ';

        if(with_orientation)
        {
            pdata = (double *)m_cam_orientations[i].ptr<double>(0);
            fout << pdata[0] << ' ' << pdata[1] << ' ' << pdata[2] << ' ';
        }

        fout << endl;
    }
    fout.close();
    cout << "saving " << filename << " done." << endl;
}


void Qing_Extrinsics_Visualizer::read_stereo_extrinsics()
{
    cout << "here is read stereo extrinsics.." << endl;
    for(int i = 0; i < m_cam_num - 1; i+=2)
    {
        string stereoname = m_cam_names[i+0].substr(0,1) + m_cam_names[i+0].substr(2) +
                m_cam_names[i+1].substr(0,1) + m_cam_names[i+1].substr(2);
        string filename = m_ocv_folder + "/stereo_" + stereoname + ".yml";
        Mat rotation, translation;

        qing_read_stereo_yml_rt(filename, rotation, translation);
#if 0
        cout << stereoname  << endl;
        cout << rotation    << endl;
        cout << translation << endl;
#endif
    }
}

#define STEP 0.005

void Qing_Extrinsics_Visualizer::calc_axes_points()
{
    m_xaxis_points.clear();
    m_yaxis_points.clear();
    m_zaxis_points.clear();

    int xpointsize = 0.05/STEP;
    int ypointsize = 0.075/STEP;
    int zpointsize = 0.075/STEP;

    for(int c = 0; c < m_cam_num; c++)
    {
        Mat rotation = m_rotations[c];
        Mat translation = m_translations[c];

        //x-axis
        double cur = 0.0;
        for(int i = 0; i < xpointsize; i++, cur += STEP)
        {
            Mat c_coord = (Mat_<double>(3,1) << cur, 0.0, 0.0);
            Mat w_coord = qing_cam_coord_to_world_coord(rotation, translation, c_coord);    //cam_coord to world_coord

            double * wdata = (double *)w_coord.ptr<double>(0);
            m_xaxis_points.push_back(Vec3f(wdata[0], wdata[1], wdata[2]));
        }

        //y-axis
        cur = 0.0;
        for(int i = 0; i < ypointsize; i++, cur += STEP)
        {
            Mat c_coord = (Mat_<double>(3,1) << 0.0, cur, 0.0);
            Mat w_coord = qing_cam_coord_to_world_coord(rotation, translation, c_coord);    //cam_coord to world_coord

            double * wdata = (double *)w_coord.ptr<double>(0);
            m_yaxis_points.push_back(Vec3f(wdata[0], wdata[1], wdata[2]));
        }

        //z-axis
        cur = 0.0;
        for(int i = 0; i < zpointsize; i++, cur += STEP)
        {
            Mat c_coord = (Mat_<double>(3,1) << 0.0, 0.0, cur);
            Mat w_coord = qing_cam_coord_to_world_coord(rotation, translation, c_coord);    //cam_coord to world_coord

            double * wdata = (double *)w_coord.ptr<double>(0);
            m_zaxis_points.push_back(Vec3f(wdata[0], wdata[1], wdata[2]));
        }
    }
    cout << "x-axis point size: " << m_xaxis_points.size() << endl;
    cout << "y-axis point size: " << m_yaxis_points.size() << endl;
    cout << "z-axis point size: " << m_zaxis_points.size() << endl;
}

void Qing_Extrinsics_Visualizer::visualizer_by_ply(const string& plyname)
{
    calc_axes_points();
    vector<Vec3f> all_points(0);
    vector<Vec3f> all_colors(0);

    int totalsize = m_xaxis_points.size() + m_yaxis_points.size() + m_zaxis_points.size();
    all_points.reserve(totalsize);
    all_colors.reserve(totalsize);

    //x-axis
    std::copy(m_xaxis_points.begin(), m_xaxis_points.end(), back_inserter(all_points));
    for(int i = 0; i < m_xaxis_points.size(); ++i) {
        all_colors.push_back(Vec3f(255.f, 0.f, 0.f));
    }

    //y-axis
    std::copy(m_yaxis_points.begin(), m_yaxis_points.end(), back_inserter(all_points));
    for(int i = 0; i < m_yaxis_points.size(); ++i) {
        all_colors.push_back(Vec3f(0.f, 255.f, 0.f));
    }

    //z-axis
    std::copy(m_zaxis_points.begin(), m_zaxis_points.end(), back_inserter(all_points));
    for(int i = 0; i < m_zaxis_points.size(); ++i) {
        all_colors.push_back(Vec3f(0.f, 0.f, 255.f));
    }

    qing_write_point_color_ply(plyname, all_points, all_colors);
}

void Qing_Extrinsics_Visualizer::sfm_triangulate_chessboard(const string calibfolder)
{
    m_chessboard_points.clear();
    m_chessboard_points.resize(m_cam_num/2);
    for(int i = 0; i < m_cam_num - 1; i += 2)
    {
        m_chessboard_points[i/2].clear();
        sfm_triangulate_chessboard(calibfolder, i+0, i+1, m_chessboard_points[i/2]);
        cout << "finish all frames' triangulation.." << m_chessboard_points[i/2].size() << " frames." << endl;
    }
}

void Qing_Extrinsics_Visualizer::sfm_triangulate_chessboard(const string calibfolder, const int camidx0 , const int camidx1, vector<vector<Vec3f> >& total_chessboard)
{
    string cam0 = m_cam_names[camidx0].substr(0,1) + m_cam_names[camidx0].substr(2);
    string cam1 = m_cam_names[camidx1].substr(0,1) + m_cam_names[camidx1].substr(2);
    string camfolder0 = calibfolder + "/" + cam0;
    string camfolder1 = calibfolder + "/" + cam1;
    string calibfile0 = calibfolder + "/imagelist_" + cam0 + ".txt";
    string calibfile1 = calibfolder + "/imagelist_" + cam1 + ".txt";

    vector<string> imagenames0(0);
    vector<string> imagenames1(0);
    qing_read_txt(calibfile0, imagenames0);
    qing_read_txt(calibfile1, imagenames1);

    if(imagenames0.size() != imagenames1.size())
    {
        cerr << "invalid stereo rigs " << cam0 << cam1 << endl;
        return ;
    }

    string outdir = "./" + cam0 + cam1 ;
    qing_create_dir(outdir);
    qing_set_chessboard_size(cam0, m_board_size);

    // two projection matrix
    Mat P0 = m_cam_intrinsics[camidx0] * m_cam_pmatrices[camidx0];
    Mat P1 = m_cam_intrinsics[camidx1] * m_cam_pmatrices[camidx1];
    P0.convertTo(P0, CV_32FC1);
    P1.convertTo(P1, CV_32FC1);
    cout << cam0 << endl << P0 << endl;
    cout << cam1 << endl << P1 << endl;

    // start to triangulate each chessboard frame
    int nframes = imagenames0.size();
    for(int j = 0; j < nframes; ++j)
    {
        string imgname0 = camfolder0 + "/" + imagenames0[j];
        string imgname1 = camfolder1 + "/" + imagenames1[j];

        vector<Point2f> corners0(0), corners1(0);
        vector<Vec3f> chessboard(0);
        if( qing_extract_corners(imgname0, m_board_size, corners0) && qing_extract_corners(imgname1, m_board_size, corners1) && corners0.size() == corners1.size() )
        {
            cout << "start to triangulate 3D points of frame " << j << " ... " << corners0.size() << ", " << corners1.size() << " \t ";
            int ptsize = corners0.size();

            //transfer corners vector to correct data format for triangulatePoints
            cv::Mat points0(2, ptsize, CV_32FC1);
            cv::Mat points1(2, ptsize, CV_32FC1);
            float * pptr0 = (float *)points0.ptr<float>(0);
            float * pptr1 = (float *)points1.ptr<float>(0);
            for(int k = 0; k < ptsize; ++k)
            {
                pptr0[0*ptsize + k] = corners0[k].x;
                pptr0[1*ptsize + k] = corners0[k].y;
                pptr1[0*ptsize + k] = corners1[k].x;
                pptr1[1*ptsize + k] = corners1[k].y;
            }

            cv::Mat points3D(4, ptsize, CV_32FC1);
            cv::triangulatePoints(P0, P1, points0, points1, points3D);

            //transfer homogenous points to non-homogenous points
            float * pptr3d = (float *)points3D.ptr<float>(0);
            for(int k = 0; k < ptsize; ++k)
            {
                double wx = pptr3d[0 * ptsize + k];
                double wy = pptr3d[1 * ptsize + k];
                double wz = pptr3d[2 * ptsize + k];
                double ww = pptr3d[3 * ptsize + k];
                chessboard.push_back( Vec3f(wx/ww, wy/ww, wz/ww) );
            }
            cout << "triangulations done..\t"  << chessboard.size() << " 3d points....\t";
            total_chessboard.push_back(chessboard);
        }
        else
        {
            cerr << "failed to extract corners in frame pair " << j << " in " << cam0 + cam1<< endl;
            continue;
        }

        string savefile = outdir + "/chessboard_" + int2FormatString(j, 4, '0') + ".xyz";
        cout << "saving " << savefile << endl;
        qing_write_xyz(savefile, chessboard);
    }
}

void Qing_Extrinsics_Visualizer::eval_sfm_triangulation(const string evalfile)
{
    fstream fout(evalfile.c_str(), ios::out);
    if(fout.is_open() == false)
    {
        cerr << "failed to open " << evalfile << endl;
        return ;
    }

    fout << " stereo_name \t  " << " max_recons_err \t  " << " avg_recons_err \t  " << endl;
    for(int i = 0; i < m_cam_num -1; i+=2)
    {
        string cam0 = m_cam_names[i+0].substr(0,1) + m_cam_names[i+0].substr(2);
        string cam1 = m_cam_names[i+1].substr(0,1) + m_cam_names[i+1].substr(2);
        string stereoname = cam0 + cam1;
        int stereoidx = i/2;

        double avg_recons_err = 0.0f;
        double max_recons_err = 0.0f;

        vector<vector<Vec3f> >& chessboard = m_chessboard_points[stereoidx];
        int nframes = chessboard.size();
        for(int f = 0; f < nframes ; ++f)   //frame idx
        {
            qing_calc_recons_error(chessboard[f], m_board_size, avg_recons_err, max_recons_err);
        }

        avg_recons_err /= (nframes * ( ( m_board_size.width - 1 ) * m_board_size.height +
                                       m_board_size.width * (m_board_size.height - 1) ) );

        fout << stereoname << " \t " << setw(10) << setprecision(6) << max_recons_err << '\t'
             << setw(10) << setprecision(6)  << avg_recons_err << endl;
    }
    fout.close();
}
