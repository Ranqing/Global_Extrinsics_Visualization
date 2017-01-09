#include "global_extrinsics_visualizer.h"

#include "../../../Qing/qing_string.h"
#include "../../../Qing/qing_basic.h"
#include "../../../Qing/qing_io.h"

void qing_read_sfm_pmatrix(const string filename, Mat& rotation, Mat& translation)
{
    fstream fin(filename.c_str(), ios::in);
    if(fin.is_open() == false)
    {
        cerr << "failed to open " << filename << endl;
        return ;
    }
    cout << filename << endl;

    double rdata[9];    //rotation data
    double tdata[3];    //tranlatio  data

    string line;
    vector<string> words(0);
    while(getline(fin, line))
    {
        if('#' != line[0])  continue;
        words.clear();
        qing_split_a_string_by_space(line.substr(1), words);

        if ("width" == words[0])       {  getline(fin, line);  continue;  }         //width height
        if ("focal" == words[0])       {  getline(fin, line);  continue;  }         //focal centerx centery
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

    rotation.create(3, 3, CV_64FC1);
    memcpy(rotation.data, rdata, sizeof(double)*9);
    translation.create(3,1,CV_64FC1);
    memcpy(translation.data, tdata, sizeof(double)*3);
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
}


//known: openmvg data: p_world_coord = rotation.inv() * p_cam_coord + translation
//
void Qing_Extrinsics_Visualizer::read_sfm_extrinsics()
{
    for(int i = 0; i < m_cam_num; ++i)
    {
        string filename = m_sfm_folder + "/" + m_cam_names[i] + ".txt";
        Mat rotation, translation;
        qing_read_sfm_pmatrix(filename, rotation, translation);

        //openmvg:
        //p_world_coord = rotation.inv() * p_cam_coord + translation <==> p_cam_coord = rotation * p_world_coord - rotation * translation
        m_rotations[i] = rotation;
        m_translations[i] = -(rotation * translation);

# if 1
        cout << m_cam_names[i] <<" : " << endl ;
        cout << m_rotations[i] << endl << m_translations[i] << endl;
# endif
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
        cout << m_cam_names[i] << " : " << m_cam_poses[i].t() << endl;

        delete[] center;
    }
}

void Qing_Extrinsics_Visualizer::calc_camera_orientations()
{
   cout << "here is calc_camera_orientations" << endl;

   for(int i = 0; i < m_cam_num; ++i)
   {
       Mat c_coord = (Mat_<double>(3,1) << 1.0, 0.0, 0.0);      //camera_coord
       Mat w_coord = m_rotations[i].t() * (c_coord - m_translations[i]);

       m_cam_orientations[i] = w_coord.clone();

       cout << m_cam_names[i] << " orientation: " << m_cam_orientations[i].t() << endl;
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
