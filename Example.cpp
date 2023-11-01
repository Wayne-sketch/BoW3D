#include <ros/ros.h>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>
#include <pcl/filters/extract_indices.h>

#include <sstream>
#include <iomanip>
#include "LinK3D_Extractor.h"
#include "BoW3D.h"


using namespace std;
using namespace BoW3D;


//Parameters of LinK3D
//雷达扫描线数
int nScans = 64; //Number of LiDAR scan lines
//雷达扫描周期
float scanPeriod = 0.1; 
//最小点云距离范围，距离原点小于该阈值的点将被删除
float minimumRange = 0.1;
//判断区域内某点和聚类点均值距离，以及在x，y轴上的距离
float distanceTh = 0.4;
//描述子匹配所需的最低分数阈值 ，描述子匹配分数低于此分数的两个关键点不匹配
int matchTh = 6;

//Parameters of BoW3D
//比率阈值。
float thr = 3.5;
//频率阈值。
int thf = 5;
//每帧添加或检索的特征数量。
int num_add_retrieve_features = 5;

//Here, the KITTI's point cloud with '.bin' format is read.
vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file;
    lidar_data_file.open(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    if(!lidar_data_file)
    {
        cout << "Read End..." << endl;
        exit(-1);
    }

    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "BoW3D");
    ros::NodeHandle nh;  


    /*Please replace the dataset folder path with the path in your computer. KITTI's 00, 02, 05, 06, 07, 08 have loops*/
    string dataset_folder;
    dataset_folder = "/home/cuiyunge/dataset/velodyne/"; //The last '/' should be added 

    //LinK3D提取器
    BoW3D::LinK3D_Extractor* pLinK3dExtractor = new BoW3D::LinK3D_Extractor(nScans, scanPeriod, minimumRange, distanceTh, matchTh); 
    //BoW3D词袋
    BoW3D::BoW3D* pBoW3D = new BoW3D::BoW3D(pLinK3dExtractor, thr, thf, num_add_retrieve_features);

    //点云索引
    size_t cloudInd = 0;
    //雷达频率，因为是仿真，所以自己设定频率，按频率读取数据
    ros::Rate LiDAR_rate(10); //LiDAR frequency 10Hz
    while (ros::ok())
    {              
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << std::setfill('0') << std::setw(6) << cloudInd << ".bin";
        vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        //读取点云数据
        for(std::size_t i = 0; i < lidar_data.size(); i += 4)
        {            
            pcl::PointXYZ point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
    
            current_cloud->push_back(point);
        }

        //创建点云帧
        Frame* pCurrentFrame = new Frame(pLinK3dExtractor, current_cloud); 
            
        //Id从0开始算，前两帧更新BoW3D词袋
        if(pCurrentFrame->mnId < 2)
        {
            pBoW3D->update(pCurrentFrame);  
        }
        else
        {                
            int loopFrameId = -1;
            Eigen::Matrix3d loopRelR;
            Eigen::Vector3d loopRelt;

            clock_t start, end;
            double time;       
            start = clock();

            //在3D词袋模型中检索与当前帧相似的帧
            pBoW3D->retrieve(pCurrentFrame, loopFrameId, loopRelR, loopRelt); 

            end = clock();
            time = ((double) (end - start)) / CLOCKS_PER_SEC;
            
            //更新词袋
            pBoW3D->update(pCurrentFrame);               

            //输出检测结果
            if(loopFrameId == -1)
            {
                cout << "-------------------------" << endl;
                cout << "Detection Time: " << time << "s" << endl;
                cout << "Frame" << pCurrentFrame->mnId << " Has No Loop..." << endl;
            }
            else
            {
                cout << "--------------------------------------" << endl;
                cout << "Detection Time: " << time << "s" << endl;
                cout << "Frame" << pCurrentFrame->mnId << " Has Loop Frame" << loopFrameId << endl;
                
                cout << "Loop Relative R: " << endl;
                cout << loopRelR << endl;
                                
                cout << "Loop Relative t: " << endl;                
                cout << "   " << loopRelt.x() << " " << loopRelt.y() << " " << loopRelt.z() << endl;
            }
        }                       
        
        cloudInd ++;

        ros::spinOnce();
        LiDAR_rate.sleep();
    }

    return 0;
}

