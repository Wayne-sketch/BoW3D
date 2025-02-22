#include "Frame.h"
#include <thread>

namespace BoW3D
{
    //静态（全局？）变量要在这里初始化
    long unsigned int Frame::nNextId = 0;
   
    Frame::Frame(LinK3D_Extractor* pLink3dExtractor, pcl::PointCloud<pcl::PointXYZ>::Ptr pLaserCloudIn):mpLink3dExtractor(pLink3dExtractor)
    {
        mnId = nNextId++; 

        (*mpLink3dExtractor)(pLaserCloudIn, mvAggregationKeypoints, mDescriptors, mClusterEdgeKeypoints);
    }

                
}