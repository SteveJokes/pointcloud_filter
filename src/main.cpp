#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <assert.h>
#include "dmcam.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <iostream>
#include "img_filter.h"

#define FRAME_SIZE (320*240*4*2)
#define FRAME_BUF_FCNT 1
static dmcam_dev_t *dev;
int user_data;
using namespace std;
void pc_to_pcl(int pcl_len,float *pc,pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud2);
int main (int argc, char** argv)
{
    int dev_cnt;
    dmcam_dev_t dev_list[4];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ror_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr vg_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    // 填入点云数据
    //pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloud);

    /* list all dmcam devices (max = 4) */
    dev_cnt = dmcam_dev_list(dev_list, 4);

    std::cout<<dev_cnt<<" dmcam device found\n"<<endl;
    if (dev_cnt == 0)
        std::cout<<"device not found!\n"<<endl;

    /* open the first present devic */
    /*   you can also use NULL for dmcam_dev_open to open the
     *   first device e.g. dev =  dmcam_dev_open(NULL); */
    dev = dmcam_dev_open(&dev_list[0]);
    if (!dev) {
        std::cout<<" open device failed\n"<<endl;
    }

    /* set frame buffer to use internal buffer */
    dmcam_cap_set_frame_buffer(dev, NULL, FRAME_SIZE * FRAME_BUF_FCNT);

    /* disable error callback for capturing */
    dmcam_cap_set_callback_on_error(dev, NULL);

    /* disable frame ready callback */
    dmcam_cap_set_callback_on_frame_ready(dev, NULL);

    {
        dmcam_frame_t fbuf_info;
        int n;
        int total_fr = 0;
        int fr_cnt;
        int rd_fr_once = 1;
        //uint8_t *fbuf = (uint8_t *)malloc(FRAME_SIZE * rd_fr_once);
        uint8_t *fbuf=new uint8_t[FRAME_SIZE*rd_fr_once];
        assert(fbuf);

        /* start capturing frames */
        dmcam_cap_start(dev);

        {
            int w, h;

            fr_cnt = dmcam_cap_get_frames(dev, rd_fr_once, fbuf, FRAME_SIZE * rd_fr_once, &fbuf_info);
            if (fr_cnt < rd_fr_once) { // less frames means sampling is stopped.
                std::cout<<"capturing is stopped due to some error!\n"<<endl;
            }
            /* proc frames in fbuf */
            std::cout<<"proc frames ....\n"<<endl;
            w = fbuf_info.frame_info.width;
            h = fbuf_info.frame_info.height;

            /* decode one frame to distance */
            {
                int dist_len = fbuf_info.frame_info.width * fbuf_info.frame_info.height;
                float *dist;
                dist=new float[dist_len];
                int calc_len = dmcam_frame_get_distance(dev, dist, dist_len, fbuf, fbuf_info.frame_info.frame_size, &fbuf_info.frame_info);

                if (calc_len == 0) {
                    std::cout<<"the frame is not valid !\n"<<endl;
                }
                /*plane fit filter*/
                img_plane_mf_sqr3(dist,dist);
                /* decode one frame to pcl */
                {
                    int pcl_len = fbuf_info.frame_info.width * fbuf_info.frame_info.height;
                    float *pcl_data = new float[pcl_len * 3];
                    //int calc_len = dmcam_frame_get_pcl_xyzd(dev, pcl_data, pcl_len, dist, dist_len, w, h, true, NULL);
                    int calc_len = dmcam_frame_get_pcl(dev, pcl_data, pcl_len*3, dist, dist_len, w, h, NULL);

                    pc_to_pcl(pcl_len,pcl_data,cloud,cloud2);
                    /* process distance data */
                    std::cout<<"proc pcl data ....\n"<<endl;

                    delete pcl_data;
                }
                delete dist;
            }

        }
        free(fbuf);

        /* stop capturing */
        dmcam_cap_stop(dev);
    }

    /* close device */

    dmcam_dev_close(dev);
    /*filter*/
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud2);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1);
    sor.filter(*sor_filtered);
    //sor_filtered=cloud2;

    pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
    ror.setInputCloud(cloud2);
    ror.setRadiusSearch(0.8);
    ror.setMinNeighborsInRadius(2);
    sor.filter(*ror_filtered);

    pcl::VoxelGrid<pcl::PointXYZI> vgf;
    vgf.setInputCloud(cloud2);
    vgf.setLeafSize (0.01f, 0.01f, 0.01f);
    vgf.filter(*vg_filtered);

    /*visualization*/
    pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
    viewer.initCameraParameters();
    int v1(0);
    viewer.createViewPort(0.0, 0.5, 0.5, 1, v1);
    viewer.setBackgroundColor(1.0, 0.5, 1.0,v1);
    viewer.addText("Cloud before filtering", 10, 10,"v1 test", v1);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> cloud_color(cloud2, "intensity");
    viewer.addPointCloud<pcl::PointXYZI>(cloud2,cloud_color,"cloud2", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");

    int v2(0);
    viewer.createViewPort(0.5, 0.5, 1.0, 1, v2);
    viewer.setBackgroundColor(1.0, 0.5, 1.0,v2);
    viewer.addText("Cloud after sor filtering", 10, 10, "v2 test", v2);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> sorfilter_color(sor_filtered, "intensity");
    viewer.addPointCloud<pcl::PointXYZI>(sor_filtered, sorfilter_color, "sor_filtered", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sor_filtered");

    int v3(0);
    viewer.createViewPort(0, 0, 0.5, 0.5, v3);
    viewer.setBackgroundColor(1.0, 0.5, 1.0,v3);
    viewer.addText("Cloud after ror filtering", 10, 10, "v3 test", v3);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> rorfilter_color(ror_filtered, "intensity");
    viewer.addPointCloud<pcl::PointXYZI>(ror_filtered, rorfilter_color, "ror_filtered", v3);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ror_filtered");

    int v4(0);
    viewer.createViewPort(0.5, 0, 1, 0.5, v4);
    viewer.setBackgroundColor(1.0, 0.5, 1.0,v4);
    viewer.addText("Cloud after vg filtering", 10, 10, "v4 test", v4);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> vgfilter_color(ror_filtered, "intensity");
    viewer.addPointCloud<pcl::PointXYZI>(vg_filtered, vgfilter_color, "vg_filtered", v4);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "vg_filtered");

    while (!viewer.wasStopped())
    {
        //在此处可以添加其他处理
        user_data++;
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }



    return (0);
}
void pc_to_pcl(int pcl_len,float *pc,pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud2)
{
    for (int m = 0; m < pcl_len; m++) {
        pCloud->points.push_back(pcl::PointXYZ(pc[3 * m], pc[3 * m + 1], pc[3 * m + 2]));
    }
    pCloud->height = 1;
    pCloud->width = pCloud->points.size();
    pCloud->is_dense = false;
    pcl::copyPointCloud(*pCloud, *pCloud2);
}
