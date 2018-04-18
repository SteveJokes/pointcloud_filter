//#include <iostream>
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

#define FRAME_SIZE (320*240*4 * 2)
#define FRAME_BUF_FCNT 10
static dmcam_dev_t *dev;
int user_data;
int main (int argc, char** argv)
{
    int dev_cnt;
    dmcam_dev_t dev_list[4];

    /* list all dmcam devices (max = 4) */
    dev_cnt = dmcam_dev_list(dev_list, 4);

    printf(" %d dmcam device found\n", dev_cnt);
    if (dev_cnt == 0)
        goto FINAL;

    /* open the first present devic */
    /*   you can also use NULL for dmcam_dev_open to open the
     *   first device e.g. dev =  dmcam_dev_open(NULL); */
    dev = dmcam_dev_open(&dev_list[0]);
    if (!dev) {
        printf(" open device failed\n");
        goto FINAL;
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
        int rd_fr_once = 10;
        uint8_t *fbuf = malloc(FRAME_SIZE * rd_fr_once);

        assert(fbuf);

        /* start capturing frames */
        dmcam_cap_start(dev);

        for (n = 0; n < 10; n++) {
            int w, h;

            /* get 20 frames */
            fr_cnt = dmcam_cap_get_frames(dev, rd_fr_once, fbuf, FRAME_SIZE * rd_fr_once, &fbuf_info);
            if (fr_cnt < rd_fr_once) { // less frames means sampling is stopped.
                printf("capturing is stopped due to some error!\n");
                break;
            }
            total_fr += fr_cnt;
            printf("get %d frames: [%u, %ux%u, %u]\n",
                   fr_cnt, fbuf_info.frame_info.frame_idx,
                   fbuf_info.frame_info.width, fbuf_info.frame_info.height, fbuf_info.frame_info.frame_format);


            /* stop capture if get enough frames */
            if (total_fr > 100) {
                printf("get enough frames, we stop\n");
                break;
            }


            /* proc frames in fbuf */
            printf("proc frames ....\n");

            w = fbuf_info.frame_info.width;
            h = fbuf_info.frame_info.height;

            /* decode one frame to distance */
            {
                int dist_len = fbuf_info.frame_info.width * fbuf_info.frame_info.height;
                float *dist = malloc(sizeof(float) * dist_len);
                int calc_len = dmcam_frame_get_distance(dev, dist, dist_len, fbuf, fbuf_info.frame_info.frame_size, &fbuf_info.frame_info);

                if (calc_len == 0) {
                    printf("the frame is not valid !\n");
                }
                (void)calc_len;


                /* decode one frame to pcl */
                {
                    int pcl_len = fbuf_info.frame_info.width * fbuf_info.frame_info.height;
                    float *pcl = malloc(sizeof(float) * pcl_len * 4);
                    int calc_len = dmcam_frame_get_pcl_xyzd(dev, pcl, pcl_len, dist, dist_len, w, h, true, NULL);

                    /* process distance data */
                    printf("proc pcl data ....\n");
                    (void)calc_len;

                    free(pcl);
                }
                free(dist);
            }
            /* decode one frame to gray */
            {
                int gray_len = fbuf_info.frame_info.width * fbuf_info.frame_info.height;
                float *gray = malloc(sizeof(float) * gray_len);
                int calc_len = dmcam_frame_get_gray(dev, gray, gray_len, fbuf, fbuf_info.frame_info.frame_size, &fbuf_info.frame_info);

                assert(calc_len == gray_len);
                printf("proc gray data ....\n");

                /* process gray data */
                (void)calc_len;

                free(gray);
            }

        }
        free(fbuf);

        /* stop capturing */
        dmcam_cap_stop(dev);
    }

    /* close device */
    dmcam_dev_close(dev);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ror_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    // 填入点云数据
    pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloud);


//    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//        << " data points (" << pcl::getFieldsList(*cloud) << ").";

    // 创建滤波器对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1);
    sor.filter(*sor_filtered);

    pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(10);
    ror.setMinNeighborsInRadius(2);
    ror.filter(*ror_filtered);
//    std::cout<<"step 3"<<endl;

//    std::cerr << "PointCloud after filtering: " << sor_filtered->width * sor_filtered->height
//        << " data points (" << pcl::getFieldsList(*sor_filtered) << ").";


    pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
    viewer.initCameraParameters();

    int v1(0);
    viewer.createViewPort(0.0, 0.5, 0.5, 1, v1);
    viewer.setBackgroundColor(1.0, 0.5, 1.0,v1);
    viewer.addText("Cloud before filtering", 10, 10,"v1 test", v1);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> cloud_color(cloud, "intensity");
    viewer.addPointCloud<pcl::PointXYZI>(cloud, cloud_color, "cloud", v1);

    int v2(0);
    viewer.createViewPort(0.5, 0.5, 1.0, 1, v2);
    viewer.setBackgroundColor(1.0, 0.5, 1.0,v2);
    viewer.addText("Cloud after filtering", 10, 10, "v2 test", v2);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> cloud_afterfilter_color(sor_filtered, "intensity");
    viewer.addPointCloud<pcl::PointXYZI>(sor_filtered, cloud_afterfilter_color, "sor_filtered", v2);



    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sor_filtered");



    while (!viewer.wasStopped())
    {
        //在此处可以添加其他处理
        user_data++;
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }



    return (0);
}
