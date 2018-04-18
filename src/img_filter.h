/**
 * @file    img_filter.c
 * @author  YRD
 * @version 1.0
 * @date    2016-8-30
 * @brief   图像滤波运算函数
 * @details 本文件包括了时域和空间域的滤波算法
*/


#ifndef __IMG_FILTER_H__
#define __IMG_FILTER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    /** 
 * @fn              float *img_fir_cross(float *img_out, float *img_in, float *coff)
 * @brief           使用十字滤波模板的图像滤波
 * @param [in]      float *img_in：指针，指向待滤波图像
 * @param [in]      float *coff：指针，指向5个滤波系数
 * @param [out]     float *img_out：指针，指向的空间存放图像运算结果
 * @retval          float *：和img_out相同
 */ 
float *img_fir_cross(float *img_out, float *img_in, float *coff);

/** 
 * @fn              float *img_fir_cross_sa(float *img_out, float *img_in, float *coff)
 * @brief           使用十字滤波模板的图像滤波（原址操作），功能同img_fir_cross，但原址运算实现
 *                  注意：滤波输出图像的最外圈边沿（1层像素）是无效数据
 * @param [inout]   float *img_inout：指针，指向待滤波图和图像运算结果
 * @param [in]      float *coff：指针，指向5个滤波系数
 * @param [inout]   环形缓冲器，存放3行数据
 * @retval          float *：和img_inout相同
 */ 
float *img_fir_cross_sa(float *img_inout, float *coff, struct ring_buf_f32_s *rbuf);

/** 
 * @fn              float *img_fir_sqr3(float *img_out, float *img_in, float *coff)
 * @brief           使用3x3滤波模板的图像滤波
 * @param [in]      float *img_in：指针，指向待滤波图像
 * @param [in]      float *coff：指针，指向9个滤波系数
 * @param [out]     float *img_out：指针，指向的空间存放图像运算结果
 * @retval          float *：和img_out相同
 */ 
float *img_fir_sqr3(float *img_out, float *img_in, float *coff);

/** 
 * @fn              float *img_fir_sqr3_sa(float *img_inout, float *coff, struct ring_buf_f32_s *rbuf)
 * @brief           使用3x3滤波模板的图像滤波（原址操作），功能同img_fir_sqr3，但通过原址操作实现
 *                  注意：滤波输出图像的最外圈边沿（1层像素）是无效数据
 * @param [inout]   float *img_inout：指针，指向待滤波图和图像运算结果
 * @param [in]      float *coff：指针，指向9个滤波系数
 * @param [inout]   ring_buf_f32_s *rbuf环形缓冲器，存放3行数据
 * @retval          float*：和img_inout相同
 */ 
float *img_fir_sqr3_sa(float *img_inout, float *coff, struct ring_buf_f32_s *rbuf);

/** 
 * @fn              float *img_iir_t(float *img_out,float *img_in, float alpha)
 * @brief           1阶IIR图像序列的时间滤波，使用有损积分器结构
 * @param [in]      float *img_in：指针，指向待滤波图像
 * @param [in]      float alpha：滤波系数（遗忘因子）0~1，越接近1，滤波器带宽越小
 * @param [inout]   float *img_inout：指针，指向空间存放先前滤波结果和新的滤波结果
 * @retval          float *：和img_inout相同
 */
float *img_iir_t(float *img_inout,float *img_in, float alpha);

/** 
 * @fn              float *img_fir3_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2, float *coff)
 * @brief           图像序列的FIR时间滤波，使用前2帧和当前帧数据
 * @param [in]      float *img0，img1，img2为历史图像帧(指针)，img0对应最老图像，img2对应最新图像
 * @param [in]      float *coff：指针，指向滤波加权系数数组，*coff对应img_in0，*(coff+1)对应img_in1,*(coff+2)对应img_in2
 * @param [out]     float *img_out：指针，指向空间存放滤波结果
 * @retval          float *：和img_out相同
 */
float *img_fir3_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2, float *coff);

/** 
 * @fn              float *img_fir3_t(float *img_out, float *img_in0, float *img_in1, float *img_in2, float *coff)
 * @brief           图像序列的FIR时间滤波，使用前2帧和当前帧数据
 * @param [in]      float *img_buf：为历史图像帧(指针)，指向区域连续存放最近2帧图像
 * @param [in]      float *img_in：指针，指向最新输入图像
 * @param [in]      float *coff：指针，指向滤波加权系数数组，*coff对应img_in0，*(coff+1)对应img_in1,*(coff+2)对应img_in2
 * @param [out]     float *img_out：指针，指向的空间存放滤波结果
 * @param [inout]   int *state：指针，指向滤波状态变量，初始值需设为0，指向的内容在运行后被修改
 * @retval          float *：和img_out相同
 */
float *img_fir3_t(float *img_out, float *img_buf, float *img_in, float *coff, int *state);

/** 
 * @fn              float *img_mid3_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2)
 * @brief           图像序列的中值滤波,使用前2帧和当前帧数据
 * @param [in]      img0，img1，img2为历史图像帧(指针)，img0对应最老图像，img2对应最新（当前）图像
 * @param [out]     img_out：指针，指向空间存放滤波结果
 * @retval          float *：和img_out相同
 */
float *img_mid3_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2);

/** 
 * @fn              float *img_mid3_t(float *img_out, float *img_buf, float *img_in, int *state)
 * @brief           图像序列的中值滤波,使用前2帧和当前帧数据
 * @param [in]      float *img_buf：历史图像帧(指针)，连续存放最近2帧图像
 * @param [in]      float *img_in：指针，指向最新输入图像
 * @param [out]     float *img_out：指针，指向空间存放滤波结果
 * @param [inout]   int state：指针，指向滤波状态变量，初始值需设为0，指向的内容在运行后被修改
 * @retval          float*：和img_out相同
 */
float *img_mid3_t(float *img_out, float *img_buf, float *img_in, int *state);

/** 
 * @fn              float *img_mid5_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2)
 * @brief           图像序列的中值滤波,使用前4帧和当前帧数据
 * @param [in]      img0，img1，img2, img3, img4为历史图像帧(指针)，img0对应最老图像，img4对应最新（当前）图像
 * @param [out]     img_out：指针，指向空间存放滤波结果
 * @retval          float *：和img_out相同
 */
float *img_mid5_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2, float *img_in3, float *img_in4);

/** 
 * @fn              float *img_mid5_t(float *img_out, float *img_buf, float *img_in, int *state)
 * @brief           图像序列的中值滤波,使用前4帧和当前帧数据
 * @param [in]      float *img_buf：历史图像帧(指针)，连续存放最近4帧图像
 * @param [in]      float *img_in：指针，指向最新输入图像
 * @param [out]     float *img_out：指针，指向空间存放滤波结果
 * @param [inout]   int state：指针，指向滤波状态变量(最老的图像帧在img_buf中的位置），初始值需设为0，指向的内容在运行后被修改
 * @retval          float*：和img_out相同
 */
float *img_mid5_t(float *img_out, float *img_buf, float *img_in, int *state);

/** 
 * @fn              float *img_mid5_avg_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2)
 * @brief           图像序列的平均中值滤波,使用前4帧和当前帧数据，5帧数据中对应位置像素值，去除最大最小值后平均
 * @param [in]      img0，img1，img2, img3, img4为历史图像帧(指针)，img0对应最老图像，img4对应最新（当前）图像
 * @param [out]     img_out：指针，指向空间存放滤波结果
 * @retval          float *：和img_out相同
 */
float *img_minmax_avg5_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2, float *img_in3, float *img_in4);

/** 
 * @fn              float *img_minmax_avg5_t(float *img_out, float *img_buf, float *img_in, int *state)
 * @brief           图像序列的平均中值滤波,使用前4帧和当前帧数据，，5帧数据中对应位置像素，去除最大最小值后平均
 * @param [in]      float *img_buf：历史图像帧(指针)，连续存放最近4帧图像
 * @param [in]      float *img_in：指针，指向最新输入图像
 * @param [out]     float *img_out：指针，指向空间存放滤波结果
 * @param [inout]   int state：指针，指向滤波状态变量(最老的图像帧在img_buf中的位置），初始值需设为0，指向的内容在运行后被修改
 * @retval          float*：和img_out相同
 */
float *img_minmax_avg5_t(float *img_out, float *img_buf, float *img_in, int *state);

/** 
 * @fn              float *img_mid_cross(float *img_out, float *img_in)
 * @brief           图像空间域中值滤波，使用十字滤波模板（共5个像素）。注意：滤波输出图像的最外圈边沿（1层像素）是无效数据
 * @param [in]      float *img_in：指针，指向待滤波图像
 * @param [out]     float *img_out：指针，指向的空间存放图像运算结果
 * @retval          float *：和img_out相同
 */ 
float *img_mid_cross(float *img_out, float *img_in);

/** 
 * @fn              float *img_mid_cross_sa(float *img_inout, float *img_in)
 * @brief           图像空间域中值滤波（原址运算），使用十字滤波模板（共5个像素）。注意：滤波输出图像的最外圈边沿（1层像素）是无效数据
 * @param [in]      float *img_inout：指针，指向待滤波图像和图像运算结果
 * @param [inout]   ring_buf_f32_s *rbuf：环形缓冲器，存放3行数据
 * @param [inout]   float *img_inout：指针，指向的空间存放图像运算结果
 * @retval          float *：和img_inout相同
 */ 
float *img_mid_cross_sa(float *img_inout, float *img_in, struct ring_buf_f32_s *rbuf);

/** 
 * @fn              float *img_iir_sos(float *img_out, float *img_in, float img_st0, float *img_st1, float *coff)
 * @brief           使用2阶IIR滤波器的图像时域滤波，
 * @param [in]      float *img_in：指针，指向待滤波图像
 * @param [inout]   float *img_st1,*img_st2：指针，指向滤波状态数据（图像）
 * @param [in]      float *coff：指针，指向滤波加权系数数组{b1,b2,b3,a2,a3,sc}
 * @param [out]     float *img_out：指针，指向的空间存放图像运算结果
 * @retval          float *：和img_inout相同
 */ 
float *img_iir_sos(float *img_out, float *img_in, float *img_st1, float *img_st2, float *coff);

/** 
 * @fn              float *img_weighted_iir(float *img_out, float *img_in, float *img_in_w_avg, float *img_w, float *img_w_avg, float alpha)
 * @brief           图像加权IIR平均
 * @param [in]      float *img_in：指针，指向待滤波图像
 * @param [in]      float *img_w：指针，指向加权数据
 * @param [inout]   float *img_in_w_avg：指针，指向滤波状态数据,内容在该函数运行后更新
 * @param [inout]   float *img_w_avg：指针，指向滤波状态数据,内容在该函数运行后更新
 * @param [in]      float alpha：滤波器遗忘因子(0~1)越接近0，越“健忘”
 * @param [out]     float *img_out：指针，指向的空间存放图像运算结果
 * @retval          float *：和img_inout相同
 */ 
float *img_weighted_iir(float *img_out, float *img_in, float *img_in_w_avg, float *img_w, float *img_w_avg, float alpha);

/** 
 * @fn              float *img_max3_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2)
 * @brief           从3帧图像序列中找到的每个位置的像素最大值,使用前2帧和当前帧数据
 * @param [in]      float* img0，img1，img2为历史图像帧(指针)，img0对应最老图像，img2对应最新（当前）图像
 * @param [out]     float* img_out：指针，指向空间存放滤波结果
 * @retval          float *：和img_out相同
 */
float *img_max3_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2);

/** 
 * @fn              float *img_max3_t(float *img_out, float *img_buf, float *img_in, int *state)
 * @brief           从连续输入的最近3帧图像序列中找到的每个位置的像素最大值,使用前2帧和当前帧数据
 * @param [in]      float *img_buf：历史图像帧(指针)，连续存放最近2帧图像
 * @param [in]      float *img_in：指针，指向最新输入图像
 * @param [out]     float *img_out：指针，指向空间存放滤波结果
 * @param [inout]   int state：指针，指向滤波状态变量，初始值需设为0，指向的内容在运行后被修改
 * @retval          float*：和img_out相同
 */
float *img_max3_t(float *img_out, float *img_buf, float *img_in, int *state);

/** 
 * @fn              float *img_max5_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2)
 * @brief           从连续输入的最近5帧图像序列中找到的每个位置的像素最大值,使用前4帧和当前帧数据
 * @param [in]      float *img0，img1，img2, img3, img4为历史图像帧(指针)，img0对应最老图像，img4对应最新（当前）图像
 * @param [out]     float *img_out：指针，指向空间存放滤波结果
 * @retval          float *：和img_out相同
 */
float *img_max5_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2, float *img_in3, float *img_in4);

/** 
 * @fn              float *img_max5_t(float *img_out, float *img_buf, float *img_in, int *state)
 * @brief           从连续输入的最近5帧图像序列中找到的每个位置的像素最大值,使用前4帧和当前帧数据
 * @param [in]      float *img_buf：历史图像帧(指针)，连续存放最近4帧图像
 * @param [in]      float *img_in：指针，指向最新输入图像
 * @param [out]     float *img_out：指针，指向空间存放滤波结果
 * @param [inout]   int state：指针，指向滤波状态变量(最老的图像帧在img_buf中的位置），初始值需设为0，指向的内容在运行后被修改
 * @retval          float*：和img_out相同
 */
float *img_max5_t(float *img_out, float *img_buf, float *img_in, int *state);

/** 
 * @fn              float *img_min5_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2)
 * @brief           从连续输入的最近5帧图像序列中找到的每个位置的像素最小值,使用前4帧和当前帧数据
 * @param [in]      float *img0，img1，img2, img3, img4为历史图像帧(指针)，img0对应最老图像，img4对应最新（当前）图像
 * @param [out]     float *img_out：指针，指向空间存放滤波结果
 * @retval          float *：和img_out相同
 */
float *img_min5_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2, float *img_in3, float *img_in4);

/** 
 * @fn              float *img_min5_t(float *img_out, float *img_buf, float *img_in, int *state)
 * @brief           从连续输入的最近4帧图像序列中找到的每个位置的像素最小值,使用前4帧和当前帧数据
 * @param [in]      float *img_buf：历史图像帧(指针)，连续存放最近4帧图像
 * @param [in]      float *img_in：指针，指向最新输入图像
 * @param [out]     float *img_out：指针，指向空间存放滤波结果
 * @param [inout]   int state：指针，指向滤波状态变量(最老的图像帧在img_buf中的位置），初始值需设为0，指向的内容在运行后被修改
 * @retval          float*：和img_out相同
 */
float *img_min5_t(float *img_out, float *img_buf, float *img_in, int *state);

/** 
 * @fn              float *img_nnf_sqr3(float *img_out, float *img_in, float *coff)
 * @brief           像素最近邻选择滤波，使用使用3x3滤波模板，如果邻近像素和中心像素差异超过门限则使用空间十字模板（5点）中值滤波
 * @param [in]      float *img_in：指针，指向待滤波图像
 * @param [in]      float th：滤波门限
 * @param [out]     float *img_out：指针，指向的空间存放图像运算结果
 * @retval          float *：和img_out相同
 */ 
float *img_nnf_sqr3(float *img_out, float *img_in, float th);

/** 
 * @fn              float *img_fb_mid5_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2)
 * @brief           图像序列的前向后向选择中值滤波,使用前4帧和当前帧数据       
 * @param [in]      float *img0，img1，img2, img3, img4为历史图像帧(指针)，img0对应最老图像，img4对应最新（当前）图像
 * @param [in]      float th使用前向MID3滤波结果（新数据）的门限
 * @param [out]     float *img_out：指针，指向空间存放滤波结果
 * @retval          float *：和img_out相同
 */
float *img_fb_mid3_t_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2, float *img_in3, float *img_in4, float th);

/** 
 * @fn              float *img_fb_mid5_t(float *img_out, float *img_buf, float *img_in, int *state)
 * @brief           图像序列的前向后向选择中值滤波,使用前4帧和当前帧数据    
 * @param [in]      float *img0，img1，img2, img3, img4为历史图像帧(指针)，img0对应最老图像，img4对应最新（当前）图像
 * @param [in]      float *img_buf：历史图像帧(指针)，连续存放最近4帧图像
 * @param [in]      float *img_in：指针，指向最新输入图像
 * @param [in]      float th使用前向MID3滤波结果（新数据）的门限
 * @param [out]     float *img_out：指针，指向空间存放滤波结果
 * @param [inout]   int state：指针，指向滤波状态变量(最老的图像帧在img_buf中的位置），初始值需设为0，指向的内容在运行后被修改
 * @retval          float*：和img_out相同
 */
float *img_fb_mid3_t(float *img_out, float *img_buf, float *img_in, float th, int *state);

/** 
 * @fn              float *img_nnf_sqr3_mid5_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2, float *img_in3, float *img_in4, float th)
 * @brief           像素最近邻选择滤波，使用使用3x3滤波模板，如果邻近像素和中心像素差异过大则使用5帧图像的时间中值滤波
 * @param [in]      float *img_in：指针，指向待滤波图像
 * @param [in]      float th：滤波门限
 * @param [out]     float *img_out：指针，指向的空间存放图像运算结果
 * @retval          float *：和img_out相同
 */ 
float *img_nnf_sqr3_mid5_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2, float *img_in3, float *img_in4, float th);

/** 
 * @fn              float *img_nnf_sqr3_mid5(float *img_out, float *img_buf, float *img_in, int *state,float th)
 * @brief           像素最近邻选择滤波，使用使用3x3滤波模板，如果邻近像素和中心像素差异过大则使用5帧图像的时间中值滤波
 * @param [in]      float *img_buf：历史图像帧(指针)，连续存放最近4帧图像
 * @param [in]      float *img_in：指针，指向最新输入图像
 * @param [in]      float th：滤波门限
 * @param [out]     float *img_out：指针，指向空间存放滤波结果
 * @param [inout]   int state：指针，指向滤波状态变量(最老的图像帧在img_buf中的位置），初始值需设为0，指向的内容在运行后被修改
 * @retval          float*：和img_out相同
 */
float *img_nnf_sqr3_mid5(float *img_out, float *img_buf, float *img_in, int *state,float th);

/** 
 * @fn              float *img_mid7_st_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2)
 * @brief           图像序列的时空中值滤波,使用2帧历史数据
 * @param [in]      img0，img1，img2为历史图像帧(指针)，img0对应最老图像，img2对应最新（当前）图像
 * @param [out]     img_out：指针，指向空间存放滤波结果
 * @retval          float *：和img_out相同
 */
float *img_mid7_st_raw(float *img_out, float *img_in0, float *img_in1, float *img_in2);

/** 
 * @fn              float *img_mid7_st(float *img_out, float *img_buf, float *img_in, int *state)
 * @brief           从前后帧和当前帧得到7个像素，用中值取代当前帧数据
 * @param [in]      float *img_buf：历史图像帧(指针)，连续存放最近2帧图像
 * @param [in]      float *img_in：指针，指向最新输入图像
 * @param [out]     float *img_out：指针，指向空间存放滤波结果
 * @param [inout]   int state：指针，指向滤波状态变量，初始值需设为0，指向的内容在运行后被修改
 * @retval          float*：和img_out相同
 */
float *img_mid7_st(float *img_out, float *img_buf, float *img_in, int *state);

float img_plane_mf_pix(float z0, float z1, float z2, float z3, float z4, float z5, float z6, float z7, float z8);
//float *img_plane_mf_sqr3_sa(float *img_inout, struct ring_buf_f32_s *rbuf);
float *img_plane_mf_sqr3(float *img_out, float *img_in);

/** 
 * @fn              float *img_hole_fill(float *img_out, float *img_in, uint8_t *img_mask);
 * @details         像素空洞检测滤波，如果某个像素无效，且他的邻近像素超过(包括）5个非零，则用有效像素平均值填充
 *                  3x3图像滤波器模板如下
 *                  0(p)    1(p+1)    2(p+2)
 *                  3(p+W)  4(p+W+1)  5(p+W+2)
 *                  6(P+2W) 7(p+2W+1) 8(p+2W+2)
 *                  注意：滤波输出图像的最外圈边沿（1层像素）是无效数据
 * @param [in]      float *img_in：指针，指向待滤波图像
 * @param [in]      float *coff：指针，指向9个滤波系数
 * @param [out]     float *img_out：指针，指向的空间存放图像运算结果
 * @param [inout]   uint8_t *img_mask：指针，指向的空间存放空洞指示，注意，填补空洞后悔修改该指针对应空间内容
 * @retval          float *：和img_out相同
 */ 
float *img_hole_fill(float *img_out, float *img_in, uint8_t *img_mask);


float *img_nnd_sqr3(float *img_out, float *img_in);

#ifdef __cplusplus
}
#endif
#endif
