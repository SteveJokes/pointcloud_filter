#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "img_filter.h"
#define IMG_WID 320
#define IMG_HGT 240
float sqr_f32(float n)
{
    return sqrt(n);
}

float img_plane_mf_pix(float z0, float z1, float z2, float z3, float z4, float z5, float z6, float z7, float z8)
{
    float e0,e1,e2,e3,e4,e5,e6,e7;
    float minv;
    int min_id;
    float zc;

    e0= (sqr_f32(-5*z0+4*z1+  z2+3*z3     -3*z5)+   //  z0 z1 z2    // 用上方6个点你和平面，计算拟合误差e0
         sqr_f32( 4*z0-8*z1+4*z2               )+   //  z3 z4 z5
         sqr_f32(   z0+4*z1-5*z2-3*z3     +3*z5)+   //  *  *  *
         sqr_f32( 3*z0     -3*z2-5*z3+4*z4+  z5)+
         sqr_f32(                4*z3-8*z4+4*z5)+
         sqr_f32(-3*z0     +3*z2+  z3+4*z4-5*z5))/144;

    e1= (sqr_f32(-3*z0+3*z1-  z2+3*z4-  z5-  z8)+   //  z0 z1 z2    // 用右上方6个点你和平面，计算拟合误差e1
         sqr_f32( 3*z0-7*z1+3*z2+  z4+  z5-  z8)+   //  *  z4 z5
         sqr_f32(  -z0+3*z1-3*z2-  z4+3*z5-  z8)+   //  *  *  z8
         sqr_f32( 3*z0+  z1-  z2-7*z4+  z5+3*z8)+
         sqr_f32(  -z0+  z1+3*z2+  z4-7*z5+3*z8)+
         sqr_f32(  -z0-  z1-  z2+3*z4+3*z5-3*z8))/100;

    e2= (sqr_f32(-5*z1+3*z2+4*z4     +  z7-3*z8)+   //  *  z1 z2    // 用右方6个点你和平面，计算拟合误差e2
         sqr_f32( 3*z1-5*z2     +4*z5-3*z7+  z8)+   //  *  z4 z5
         sqr_f32( 4*z1     -8*z4     +4*z7     )+   //  *  z7 z8
         sqr_f32(      4*z2     -8*z5     +4*z8)+
         sqr_f32(   z1-3*z2+4*z4     -5*z7+3*z8)+
         sqr_f32(-3*z1+  z2     +4*z5+3*z7-5*z8))/144;

    e3= (sqr_f32(-3*z2+3*z4+3*z5-  z6-  z7-  z8)+   //  *  *  z2    // 用右下方6个点你和平面，计算拟合误差e3
         sqr_f32( 3*z2-7*z4+  z5+3*z6+  z7-  z8)+   //  *  z4 z5
         sqr_f32( 3*z2+  z4-7*z5-  z6+  z7+3*z8)+   //  z6 z7 z8
         sqr_f32(  -z2+3*z4-  z5-3*z6+3*z7-  z8)+
         sqr_f32(  -z2+  z4+  z5+3*z6-7*z7+3*z8)+
         sqr_f32(  -z2-  z4+3*z5-  z6+3*z7-3*z8))/100;

    e4= (sqr_f32(-5*z3+4*z4+  z5+3*z6     -3*z8)+   //  *  *  *     // 用下方6个点你和平面，计算拟合误差e4
         sqr_f32( 4*z3-8*z4+4*z5               )+   //  z3 z4 z5
         sqr_f32(   z3+4*z4-5*z5-3*z6     +3*z8)+   //  z6 z7 z8
         sqr_f32( 3*z3     -3*z5-5*z6+4*z7+  z8)+
         sqr_f32(                4*z6-8*z7+4*z8)+
         sqr_f32(-3*z3     +3*z5+  z6+4*z7-5*z8))/144;

    e5= (sqr_f32(-3*z0+3*z3+3*z4-  z6-  z7-  z8)+   //  z0 *  *     // 用左下方6个点你和平面，计算拟合误差e5
         sqr_f32( 3*z0-7*z3+  z4+3*z6+  z7-  z8)+   //  z3 z4 *
         sqr_f32( 3*z0+  z3-7*z4-  z6+  z7+3*z8)+   //  z6 z7 z8
         sqr_f32(  -z0+3*z3-  z4-3*z6+3*z7-  z8)+
         sqr_f32(  -z0+  z3+  z4+3*z6-7*z7+3*z8)+
         sqr_f32(  -z0-  z3+3*z4-  z6+3*z7-3*z8))/100;

    e6= (sqr_f32(-5*z0+3*z1+4*z3     +  z6-3*z7)+   //  z0 z2 *     // 用左方6个点你和平面，计算拟合误差e6
         sqr_f32( 3*z0-5*z1     +4*z4-3*z6+  z7)+   //  z3 z4 *
         sqr_f32( 4*z0     -8*z3     +4*z6     )+   //  z6 z7 *
         sqr_f32(      4*z1     -8*z4     +4*z7)+
         sqr_f32(   z0-3*z1+4*z3     -5*z6+3*z7)+
         sqr_f32(-3*z0+  z1     +4*z4+3*z6-5*z7))/144;

    e7= (sqr_f32(-3*z0+3*z1-  z2+3*z3-  z4-  z6)+   //  z0 z1 z2    // 用左上6个点你和平面，计算拟合误差e7
         sqr_f32( 3*z0-7*z1+3*z2+  z3+  z4-  z6)+   //  z3 z4 *
         sqr_f32(-  z0+3*z1-3*z2-  z3+3*z4-  z6)+   //  z6 *  *
         sqr_f32( 3*z0+  z1-  z2-7*z3+  z4+3*z6)+
         sqr_f32(  -z0+  z1+3*z2+  z3-7*z4+3*z6)+
         sqr_f32(  -z0-  z1-  z2+3*z3+3*z4-3*z6))/100;

    // 根据e0~e7，找到最优拟合方案
    minv=e0;
    min_id=0;
    if (e1<minv) { minv=e1; min_id=1; }
    if (e2<minv) { minv=e2; min_id=2; }
    if (e3<minv) { minv=e3; min_id=3; }
    if (e4<minv) { minv=e4; min_id=4; }
    if (e5<minv) { minv=e5; min_id=5; }
    if (e6<minv) { minv=e6; min_id=6; }
    if (e7<minv) { minv=e7; min_id=7; }

    // 计算中心点深度修正结果
         if (min_id==0) zc=(                  z3+  z4+  z5)/3 ;
    else if (min_id==1) zc=( 3*z0+  z1-  z2+3*z4+  z5+3*z8)/10;
    else if (min_id==2) zc=(   z1     +  z4     +  z7     )/3 ;
    else if (min_id==3) zc=( 3*z2+3*z4+  z5+3*z6+  z7  -z8)/10;
    else if (min_id==4) zc=(   z3+  z4+  z5               )/3 ;
    else if (min_id==5) zc=( 3*z0+  z3+3*z4  -z6+  z7+3*z8)/10;
    else if (min_id==6) zc=(        z1     +  z4     +  z7)/3 ;
    else if (min_id==7) zc=(  -z0+  z1+3*z2+  z3+3*z4+3*z6)/10;

    return zc;
}


float *img_plane_mf_sqr3(float *img_out, float *img_in)
{
    float *p0=img_in          , *p1=img_in          +1, *p2=img_in          +2;
    float *p3=img_in+  IMG_WID, *p4=img_in+  IMG_WID+1, *p5=img_in+  IMG_WID+2;
    float *p6=img_in+2*IMG_WID, *p7=img_in+2*IMG_WID+1, *p8=img_in+2*IMG_WID+2;

    float *q=img_out+IMG_WID+1,*q_end=img_out+IMG_WID*(IMG_HGT-1)-1;

    for (;q<q_end;q++,p0++,p1++,p2++,p3++,p4++,p5++,p6++,p7++,p8++)
        *q=img_plane_mf_pix(*p0,*p1,*p2,*p3,*p4,*p5,*p6,*p7,*p8);

    return img_out;
}

//float *img_plane_mf_sqr3_sa(float *img_inout, struct ring_buf_f32_s *rbuf)
//{
//    float s;
//    float *p0=img_inout          , *p1=img_inout          +1, *p2=img_inout          +2;
//    float *p3=img_inout+  IMG_WID, *p4=img_inout+  IMG_WID+1, *p5=img_inout+  IMG_WID+2;
//    float *p6=img_inout+2*IMG_WID, *p7=img_inout+2*IMG_WID+1, *p8=img_inout+2*IMG_WID+2;

//    float *q=img_inout+IMG_WID+1-3*IMG_WID,*q_end=img_inout+IMG_WID*(IMG_HGT-1)-1-3*IMG_WID;

//    int n=3*IMG_WID;

//    for (;q<q_end;q++,p0++,p1++,p2++,p3++,p4++,p5++,p6++,p7++,p8++)
//    {
//        s=img_plane_mf_pix(*p0,*p1,*p2,*p3,*p4,*p5,*p6,*p7,*p8);

//        if (n)
//        {
//            n--;
//            ring_buf_f32_io(rbuf,s);
//        }
//        else
//            *q=ring_buf_f32_io(rbuf,s);
//    }

//    for (n=0;n<3*IMG_WID;n++,q++)
//        *q=ring_buf_f32_io(rbuf,0);

//    return img_inout;
//}

