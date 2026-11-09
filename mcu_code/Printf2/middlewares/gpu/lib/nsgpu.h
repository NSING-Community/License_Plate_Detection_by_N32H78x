/**
*     Copyright (c) 2025, Nations Technologies Inc.
* 
*     All rights reserved.
*
*     This software is the exclusive property of Nations Technologies Inc. (Hereinafter 
* referred to as NATIONS). This software, and the product of NATIONS described herein 
* (Hereinafter referred to as the Product) are owned by NATIONS under the laws and treaties
* of the People's Republic of China and other applicable jurisdictions worldwide.
*
*     NATIONS does not grant any license under its patents, copyrights, trademarks, or other 
* intellectual property rights. Names and brands of third party may be mentioned or referred 
* thereto (if any) for identification purposes only.
*
*     NATIONS reserves the right to make changes, corrections, enhancements, modifications, and 
* improvements to this software at any time without notice. Please contact NATIONS and obtain 
* the latest version of this software before placing orders.

*     Although NATIONS has attempted to provide accurate and reliable information, NATIONS assumes 
* no responsibility for the accuracy and reliability of this software.
* 
*     It is the responsibility of the user of this software to properly design, program, and test 
* the functionality and safety of any application made of this information and any resulting product. 
* In no event shall NATIONS be liable for any direct, indirect, incidental, special,exemplary, or 
* consequential damages arising in any way out of the use of this software or the Product.
*
*     NATIONS Products are neither intended nor warranted for usage in systems or equipment, any
* malfunction or failure of which may cause loss of human life, bodily injury or severe property 
* damage. Such applications are deemed, "Insecure Usage".
*
*     All Insecure Usage shall be made at user's risk. User shall indemnify NATIONS and hold NATIONS 
* harmless from and against all claims, costs, damages, and other liabilities, arising from or related 
* to any customer's Insecure Usage.

*     Any express or implied warranty with regard to this software or the Product, including,but not 
* limited to, the warranties of merchantability, fitness for a particular purpose and non-infringement
* are disclaimed to the fullest extent permitted by law.

*     Unless otherwise explicitly permitted by NATIONS, anyone may not duplicate, modify, transcribe
* or otherwise distribute this software for any purposes, in whole or in part.
*
*     NATIONS products and technologies shall not be used for or incorporated into any products or systems
* whose manufacture, use, or sale is prohibited under any applicable domestic or foreign laws or regulations. 
* User shall comply with any applicable export control laws and regulations promulgated and administered by 
* the governments of any countries asserting jurisdiction over the parties or transactions.
**/
 
/**
 *\*\file nsgpu.h
 *\*\author Nations
 *\*\version v1.0.0
 *\*\copyright Copyright (c) 2025, Nations Technologies Inc. All rights reserved.
 **/

#ifndef NSGPU_H
#define NSGPU_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "nsgpu_hw.h"

//Reference the values below but don't modify them except GPU_MEMORY_BASE_ADDRESS
#define FRAME_BUF_MAX       									2 
#define NSGPU_LVGL_FUNC										0
#define CMD_SYNC_MODE										0
#define GPU_MEMORY_BASE_ADDRESS   								0xC0C00000

#if NSGPU_LVGL_FUNC	
#include "lv_draw_private.h"
#include "lv_draw_nsgpu_utils.h"
#include "lv_draw_label_private.h"
#endif

int nsgpu_init(u32 gpu_memory_base, u32 fb_num, u32 width, u32 height,u32 pixel_bytes);

void nsgpu_deinit(void);

u32 nsgpu_get_ready_framebuffer(u8 index);

void nsgpu_set_ready_framebuffer(u8 index);

u32 nsgpu_get_framebuffer(void);

void nsgpu_set_framebuffer(u32 addr);

void nsgpu_data_fill(u32 *dst, u32 dst_width, u32 dst_height, u32 dst_startx, u32 dst_starty, u32 color, u8 alpha, u32 width, u32 height);

void nsgpu_data_copy(u32 *dst, u32 dst_width, u32 dst_height, u32 dst_startx, u32 dst_starty, u32 *src, u32 width, u32 height, u32 pitch, u32 pixel_bytes);

void nsgpu_set_draw_color(u32 color);

void nsgpu_set_draw_alpha(u8 alpha);

void nsgpu_vgdraw_line(float x1, float y1, float x2, float y2, float width);

void nsgpu_vgdraw_rect(float x, float y, float width, float height);

void nsgpu_vgdraw_triangle(float x1, float y1, float x2, float y2,float x3, float y3);
		
void nsgpu_vgdraw_circle(float cx, float cy, float r);

#if NSGPU_LVGL_FUNC
void nsgpu_lvgl_fill_proc(lv_draw_task_t *t,const lv_draw_fill_dsc_t * draw_dsc,lv_area_t draw_area);

void nsgpu_lvgl_label_proc(lv_draw_task_t *t,lv_draw_glyph_dsc_t * glyph_draw_dsc,lv_area_t draw_area,lv_area_t letter_area);

void nsgpu_lvgl_image_proc(lv_draw_task_t *t,const lv_draw_image_dsc_t * image_draw_dsc,lv_area_t draw_area,lv_area_t image_area);
#endif




#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*nsgpu.h*/
