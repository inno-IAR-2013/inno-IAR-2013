/******************** (C) COPYRIGHT 2011 珧鳶宒羲楷馱釬弅 ********************
 * 恅璃靡       ㄩcommon.h
 * 鏡扴         ㄩ籵蚚饜离芛恅璃ㄛ巠磁窒腔馱最
 *
 * 妗桄怢     ㄩ珧鳶kinetis羲楷啣
 * 踱唳掛       ㄩ
 * 炵苀     ㄩ
 *
 * 釬氪         ㄩ
 * 杬惘虛       ㄩhttp://firestm32.taobao.com
 * 撮扲盓厥蹦抭 ㄩhttp://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/	

#ifndef _COMMON_H_
#define _COMMON_H_

//峈妏蚚溘淩耀宒奧氝樓腔ㄛ溘淩耀宒茼蜆敖揹諳楷冞﹜睿PLL坶眈遠扢离
//#define   Simulator
#ifdef    Simulator
  #define NO_PLL_INIT   //輦蚚坶眈遠
  #define NPRINTF       //輦蚚printf
#endif

/********************************************************************/

/*
 * Debug prints ON (#define) or OFF (#undef)
 */
//#define DEBUG_PRINT



/*****************************扢离杅擂濬倰*****************************/
typedef 	unsigned 	char		u8;	  //拸睫瘍倰
typedef 	unsigned 	short int	u16;
typedef 	unsigned 	long  int	u32;

typedef 			char		s8;	  //衄睫瘍倰
typedef 			short int	s16;
typedef 			long  int	s32;



/*
 * Include the generic CPU header file
 */
#include "arm_cm4.h"

/*
 * Include the platform specific header file
 */
#if (defined(TWR_K40X256))
  #include "k40_tower.h"
#elif (defined(TWR_K60N512))
// #include "k60_tower.h"
  #include "k60_fire.h"
#elif (defined(TWR_K53N512))
 #include "k53_tower.h"
#else
  #error "No valid platform defined"
#endif

/*
 * Include the cpu specific header file
 */
#if (defined(CPU_MK40N512VMD100))
//  #include "MK40N512VMD100.h"
  #include "MK40DZ10.h"
#elif (defined(CPU_MK60N512VMD100))
// #include "MK60N512VMD100.h"
  #include "MK60DZ10.h"
#elif (defined(CPU_MK53N512CMD100))
//  #include "MK53N512CMD100.h"
  #include "MK53DZ10.h"
#else
  #error "No valid CPU defined"
#endif


/*
 * Include any toolchain specfic header files
 */
#if (defined(CW))
  #include "cw.h"
#elif (defined(IAR))
  #include "iar.h"
#else
#warning "No toolchain specific header included"
#endif

/*
 * Include common utilities
 */
#include "assert.h"
#include "io.h"
#include "startup.h"
#include "stdlib.h"


#if (defined(IAR))
	#include "intrinsics.h"
#endif


#include  "sysinit.h"           //炵苀饜离
#include "mcg.h"
#include "fire_drivers_cfg.h"   //奪褐葩蚚饜离


/********************************************************************/

#endif /* _COMMON_H_ */
