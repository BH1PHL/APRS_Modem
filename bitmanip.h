#ifndef __BITMANIP_H_
#define __BITMANIP_H_

#define BIT(x)	(1 << (x))		//选择某一位

#define STB(p,b) ((p)|=(b))		//置位
#define CLB(p,b) ((p)&=~(b))	//清位
#define FLB(p,b) ((p)^=(b))	  	//flip bit
#define CHB(p,b) ((p) & (b))	//测试某一位是否是0

#endif
