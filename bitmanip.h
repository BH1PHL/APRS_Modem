#ifndef __BITMANIP_H_
#define __BITMANIP_H_

#define BIT(x)	(1 << (x))		//ѡ��ĳһλ

#define STB(p,b) ((p)|=(b))		//��λ
#define CLB(p,b) ((p)&=~(b))	//��λ
#define FLB(p,b) ((p)^=(b))	  	//flip bit
#define CHB(p,b) ((p) & (b))	//����ĳһλ�Ƿ���0

#endif
