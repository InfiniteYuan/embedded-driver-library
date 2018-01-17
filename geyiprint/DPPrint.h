#ifndef _DPPRINT_H_
#define _DPPRINT_H_

#include "stm32f4xx.h"

#include "FIFOBuffer.h"
#include "TaskManager.h"
#include "USART.h"
#include "Protocol.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"

/*
1A 5B 00 ҳ��ʼָ��1
1A 5B 01 x_L x_H y_L y_H Width_L width_H Height_L Height_H Rotateҳ��ʼָ��2
1A 5D 00 ҳ����ָ��

1A 4F 00
1A 4F 01 PrintNum ҳ��ӡָ��

1A 54 00 x_L x_H y_L y_H String00
1A 54 01 x_L x_H y_L y_H FontHeight_L FontHeight_H FontType_L FontType_H String00 �ı�����ָ��

1A 5C 00 StartX_L StartrX_H StartY_L StartrY_H EndX_L EndX_H EndY_L EndY_L
1A 5C 01 StartX_L StartX_H StartY_L StartY_H EndX_L EndX_H EndY_L EndY_H Width_L Width_H Color �߶λ���ָ��

1A 26 00 Left_L Left_H Top_L Top_H Right_L Right_H Bottom_L Bottom_H
1A 26 01 Left_L Left_H Top_L Top_H Right_L Right_H Bottom_L Bottom_H Width_L Width_H Color ���ο����ָ��

1A 2A 00 Left_L Left_H Top_L Top_H Right_L Right_H Bottom_L Bottom_H Color ���ƾ��ο�ָ��

1A 0C 00
1A 0C 01 StopPosition Offset_L Offset_H ��ָֽ��
*/
class DPPrint
{
private:
	USART &com;
public:
	DPPrint(USART &usar);
	void TagPrint(TagData mTagData);
	void PrintPageStart(void);
	void PrintPageStart(uint16_t off_x, uint16_t off_y, uint16_t width, uint16_t height, bool IsRotate);
	void PrintPageEnd(void);
	void PagePrint(u8 num);
	void PaperRun(u8 StopType, uint16_t Offset);
	void TextDraw(uint16_t start_x, uint16_t start_y, uint16_t fontHeight, uint16_t fontType, char *string);
	void lineDraw(uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t width, char color);
	void rectangleDraw(uint16_t left_x, uint16_t left_y, uint16_t right_x, uint16_t right_y, uint16_t width, char color);
	void rectangleBlockDraw(uint16_t left_x, uint16_t left_y, uint16_t right_x, uint16_t right_y, char color);
	/*
	** ������void InitializePrint(void)
	** ���ܣ���ʼ����ӡ���������ӡ����
	*/
	void InitializePrint(void);
	/*
	** ������void jump_lattice(void)
	** ��������
	** ���ܣ�ÿ����һ�Σ���ӡλ������ƶ�һ���ַ���λ
	*/
	void jump_lattice(void);
	/*
	** ������void print_And_Line(void)
	** ��������
	** ���ܣ���ӡ���������ݲ���ǰ�ƽ�һ��
	*/
	void print_And_Line(void);
	/*
	** ������void Print_ASCII(unsigned char *Str,unsigned char StrLen)
	** ������*Str ָ��Ҫ��ӡ���ַ���
	StrLen ָ��Ҫ��ӡ�ַ����ĳ���
	** ���ܣ���Ҫ��ӡ���ַ������뻺����
	** ʾ������ӡASCII abcdefg Print_ASCII("abcdefg",7);
	*/
	void Print_ASCII(unsigned char *Str, unsigned char StrLen);
	/*
	** ������void Set_Right_Interval(unsigned char interval)
	** ������interval �Ҳ��ַ���� ÿ��λ0.125mm
	** ���ܣ������ַ��Ҳ�ļ��Ϊinterval X 0.125mm
	*/
	void Set_Right_Interval(unsigned char interval);
	/*
	** ������void Set_Print_Mode(unsigned char optbit)
	** ������optbit  ��������ĺ궨������������ֵ����ѡ��ͬ�Ĺ���ģʽ
	** ���ܣ�ѡ���ӡģʽ
	** ʾ������Ҫѡ������B
	*/
#define		EnableASCII9X17(optbit)			optbit=(optbit|0x01)			/* ѡ������B 9*17 */
#define		EnableASCII12X24(optbit)		optbit=(optbit&0xfe)			/* ѡ������A 12*24 */
#define		EnableInverse(optbit)				optbit=(optbit|0x02)			/* ʹ�ܴ�ӡ����ģʽ */
#define		DisableInverse(optbit)			optbit=(optbit&0xfd)			/* ȡ����ӡ����ģʽ */	
#define		EnableInversion(optbit)			optbit=(optbit|0x04)			/* �������µ���ģʽ */
#define		DisableInversion(optbit)		optbit=(optbit&0xfb)			/* ȡ�����µ���ģʽ */
#define		EnableBold(optbit)					optbit=(optbit|0x08)			/* ���ô���ģʽ */
#define		DisableBold(optbit)					optbit=(optbit&0x07)			/* ȡ������ģʽ */
#define		EnableDoubleHight(optbit)		optbit=(optbit|0x10)			/* ���ñ���ģʽ */
#define		DisableDoubleHight(optbit)	optbit=(optbit&0xef)			/* ȡ������ģʽ */
#define		EnableDoubleWide(optbit)		optbit=(optbit|0x20)			/* ���ñ���ģʽ */
#define		DisableDoubleWide(optbit)		optbit=(optbit&0xdf)			/* ȡ������ģʽ */
#define		EnableUnderLine(optbit)			optbit=(optbit|0x40)			/* ʹ���»��� */
#define		DisableUnderLine(optbit)		optbit=(optbit&0xbf)			/* ȡ���»��� */
	void Set_Print_Mode(unsigned char optbit);
	/*
	** ������void Set_Print_Position(unsigned char Lpos,unsigned char Hpos)
	** ������Lpos��ӡλ����ֵ�ĵ�8λ��Hpos��ӡλ����ֵ�ĸ�8λ
	** ���ܣ��趨��һ�еĿ�ʼ����Ҫ��ӡ�ַ���λ�þ���Ϊ(Lpos+Hpos*256)*0.125mm
	*/
	void Set_Print_Position(unsigned char Lpos, unsigned char Hpos);
	/*
	** ������void Set_Left_Interval(unsigned char Interval)
	** ������Interval ��߼��  0=<Interval<=47
	** ���ܣ�������߼��
	*/
	void Set_Left_Interval(unsigned char Interval);
	/*
	** ������void Sel_Custom_Character(unsigned char SelOpt)
	** ������SelOpt  0x00 ��ʾȡ���û��Զ����ַ���
	0x01 ��ʾѡ���û��Զ����ַ���
	** ���ܣ�ѡ��/ȡ���û��Զ����ַ���
	** ʾ����ѡ���Զ����ַ���
	Sel_Custom_Character(EnableCustomCharacter);
	*/
#define		EnableCustomCharacter		(0x01)
#define		DisableCustomCharacter	(0x00)
	void Sel_Custom_Character(unsigned char SelOpt);
	/*
	** ������void UserDefineCharacter(unsigned char yByte,unsigned char xDot, unsigned char DefChar,unsigned char *pData)
	** ������yByte ��ֱ�����ֽ���
	xDot	 ˮƽ�������
	DefChar �Զ����ַ�
	*pData �Զ����ַ���������
	*/
	void UserDefineCharacter(unsigned char yByte, unsigned char xDot, unsigned char DefChar, unsigned char *pData);
	/*
	** ������void Sel_Bitmap_Mode(unsigned char mode,unsigned char DatLenLowByte,unsigned char DatLenHightByte,unsigned char *pDotData)
	** ������mode 0 8-�� ���ܶ�
	1 8-�� ˫�ܶ�
	32 24-�� ���ܶ�
	33 24-�� ˫�ܶ�
	DatLenLowByte ��������ˮƽ���ȵĵ�8λ
	DatLenHightByte ��������ˮƽ���ȵĸ�8λ
	pDotData ָ��������ݵ�ָ��
	*/
	void Sel_Bitmap_Mode(unsigned char mode, unsigned char DatLenLowByte, unsigned char DatLenHightByte, unsigned char *pDotData);
	/*
	** ������void Set_UnderLine(unsigned char Opt)
	** ������Opt 0 ����»���ģʽ
	1 �趨1����»���
	2 �趨2����»���
	*/
	void Set_UnderLine(unsigned char Opt);
	/*
	** ������void SetDefaultLineInterval(void)
	** ���ܣ�����ȱʡ�м��3.75mm
	*/
	void SetDefaultLineInterval(void);
	/*
	** ������void Del_UserDefineCharacter(unsigned char SelCharacter)
	** ������SelCharacter  32=<SelCharacter<=126
	** ���ܣ�ȡ���Զ����ַ�
	*/
	void Del_UserDefineCharacter(unsigned char SelCharacter);
	/*
	** ������void SetHorizPosition(unsigned char Position)
	** ������Position ˮƽ��λλ��
	*/
	void SetHorizPosition(unsigned char Position);
	/*
	** ������void SetBold(unsigned char opt)
	** ������opt  ==0 ��������ӡģʽ
	>=1 �趨�����ӡģʽ
	*/
	void SetBold(unsigned char opt);
	/*
	** ������void PrintGoPage(unsigned char nstep)
	** ������nstep ��ֽ���� nstep*0.125mm
	** ���ܣ���ӡ����ֽnstep*0.125mm
	*/
	void PrintGoPage(unsigned char nstep);
	/*
	** ������void SelCountryCharacter(unsigned char nsel)
	** ������nsel  0<=nsel<=15
	0:����	1:����	2:�¹�	3:Ӣ��	4:����I		5:���	6:�����
	7:������I		8:�ձ�	9:Ų��	10:����II		11:������II		12:��������
	13:����		14:˹��������		15:�й�
	*/
	void SelCountryCharacter(unsigned char nsel);
	/*
	** ������void Set_Rotate(unsigned char nsel)
	** ������nsel  0 ȡ��˳ʱ��90����תģʽ
	1 ����˳ʱ��90����תģʽ
	*/
	void Set_Rotate(unsigned char nsel);
	/*
	** ������void Get_Print_State(void)
	** ���ܣ����ô˺������ӡ�����Ͳ�ѯ״̬�����ӡ������һ���ֽ����ݣ����ɿͻ������մ���
	�����ֽڸ�λ�����壺
	Bit0: 0 ��оδ���ӣ�1 ��о������
	Bit1:	������
	Bit2:	0 ��ֽ	1 ȱֽ
	Bit3:	0 ��ѹ����	1 ��ѹ����9.5V
	Bit4: ������
	Bit5: ������
	Bit6: 0 �¶�����	1 �¶ȳ���60��
	Bit7: ������
	*/
	void Get_Print_State(void);
	/*
	** ������void Sel_Align_Way(unsigned char nsel)
	** ������nsel  0<=nsel<=2   0 �����		1 ����		2 �Ҷ���
	*/
	void Sel_Align_Way(unsigned char nsel);
	/*
	** ������void SetReversal(unsigned char nsel)
	** ������nsel 0 �رյߵ���ӡģʽ		1 �򿪵ߵ���ӡģʽ
	*/
	void SetReversal(unsigned char nsel);
	/*
	** ������void SetCharacterSize(unsigned char width,unsigned char hight)
	** ������ 0<=width<=7 ��ȷŴ�1~8��  0<=hight<=7 �߶ȷŴ�1~8��
	** ���ܣ������ַ���С
	*/
	void SetCharacterSize(unsigned char width, unsigned char hight);
	/*
	** ������void DownLoadBitmap(unsigned char xDot,unsigned char yDot,unsigned char *pDat)
	** ������xDot ˮƽ������� yDot ��ֱ������� *pDat λͼ��������ָ�� xDot yDot���Ϊ8�ı���
	*/
	void DownLoadBitmap(unsigned char xDot, unsigned char yDot, unsigned char *pDat);
	/*
	** ������void PrintDownLoadBitmap(unsigned char mode)
	** ������mode ��ӡģʽ 0 ��ͨ  1 ����  2 ����  3 4����С
	** ���ܣ���ӡ��DownLoadBitmap���������λͼ����
	*/
	void PrintDownLoadBitmap(unsigned char mode);
	/*
	** ������void Set_Inverse(unsigned char opt)
	** ������opt 0 ����ģʽ�ر�
	1 ����ģʽ��
	*/
	void Set_Inverse(unsigned char opt);
	/*
	** ������void Set_LeftSpaceNum(unsigned char nL,unsigned char nH)
	** ������������߿հ���Ϊ(nL+nH*256)*0.125mm
	** ���ܣ�
	*/
	void Set_LeftSpaceNum(unsigned char nL, unsigned char nH);
	/*
	** ������void Set_HRIPosition(unsigned char opt)
	** ������opt 0 ����ӡHRI		1	�������Ϸ�		2	�������·�		3 ��������Ϸ����·�
	** ���ܣ���ӡ������ʱѡ��HRI�ַ��Ĵ�ӡλ��
	*/
	void Set_HRIPosition(unsigned char opt);
	/*
	** ������void Set_BarCodeHight(unsigned char verticalDotNum)
	** ������verticalDotNum ����ĸ߶ȣ���ֱ����ĵ��� ȱʡֵ162
	** ���ܣ�����������߶�
	*/
	void Set_BarCodeHight(unsigned char verticalDotNum);
	/*
	** ������void Set_BarCodeLeftSpace(unsigned char SpaceNum)
	** ������SpaceNum ��߼��
	** ���ܣ�������������߼��
	*/
	void Set_BarCodeLeftSpace(unsigned char SpaceNum);
	/*
	** ������void Set_BarCodeWidth(unsigned char widthsel)
	** ������widthsel (2<=widthsel<=6)  2->0.25mm	3->0.375mm 4->0.56mm 5->0.625mm 6->0.75mm
	** ���ܣ�����������
	*/
	void Set_BarCodeWidth(unsigned char widthsel);
	/*
	** ������void PrintBarCode(unsigned char CodeType,unsigned char *pData,unsigned char pDataLen)
	** ������CodeType								pDataLen					��������
	65 UPC-A								11<=k<=12				  48<=d<=57
	66 UPC-E								11<=k<=12					48<=d<=57
	67 JAN13(EAN13)				12<=k<=13					48<=d<=57
	68 JAN8(EAN8)					7<=k<=8						48<=d<=57
	69 CODE39							1<=k							48<=d<=57,65<=d<=90,32,36,37,43,45,46,47
	70 ITF									1<=k							48<=d<=57
	71 CODABAR							1<=k							48<=d<=57,65<=d<=90,36,43,45,46,47,58
	72 CODE93							1<=k<=255					0<=d<=127
	73 CODE128							2<=k<=255					0<=d<=127
	** ���ܣ���ӡ������
	*/
	void PrintBarCode(unsigned char CodeType, unsigned char *pData, unsigned char pDataLen);
	/*
	** ������void SetChinesemode(unsigned char opt)
	** ������opt ������ĺ궨�����
	** ���ܣ����ú����ַ���ӡģʽ
	** ʾ����ʹ�ܱ���ʹ�ܱ���
	*/
#define	EnableChinaDoubleWidth(opt)		opt=opt|0x04			/* ʹ�ܱ��� */
#define DisableChinaDoubleWidth(opt)	opt=opt&0xfb			/* ȡ������ */
#define	EnableChinaDoubleHight(opt)		opt=opt|0x08			/* ʹ�ܱ��� */
#define	DisableChinaDoubleHight(opt)	opt=opt&0xf7			/* ȡ������ */
#define	EnableChinaUnderLine(opt)			opt=opt|0x80			/* ʹ���»���*/
#define	DisableChinaUnderLine(opt)		opt=opt&0x7f			/* ȡ���»��� */
	void SetChinesemode(unsigned char opt);
	/*
	** ������void SelChineseChar(void)
	** ���ܣ�ѡ���ӡ��Ϊ���ִ�ӡģʽ
	*/
	void SelChineseChar(void);
	/*
	** ������void DisChineseChar(void)
	** ���ܣ�ȡ����ӡ�����ִ�ӡģʽ
	*/
	void DisChineseChar(void);
	/*
	** ������void Set_ChineseCode(unsigned char selopt)
	** ������selopt  0 GBK����  1 UTF-8����  3 BIG6�������
	** ���ܣ�ѡ�����ı����ʽ
	*/
	void Set_ChineseCode(unsigned char selopt);
	/*
	** ������void TestPrintPage(void)
	** ���ܣ���ӡ�Բ�ҳ
	*/
	void TestPrintPage(void);
	/*
	** ������void PrintGratinmap(unsigned char mode,unsigned int xDot,unsigned int yDot,unsigned char *pData)
	** ������mode λͼģʽ  0-->��ͨ  1-->����  2-->����   3-->�ı���С
	xDot ˮƽ�������
	yDot ��ֱ�������
	*pData λͼ����
	** ���ܣ���ӡ��դλͼ
	*/
	void PrintGratinmap(unsigned char mode, unsigned int xDot, unsigned int yDot, unsigned char *pData);
	/*
	** ������void Set_QRcodeMode(unsigned char mode)
	** ������mode ����QR���ģ������ [mode*mode]��
	*/
	void Set_QRcodeMode(unsigned char mode);
	/*
	** ������void Set_QRCodeAdjuLevel(unsigned char level)
	** ������level  level>=0x30&&level<=0x33
	** ���ܣ�����QR��Ĵ���У��ˮƽ���
	*/
	void Set_QRCodeAdjuLevel(unsigned char level);
	/*
	**	������void Set_QRCodeBuffer(unsigned int Len,unsigned char *pData)
	**	������Len ���QR�뻺�峤��  *pData ���QR�뻺������
	**	���ܣ��洢QR������ݵ�QR�뻺����
	*/
	void Set_QRCodeBuffer(unsigned int Len, unsigned char *pData);
	/*
	**	������void PrintQRCode(void)
	**	���ܣ���ӡQRCode��
	*/
	void PrintQRCode(void);

};
#endif
