//USB ���ݶ�����
#include "typedefint.h"

#pragma udata 
USB_DEVICE_STATUS  usb_stat;   // USB�豸״̬�Ĵ���
byte usb_device_state;         // Device States: DETACHED, ATTACHED, ...
byte usb_active_cfg;           // Value of current configuration
byte usb_alt_intf[MAX_NUM_INT];// ��һ�����飬Ϊ��������MAX_NUM_INT=1���ӿ���Ŀ

byte ctrl_trf_state;           // ���ƴ���״̬�Ĵ���
byte ctrl_trf_session_owner;   // ���ƴ������ж���

POINTER pSrc;                  // Data source pointer ������ָ�� 
POINTER pDst;                  // Data destination pointer ����Ŀ�ĵ�ָ��
WORD wCount;                   // Data counter  �����ֽڼ�����

byte hid_rpt_rx_len;           // HID��������ֽ���Ŀ�Ĵ���
byte idle_rate;                // HID �������ֽ�
byte active_protocol;          // Э��=0����Э�� =1����Э��
//********** USB �û������� ************
byte number_of_bytes_read;           // ��ȡ�ֽڵ���Ŀ
char transmit_buffer[EP0_BUFF_SIZE]; // 8�ֽ�����
char receive_buffer[EP0_BUFF_SIZE];  // 8���ֽ�����

//--------------------------------------USB�˵�RAM����
#pragma udata usbram4=0x400   // 256���ֽڣ�һҳ������USB�� ����������������ҳ
                              // �������ʣ���ֽڿ�����Ϊ���ݴ洢��
/*
 ������16���˵�Ķ���
*/
#if(0 <= MAX_EP_NUMBER)
volatile far BDT ep0Bo;         //Endpoint #0 BD Out
volatile far BDT ep0Bi;         //Endpoint #0 BD In
#endif

#if(1 <= MAX_EP_NUMBER)
volatile far BDT ep1Bo;         //Endpoint #1 BD Out
volatile far BDT ep1Bi;         //Endpoint #1 BD In
#endif

#if(2 <= MAX_EP_NUMBER)
volatile far BDT ep2Bo;         //Endpoint #2 BD Out
volatile far BDT ep2Bi;         //Endpoint #2 BD In
#endif

#if(3 <= MAX_EP_NUMBER)
volatile far BDT ep3Bo;         //Endpoint #3 BD Out
volatile far BDT ep3Bi;         //Endpoint #3 BD In
#endif

#if(4 <= MAX_EP_NUMBER)
volatile far BDT ep4Bo;         //Endpoint #4 BD Out
volatile far BDT ep4Bi;         //Endpoint #4 BD In
#endif

#if(5 <= MAX_EP_NUMBER)
volatile far BDT ep5Bo;         //Endpoint #5 BD Out
volatile far BDT ep5Bi;         //Endpoint #5 BD In
#endif

#if(6 <= MAX_EP_NUMBER)
volatile far BDT ep6Bo;         //Endpoint #6 BD Out
volatile far BDT ep6Bi;         //Endpoint #6 BD In
#endif

#if(7 <= MAX_EP_NUMBER)
volatile far BDT ep7Bo;         //Endpoint #7 BD Out
volatile far BDT ep7Bi;         //Endpoint #7 BD In
#endif

#if(8 <= MAX_EP_NUMBER)
volatile far BDT ep8Bo;         //Endpoint #8 BD Out
volatile far BDT ep8Bi;         //Endpoint #8 BD In
#endif

#if(9 <= MAX_EP_NUMBER)
volatile far BDT ep9Bo;         //Endpoint #9 BD Out
volatile far BDT ep9Bi;         //Endpoint #9 BD In
#endif

#if(10 <= MAX_EP_NUMBER)
volatile far BDT ep10Bo;        //Endpoint #10 BD Out
volatile far BDT ep10Bi;        //Endpoint #10 BD In
#endif

#if(11 <= MAX_EP_NUMBER)
volatile far BDT ep11Bo;        //Endpoint #11 BD Out
volatile far BDT ep11Bi;        //Endpoint #11 BD In
#endif

#if(12 <= MAX_EP_NUMBER)
volatile far BDT ep12Bo;        //Endpoint #12 BD Out
volatile far BDT ep12Bi;        //Endpoint #12 BD In
#endif

#if(13 <= MAX_EP_NUMBER)
volatile far BDT ep13Bo;        //Endpoint #13 BD Out
volatile far BDT ep13Bi;        //Endpoint #13 BD In
#endif

#if(14 <= MAX_EP_NUMBER)
volatile far BDT ep14Bo;        //Endpoint #14 BD Out
volatile far BDT ep14Bi;        //Endpoint #14 BD In
#endif

#if(15 <= MAX_EP_NUMBER)
volatile far BDT ep15Bo;        //Endpoint #15 BD Out
volatile far BDT ep15Bi;        //Endpoint #15 BD In
#endif
//USB Buffer
volatile far CTRL_TRF_SETUP SetupPkt;    //EP0�˵�������õ����ݻ�����
volatile far CTRL_TRF_DATA  CtrlTrfData; //�����շ�������

// HID Buffer
volatile far unsigned char hid_report_out[HID_INT_OUT_EP_SIZE];  //�豸��������
volatile far unsigned char hid_report_in[HID_INT_IN_EP_SIZE];    //�豸��������
volatile far unsigned char hid_report_feature[EP0_BUFF_SIZE];    //����ֵ����



#pragma romdata   //������������ROM����������ʽ�洢

// ����������
rom struct{byte report[HID_RPT01_SIZE];}hid_rpt01={  

	  	0x06, 0xA0, 0xFF,	// Usage page (vendor defined)������ָ���豸�Ĺ���
	  	0x09, 0x01,	        // Usage ID (vendor defined) ָ�����𱨱���
	  	0xA1, 0x01,	        // Collection (application)  

		// ���뱨��
        0x09, 0x03,     	// Usage ID - vendor defined
        0x15, 0x00,     	// Logical Minimum (0)  
        0x26, 0xFF, 0x00,   // Logical Maximum (255)
        0x75, 0x08,     	// Report Size (8 bits)
        0x95, 0x02,     	// Report Count (2 fields)
        0x81, 0x02,     	// Input (Data, Variable, Absolute)  

		// �������
        0x09, 0x04,     	// Usage ID - vendor defined
        0x15, 0x00,     	// Logical Minimum (0)
        0x26, 0xFF, 0x00,   // Logical Maximum (255)
        0x75, 0x08,     	// Report Size (8 bits)
        0x95, 0x02,     	// Report Count (2 fields)
        0x91, 0x02,      	// Output (Data, Variable, Absolute)  

		// ��������
        0x09, 0x05,     	// Usage ID - vendor defined
        0x15, 0x00,     	// Logical Minimum (0)
        0x26, 0xFF, 0x00,   // Logical Maximum (255)
        0x75, 0x08,	        // Report Size (8 bits)
        0x95, 0x02, 		// Report Count (2 fields)
        0xB1, 0x02,     	// Feature (Data, Variable, Absolute)  

	  	0xC0};	            // end collection

// �豸����������

rom USB_DEV_DSC device_dsc=
{
   sizeof(USB_DEV_DSC),   //��������С
   DSC_DEV,               //����0x01
   0x0200,                //USB�汾���� 2.0
   0x00,                  //�����
   0x00,                  //�������
   0x00,                  //Э����
   EP0_BUFF_SIZE,         //�˵�0�������Ϣ����С
   0xFFFF,                //����ID gcat
   0x1000,                //��ƷID gcat
   0x0001,                //�豸�汾��
   0x01,                  //�����ߵ��ַ���������������ֵ
   0x02,                  //��Ʒ���ַ���������������ֵ
   0x00,                  //���к��ַ���������������ֵ
   0x01,                  //�豸֧�ֵ��������õ���Ŀ��Ҳ����������������Ŀ
}; 

// ��������

CFG01=
{
// ����������
  sizeof(USB_CFG_DSC),   // ������������С
  DSC_CFG,               // ��������������
  sizeof(cfg01),         // �������������ص��������ݴ�С
  1,                     // ������֧�ֵĶ˿���Ŀ
  1,                      
  0,                     // �������ַ���������������ֵ 
  _DEFAULT|_RWU,         // �Ա���Դ,֧��Զ�̻���
  50,                    // ����������100MA����
 
// �����������������ĸ����������������ӿںͶ˵��Լ�HID��������
// �ӿ�������
  sizeof(USB_INTF_DSC),  // �ӿ���������С
  DSC_INTF,              // �ӿ�����������
  0,                     // ʶ��˽ӿڵ����֣�bInterface
  0,                     // ѡ��һ��������õ���ֵbAlternateInterface
  2,                     // ���˵�0��֧�ֵĶ˵���Ŀ��ֻ֧�ֶ˵�1
  HID_INTF,              // HID ����룬ֻ��HID�ڴ˸����������������е���������
  0,                     // ����������0 
  0,                     // Э����
  0,                     // �˽ӿڵ��ַ���������������ֵ  

//USB_HID_DSC��HID���������
 sizeof(USB_HID_DSC),    // HID��������С
 DSC_HID,                // HID 
 0x0101,                 // HID�淶�汾���� 2.0
 0x00,                   // Ӳ��Ŀ�Ĺ��ҵ�ʶ������
 HID_NUM_OF_DSC,         // ֧�ֵĸ������������������Ŀ   
 DSC_RPT,                // ���������������Ǳ������
 sizeof(hid_rpt01),      // �������������ܳ���   

//HID�Ķ˵����������˵�1 
 sizeof(USB_EP_DSC),	     // ��������С
   DSC_EP,			         // ����=0x05
   _EP01_IN,			     // �˵�1��������=0x81	
   _INT,					 // �жϴ��䣬��2λ��Ч 11�жϴ��䣬10�������䣬01ʵʱ���䣬00���ƴ���
   HID_INT_IN_EP_SIZE,	     // �������Ϣ����С8���ֽ�
   0x0A,					 // ��ѯ�˵��ʱ�࣬�Ժ���Ϊ��λ
	
   sizeof(USB_EP_DSC),	     // ��������С
   DSC_EP,				     // ����=0x05
   _EP01_OUT,			     // =0x01�˵�1��������	
   _INT,				     // �жϴ���
   HID_INT_OUT_EP_SIZE,      // Maximum packet size  =8 �������Ϣ��
   0x0A	                     // ��ѯ�˵��ʱ��MSΪ��λ
};

rom struct{byte bLength;byte bDscType;word string[1];}sd000={
sizeof(sd000),0x03,0x0409};

rom struct{byte bLength;byte bDscType;word string[25];}sd001={
sizeof(sd001),0x03,
'M','i','c','r','o','c','h','i','p',' ',
'T','e','c','h','n','o','l','o','g','y',' ','I','n','c','.'};

rom struct{byte bLength;byte bDscType;word string[23];}sd002={
sizeof(sd002),0x03,
'U','S','B',' ','C','o','m','l','e','t','e',' ',
'G','e','n','e','r','i','c',' ','H','I','D'};

rom const unsigned char *rom USB_CD_Ptr[]={&cfg01,&cfg01};        //ROMָ������,���cfg01��ַ
rom const unsigned char *rom USB_SD_Ptr[]={&sd000,&sd001,&sd002}; //ROM ָ������,����ַ�����ַ

