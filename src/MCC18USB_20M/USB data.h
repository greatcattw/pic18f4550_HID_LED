//USB 数据定义区
#include "typedefint.h"

#pragma udata 
USB_DEVICE_STATUS  usb_stat;   // USB设备状态寄存器
byte usb_device_state;         // Device States: DETACHED, ATTACHED, ...
byte usb_active_cfg;           // Value of current configuration
byte usb_alt_intf[MAX_NUM_INT];// 清一个数组，为交替设置MAX_NUM_INT=1，接口数目

byte ctrl_trf_state;           // 控制传输状态寄存器
byte ctrl_trf_session_owner;   // 控制传输所有定义

POINTER pSrc;                  // Data source pointer 数据域指针 
POINTER pDst;                  // Data destination pointer 数据目的地指针
WORD wCount;                   // Data counter  数据字节计数器

byte hid_rpt_rx_len;           // HID报表接收字节数目寄存器
byte idle_rate;                // HID 闲置率字节
byte active_protocol;          // 协议=0启动协议 =1报表协议
//********** USB 用户数据区 ************
byte number_of_bytes_read;           // 读取字节的数目
char transmit_buffer[EP0_BUFF_SIZE]; // 8字节数据
char receive_buffer[EP0_BUFF_SIZE];  // 8个字节数据

//--------------------------------------USB端点RAM定义
#pragma udata usbram4=0x400   // 256个字节（一页）这是USB的 缓冲器描述符所在页
                              // 配置完后剩余字节可以做为数据存储器
/*
 下面是16个端点的定义
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
volatile far CTRL_TRF_SETUP SetupPkt;    //EP0端点进行配置的数据缓冲区
volatile far CTRL_TRF_DATA  CtrlTrfData; //数据收发缓冲区

// HID Buffer
volatile far unsigned char hid_report_out[HID_INT_OUT_EP_SIZE];  //设备接收数据
volatile far unsigned char hid_report_in[HID_INT_IN_EP_SIZE];    //设备发送数据
volatile far unsigned char hid_report_feature[EP0_BUFF_SIZE];    //特征值数据



#pragma romdata   //描述符定义在ROM区以数据形式存储

// 报表描述符
rom struct{byte report[HID_RPT01_SIZE];}hid_rpt01={  

	  	0x06, 0xA0, 0xFF,	// Usage page (vendor defined)，用来指定设备的功能
	  	0x09, 0x01,	        // Usage ID (vendor defined) 指定个别报表功能
	  	0xA1, 0x01,	        // Collection (application)  

		// 输入报表
        0x09, 0x03,     	// Usage ID - vendor defined
        0x15, 0x00,     	// Logical Minimum (0)  
        0x26, 0xFF, 0x00,   // Logical Maximum (255)
        0x75, 0x08,     	// Report Size (8 bits)
        0x95, 0x02,     	// Report Count (2 fields)
        0x81, 0x02,     	// Input (Data, Variable, Absolute)  

		// 输出报表
        0x09, 0x04,     	// Usage ID - vendor defined
        0x15, 0x00,     	// Logical Minimum (0)
        0x26, 0xFF, 0x00,   // Logical Maximum (255)
        0x75, 0x08,     	// Report Size (8 bits)
        0x95, 0x02,     	// Report Count (2 fields)
        0x91, 0x02,      	// Output (Data, Variable, Absolute)  

		// 特征报表
        0x09, 0x05,     	// Usage ID - vendor defined
        0x15, 0x00,     	// Logical Minimum (0)
        0x26, 0xFF, 0x00,   // Logical Maximum (255)
        0x75, 0x08,	        // Report Size (8 bits)
        0x95, 0x02, 		// Report Count (2 fields)
        0xB1, 0x02,     	// Feature (Data, Variable, Absolute)  

	  	0xC0};	            // end collection

// 设备描述符定义

rom USB_DEV_DSC device_dsc=
{
   sizeof(USB_DEV_DSC),   //描述符大小
   DSC_DEV,               //常数0x01
   0x0200,                //USB版本号码 2.0
   0x00,                  //类别码
   0x00,                  //子类别码
   0x00,                  //协议码
   EP0_BUFF_SIZE,         //端点0的最大信息包大小
   0xFFFF,                //厂商ID gcat
   0x1000,                //产品ID gcat
   0x0001,                //设备版本号
   0x01,                  //制造者的字符串描述符的索引值
   0x02,                  //产品的字符串描述符的索引值
   0x00,                  //序列号字符串描述符的索引值
   0x01,                  //设备支持的设置配置的数目，也就是配置描述符数目
}; 

// 描述符表

CFG01=
{
// 配置描述符
  sizeof(USB_CFG_DSC),   // 配置描述符大小
  DSC_CFG,               // 配置描述符类型
  sizeof(cfg01),         // 配置描述符传回的所有数据大小
  1,                     // 此配置支持的端口数目
  1,                      
  0,                     // 此配置字符串描述符的索引值 
  _DEFAULT|_RWU,         // 自备电源,支持远程唤醒
  50,                    // 向总线需求100MA电流
 
// 下面是配置描述符的附属描述符（包括接口和端点以及HID描述符）
// 接口描述符
  sizeof(USB_INTF_DSC),  // 接口描述符大小
  DSC_INTF,              // 接口描述符类型
  0,                     // 识别此接口的数字，bInterface
  0,                     // 选则一个替代设置的数值bAlternateInterface
  2,                     // 除端点0外支持的端点数目，只支持端点1
  HID_INTF,              // HID 类别码，只有HID在此给出其余是在配置中的类别码给出
  0,                     // 子类别码等于0 
  0,                     // 协议码
  0,                     // 此接口的字符串描述符的索引值  

//USB_HID_DSC是HID类别描述符
 sizeof(USB_HID_DSC),    // HID描述符大小
 DSC_HID,                // HID 
 0x0101,                 // HID规范版本号码 2.0
 0x00,                   // 硬件目的国家的识别数字
 HID_NUM_OF_DSC,         // 支持的附属子类别描述符的数目   
 DSC_RPT,                // 类别描述符的类别是报表类别
 sizeof(hid_rpt01),      // 报表描述符的总长度   

//HID的端点描述符，端点1 
 sizeof(USB_EP_DSC),	     // 描述符大小
   DSC_EP,			         // 常数=0x05
   _EP01_IN,			     // 端点1输入描述=0x81	
   _INT,					 // 中断传输，低2位有效 11中断传输，10批量传输，01实时传输，00控制传输
   HID_INT_IN_EP_SIZE,	     // 最大传输信息包大小8个字节
   0x0A,					 // 轮询端点的时距，以毫秒为单位
	
   sizeof(USB_EP_DSC),	     // 描述符大小
   DSC_EP,				     // 常数=0x05
   _EP01_OUT,			     // =0x01端点1输入描述	
   _INT,				     // 中断传输
   HID_INT_OUT_EP_SIZE,      // Maximum packet size  =8 最大传输信息包
   0x0A	                     // 轮询端点的时间MS为单位
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

rom const unsigned char *rom USB_CD_Ptr[]={&cfg01,&cfg01};        //ROM指针数组,存放cfg01地址
rom const unsigned char *rom USB_SD_Ptr[]={&sd000,&sd001,&sd002}; //ROM 指针数组,存放字符串地址

