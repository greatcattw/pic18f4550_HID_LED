
#define USE_USB_BUS_SENSE_IO
//-------------------------------USB端点定义
#define _EP01_OUT   0x01
#define _EP01_IN    0x81
#define _EP02_OUT   0x02
#define _EP02_IN    0x82
#define _EP03_OUT   0x03
#define _EP03_IN    0x83
#define _EP04_OUT   0x04
#define _EP04_IN    0x84
#define _EP05_OUT   0x05
#define _EP05_IN    0x85
#define _EP06_OUT   0x06
#define _EP06_IN    0x86
#define _EP07_OUT   0x07
#define _EP07_IN    0x87
#define _EP08_OUT   0x08
#define _EP08_IN    0x88
#define _EP09_OUT   0x09
#define _EP09_IN    0x89
#define _EP10_OUT   0x0A
#define _EP10_IN    0x8A
#define _EP11_OUT   0x0B
#define _EP11_IN    0x8B
#define _EP12_OUT   0x0C
#define _EP12_IN    0x8C
#define _EP13_OUT   0x0D
#define _EP13_IN    0x8D
#define _EP14_OUT   0x0E
#define _EP14_IN    0x8E
#define _EP15_OUT   0x0F
#define _EP15_IN    0x8F
//-------------------------------USB各种标准请求类型
#define GET_STATUS  0         // 主机请求一个设备、接口或端点的特性状态
#define CLR_FEATURE 1         // 主机请求禁止一个设备、接口或端点上的特征
#define SET_FEATURE 3         // 主机请求启用一个在设备、接口或端点上的特征
#define SET_ADR     5         // 主机指定一个地址来与设备通信
#define GET_DSC     6         // 主机请求一个指定的描述符
#define SET_DSC     7         // 主机新增一个描述符，或是更新一个存在的描述符
#define GET_CFG     8         // 主机请求目前设备配置的数值
#define SET_CFG     9         // 指示设备使用选择的配置
#define GET_INTF    10        // 如果设备支持的配置支持多个互不相关设置的端口，主机请求目前的设置
#define SET_INTF    11        // 如果设备支持多个互不相关设置的接口，主机请求设备使用一个指定的设置
#define SYNCH_FRAME 12        // 设备与报告一个端点的同步帧

#define DEVICE_REMOTE_WAKEUP    0x01   // 设备远程唤醒
#define ENDPOINT_HALT           0x00   // 端点挂起

/* Endpoint Transfer Type */
#define _CTRL       0x00            //Control Transfer
#define _ISO        0x01            //Isochronous Transfer
#define _BULK       0x02            //Bulk Transfer
#define _INT        0x03            //Interrupt Transfer

/* Descriptor Types */
#define DSC_DEV     0x01
#define DSC_CFG     0x02
#define DSC_STR     0x03
#define DSC_INTF    0x04
#define DSC_EP      0x05
/* Configuration Attributes */
#define _DEFAULT    0x01<<7         //Default Value (Bit 7 is set)
#define _SELF       0x01<<6         //Self-powered (Supports if set)
#define _RWU        0x01<<5         //Remote Wakeup (Supports if set)
//-------------------------------USB类型定义
/*
 MUID = Microchip USB Class ID
 用来指明usb的那个类型获得EP0数据的流通
*/
#define MUID_NULL               0  //无
#define MUID_USB9               1  //USB控制传输
#define MUID_HID                2  //HID 
#define MUID_CDC                3  //海量传输

//------------------------------- 端点状态定义
#define OUT         0
#define IN          1

#define PIC_EP_NUM_MASK 0b01111000
#define PIC_EP_DIR_MASK 0b00000100

#define EP00_OUT    ((0x00<<3)|(OUT<<2))
#define EP00_IN     ((0x00<<3)|(IN<<2))
#define EP01_OUT    ((0x01<<3)|(OUT<<2))
#define EP01_IN     ((0x01<<3)|(IN<<2))
#define EP02_OUT    ((0x02<<3)|(OUT<<2))
#define EP02_IN     ((0x02<<3)|(IN<<2))
#define EP03_OUT    ((0x03<<3)|(OUT<<2))
#define EP03_IN     ((0x03<<3)|(IN<<2))
#define EP04_OUT    ((0x04<<3)|(OUT<<2))
#define EP04_IN     ((0x04<<3)|(IN<<2))
#define EP05_OUT    ((0x05<<3)|(OUT<<2))
#define EP05_IN     ((0x05<<3)|(IN<<2))
#define EP06_OUT    ((0x06<<3)|(OUT<<2))
#define EP06_IN     ((0x06<<3)|(IN<<2))
#define EP07_OUT    ((0x07<<3)|(OUT<<2))
#define EP07_IN     ((0x07<<3)|(IN<<2))
#define EP08_OUT    ((0x08<<3)|(OUT<<2))
#define EP08_IN     ((0x08<<3)|(IN<<2))
#define EP09_OUT    ((0x09<<3)|(OUT<<2))
#define EP09_IN     ((0x09<<3)|(IN<<2))
#define EP10_OUT    ((0x0A<<3)|(OUT<<2))
#define EP10_IN     ((0x0A<<3)|(IN<<2))
#define EP11_OUT    ((0x0B<<3)|(OUT<<2))
#define EP11_IN     ((0x0B<<3)|(IN<<2))
#define EP12_OUT    ((0x0C<<3)|(OUT<<2))
#define EP12_IN     ((0x0C<<3)|(IN<<2))
#define EP13_OUT    ((0x0D<<3)|(OUT<<2))
#define EP13_IN     ((0x0D<<3)|(IN<<2))
#define EP14_OUT    ((0x0E<<3)|(OUT<<2))
#define EP14_IN     ((0x0E<<3)|(IN<<2))
#define EP15_OUT    ((0x0F<<3)|(OUT<<2))
#define EP15_IN     ((0x0F<<3)|(IN<<2))
//-------------------------------缓冲期描述符状态定义
/* Buffer Descriptor Status Register Initialization Parameters */
#define _BSTALL     0x04        //缓冲器停止始能
#define _DTSEN      0x08        //数据套索同步使能
#define _INCDIS     0x10        //地址递增禁止
#define _KEN        0x20        //保持保存所有权使能
#define _DAT0       0x00        //包期待下一个
#define _DAT1       0x40        //包期待下一个
#define _DTSMASK    0x40        //如果DTSEN没有使能，保持置1
#define _USIE       0x80        //SIE拥有缓冲区
#define _UCPU       0x00        //CPU拥有缓冲区

//-------------------------------控制传输常量定义
/* Control Transfer States */
// 控制传输状态定义
#define WAIT_SETUP          0   // 等待状态
#define CTRL_TRF_TX         1   // 控制传输发送数据状态
#define CTRL_TRF_RX         2   // 控制传输接收数据状态

/* USB PID: Token Types - See chapter 8 in the USB specification */
//PID 令牌
#define SETUP_TOKEN         0b00001101   // 主机到设备的控制管道设置事务  
#define OUT_TOKEN           0b00000001   // 主机到设备的数据事务
#define IN_TOKEN            0b00001001   // 设备到主机的数据事务

/* bmRequestType Definitions */
#define HOST_TO_DEV         0   //主机到设备（数据方向）
#define DEV_TO_HOST         1   //设备到主机 (数据方向) 

#define STANDARD            0x00   // 标准类别
#define CLASS               0x01   // 特定USB类别定义，HID的请求属于这一类
#define VENDOR              0x02   // 厂商自定义请求

#define RCPT_DEV            0
#define RCPT_INTF           1
#define RCPT_EP             2
#define RCPT_OTH            3

//-------------------------------HID常量定义
/* HID */
#define HID_INTF_ID             0x00    //HID接口ＩＤ号
#define HID_UEP                 UEP1  
#define HID_BD_OUT              ep1Bo   //HID输出事务端点
#define HID_INT_OUT_EP_SIZE     8       //输入事务端点缓冲区大小
#define HID_BD_IN               ep1Bi   //HID输入事务端点
#define HID_INT_IN_EP_SIZE      8       //输出事务端点缓冲区大小
#define HID_NUM_OF_DSC          1       //HID描述符个数
#define HID_RPT01_SIZE          47      //HID报表描述符大小

#define HID_INTF                    0x03

#define HID_INPUT_REPORT_BYTES   2      //输出报表大小
#define HID_OUTPUT_REPORT_BYTES  2      //输入报表大小
#define HID_FEATURE_REPORT_BYTES 2      //特征报表大小

/* Class Descriptor Types */
#define DSC_HID         0x21
#define DSC_RPT         0x22
#define DSC_PHY         0x23

// HID特定请求
#define GET_REPORT      0x01
#define GET_IDLE        0x02
#define GET_PROTOCOL    0x03
#define SET_REPORT      0x09
#define SET_IDLE        0x0A
#define SET_PROTOCOL    0x0B


//--------------------------------端点数以及EP0缓冲区字节数定义
#define EP0_BUFF_SIZE      8   // 8, 16, 32, or 64
#define MAX_NUM_INT        1   // 为跟踪交替设置，中断端点最大个数

#define MAX_EP_NUMBER      1   // 端点号定义，本例用到端点0和1

//--------------------------------端点N的初始化配置字 
#define EP_CTRL     0x06   // 配置SETUP事务并是能输入输出
#define EP_OUT      0x0C   // 配置使能输出
#define EP_IN       0x0A   // 配置使能输入
#define EP_OUT_IN   0x0E   // 配置使能输入输出
#define HSHK_EN     0x10   // 使能握手信号

#define _RAM        0      // 数据区区别定义
#define _ROM        1

//---------------------------------USB接入断开标志
 #define USB_BUS_ATTACHED    1   //USB上电 
 #define USB_BUS_DETACHED    0   //USB断点


/* USB Device States - To be used with [byte usb_device_state]
USB设备状态定义，最终应该达到CONFIGURED_STATE状态，算USB与主机配置完成
 */
#define DETACHED_STATE          0   //分离状态
#define ATTACHED_STATE          1   //接入状态
#define POWERED_STATE           2   //使能状态
#define DEFAULT_STATE           3   //缺省状态
#define ADR_PENDING_STATE       4   //地址未决状态
#define ADDRESS_STATE           5   //地址配置状态
#define CONFIGURED_STATE        6   //配置状态

//************************************************************************
#define _PPBM0      0x00            // 乒乓缓冲模式0
#define _PPBM1      0x01            // 乒乓缓冲模式1
#define _PPBM2      0x02            // 乒乓缓冲模式2
#define _LS         0x00            // 低速USB模式
#define _FS         0x04            // 全速USB模式
#define _TRINT      0x00            // 使用内部收发器
#define _TREXT      0x08            // 使用外部收发器
#define _PUEN       0x10            // 使用内部上拉电阻
#define _OEMON      0x40            // 使用OE信号监视器
#define _UTEYE      0x80            // 使能眼图


//***********************************************************************
#define MODE_PP                 _PPBM0
#define UCFG_VAL                _PUEN|_TRINT|_FS|MODE_PP  //USB配置寄存器设置


//--------------------------------------数据类型定义
typedef unsigned char   byte;           // 8-bit
typedef unsigned int    word;           // 16-bit
typedef unsigned long   dword;          // 32-bit

//-----------------------------------USB设备状态类型
typedef union _USB_DEVICE_STATUS
     {
          byte _byte;
          struct
             {
                unsigned RemoteWakeup:1;// [0]Disabled [1]Enabled: See usbdrv.c,usb9.c
                unsigned ctrl_trf_mem:1;// [0]RAM      [1]ROM
             };
     } USB_DEVICE_STATUS; 
     
//-----------------------------------字节位定义类型        
typedef union _BYTE
{
    byte _byte;
    struct
    {
        unsigned b0:1;
        unsigned b1:1;
        unsigned b2:1;
        unsigned b3:1;
        unsigned b4:1;
        unsigned b5:1;
        unsigned b6:1;
        unsigned b7:1;
    };
} BYTE;

//------------------------------------字定义类型
typedef union _WORD
{
    word _word;
    struct
    {
        byte byte0;
        byte byte1;
    };
    struct
    {
        BYTE Byte0;
        BYTE Byte1;
    };
    struct
    {
        BYTE LowB;
        BYTE HighB;
    };
    struct
    {
        byte v[2];
    };
} WORD;
#define LSB(a)      ((a).v[0])  //定义低字节（如果是WORD型的可以直接用这个替代高低字节）
#define MSB(a)      ((a).v[1])  //定义高字节

//-------------------------------------指针类型定义（双字节的）
typedef union _POINTER
{
    struct
    {
        byte bLow;
        byte bHigh;
        //byte bUpper;
    };
    word _word;                         // bLow & bHigh
    
    //pFunc _pFunc;                     // Usage: ptr.pFunc(); Init: ptr.pFunc = &<Function>;

    byte* bRam;                         // Ram byte pointer: 2 bytes pointer pointing
                                        // to 1 byte of data
    word* wRam;                         // Ram word poitner: 2 bytes poitner pointing
                                        // to 2 bytes of data

    rom byte* bRom;                     // Size depends on compiler setting
    rom word* wRom;
    //rom near byte* nbRom;             // Near = 2 bytes pointer
    //rom near word* nwRom;
    //rom far byte* fbRom;              // Far = 3 bytes pointer
    //rom far word* fwRom;
} POINTER;


//----------------------------------USB缓冲描述符类型定义

typedef union _BD_STAT   //缓冲描述符状态寄存器位定义
{
    byte _byte;
    struct{
        unsigned BC8:1;
        unsigned BC9:1;
        unsigned BSTALL:1;          //Buffer Stall Enable
        unsigned DTSEN:1;           //Data Toggle Synch Enable
        unsigned INCDIS:1;          //Address Increment Disable
        unsigned KEN:1;             //BD Keep Enable
        unsigned DTS:1;             //Data Toggle Synch Value
        unsigned UOWN:1;            //USB Ownership
    };
    struct{
        unsigned BC8:1;
        unsigned BC9:1;
        unsigned PID0:1;
        unsigned PID1:1;
        unsigned PID2:1;
        unsigned PID3:1;
        unsigned :1;
        unsigned UOWN:1;
    };
    struct{
        unsigned :2;
        unsigned PID:4;                 // PID包定义，占4位
        unsigned :2;
    };
} BD_STAT;                              // 缓冲描述状态寄存器

typedef union _BDT
{
    struct
    {
        BD_STAT Stat;    //缓冲描述符状态
        byte Cnt;        //缓冲描述符字节计数
        byte ADRL;       //缓冲区地址低字节
        byte ADRH;       //缓冲区地址高字节
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        byte* ADR;       //缓冲区地址，ADR占两个字节，但数据是一个字节
    };
} BDT;                   //缓冲描述符表


//----------------------------控制传输配置标准数据字节类型定义
typedef union _CTRL_TRF_SETUP
{
    //  排列间接地址
    struct
    {
        byte _byte[EP0_BUFF_SIZE];   //#define EP0_BUFF_SIZE 8  ,8个字节的EP0缓冲区
    };
    
    // 标准的设备请求，在USB配置设备时用到的各个位下面都有定义，具体见USB标准请求
    struct
    {
        byte bmRequestType; //设置数据的格式（标准的数据请求）
        byte bRequest;      //一个字节用来指定请求
        word wValue;        //主机用来传输信息给设备。每一个请求都可以自己定义这两个字节的意义
        word wIndex;        //主机用来传送信息给设备一般用来传送索引值或偏移
        word wLength;       //包含数据阶段中接下来的数据字节的数目，如果是主机设备的传输，表示主机设备传输的正确字节数目
                            //如果是设备到主机的传输，则是一个最大值，如果是0，表示没有数据阶段
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        WORD W_Value;
        WORD W_Index;
        WORD W_Length;
    };
    struct
    {
        unsigned Recipient:5;    //设备 接口 端点 其他，接收者定义，一般是00设备接收
        unsigned RequestType:2;  //是否是标准请求的标志
        unsigned DataDir:1;      //标志 主机到设备还是设备到主机
        unsigned :8;
        byte bFeature;           //DEVICE_REMOTE_WAKEUP,ENDPOINT_HALT
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        byte bDscIndex;           //描述符值
        byte bDscType;            //Device,Configuration,String，驱动，配置，字符串，描述符类型
        word wLangID;             //Language ID，语言ID号
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        BYTE bDevADR;              //Device Address 0-127，驱动地址
        byte bDevADRH;             //Must equal zero       必须=0
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        byte bCfgValue;             //Configuration Value 0-255,配置取值范围
        byte bCfgRSD;               //Must equal zero (Reserved)
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        byte bAltID;                 //Alternate Setting Value 0-255
        byte bAltID_H;               //Must equal zero
        byte bIntfID;                //Interface Number Value 0-255接口值
        byte bIntfID_H;              //Must equal zero
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned :8;
        byte bEPID;                   //Endpoint ID (Number & Direction)
        byte bEPID_H;                 //Must equal zero
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned EPNum:4;             //Endpoint Number 0-15
        unsigned :3;
        unsigned EPDir:1;            //Endpoint Direction: 0-OUT, 1-IN
        unsigned :8;
        unsigned :8;
        unsigned :8;
    };
    //标准设备请求的数据定义
    
} CTRL_TRF_SETUP;


//----------------------------------USB数据类型定义

typedef union _CTRL_TRF_DATA  //共8个字节
{
    /** Array for indirect addressing ****************************************/
    struct
    {
        byte _byte[EP0_BUFF_SIZE];
    };
    
    // 8个字节的间接地址 ***************************************/
    struct
    {
        byte _byte0;
        byte _byte1;
        byte _byte2;
        byte _byte3;
        byte _byte4;
        byte _byte5;
        byte _byte6;
        byte _byte7;
    };
    struct
    {
        word _word0;
        word _word1;
        word _word2;
        word _word3;
    };

} CTRL_TRF_DATA;


//********************USB 描述符定义***********************
/************** USB 描述符定义************** 
************************ ******************/

//1.USB设备描述符类型定义
typedef struct _USB_DEV_DSC
{
    byte bLength;       //描述符大小   
    byte bDscType;      //常数 0x01 
    word bcdUSB;        //USB 版本号码 1.0是0100H，1.1是0110H，2.0是0200H
    byte bDevCls;       //类别码
    byte bDevSubCls;    //子类别
    byte bDevProtocol;  //协议码
    byte bMaxPktSize0;  //端点0的最大信息包大小 
    word idVendor;      //厂商ID
    word idProduct;     //产品ID
    word bcdDevice;     //设备版本号码 （BCD）
    byte iMFR;          //制造者的字符串描述符的索引值
    byte iProduct;      //产品的字符串描述符的索引值
    byte iSerialNum;    //序号的字符串描述符的索引值
    byte bNumCfg;       //可能配置的数目
} USB_DEV_DSC;
// end  USB_DEV_DSC

//2.USB配置描述符定义
//每一个配置描述符都有其附属属性,包括一个或多个接口描述符,以及选择性的端点描述符
typedef struct _USB_CFG_DSC
{
    byte bLength;       // 描述符大小          
    byte bDscType;      // 常数 0x02
    word wTotalLength;  // 此配置符传回的所有数据大小(字节)
    byte bNumIntf;      // 此配置支持的接口数目 
    byte bCfgValue;     // SET_Configuration与Get_Configuration请求的标识符。
                        // 如果SET_Configurationq请求值为0，设备会进入无配置状态  
    byte iCfg;          // 此配置的字符串描述符
    byte bmAttributes;  // 位图自身电源/总线电源以及远程唤醒设置
    byte bMaxPower;     // 需要总线电源，表示法为（最大毫安/2）
} USB_CFG_DSC;
// end  USB_CFG_DSC

//3.USB接口描述符定义
typedef struct _USB_INTF_DSC
{
    byte bLength;       // 描述符大小
    byte bDscType;      // 常数0x04
    byte bIntfNum;      // 识别此接口的数字
    byte bAltSetting;   // 用来选择一个替代设置的数值
    byte bNumEPs;       // 除了端点0外，支持的端点数目
    byte bIntfCls;      // 类别码  
    byte bIntfSubCls;   // 子类别码
    byte bIntfProtocol; // 协议码 
    byte iIntf;         // 此接口的字符串描述符的索引值 
} USB_INTF_DSC;

//4.USB 端点描述符
typedef struct _USB_EP_DSC
{
    byte bLength;       // 描述符大小 
    byte bDscType;      // 常数0x05
    byte bEPAdr;        // 端点数目与方向
    byte bmAttributes;  // 支持的传输类型
    word wMaxPktSize;   // 支持的最大信息包大小
    byte bInterval;     // 最大延迟/轮询时距/NAK速率
} USB_EP_DSC;

//5.HID
//由于HID类别描述符最后有重复字节，最后部分可选（在使用多个描述符的设备）
typedef struct _USB_HID_HEADER
{
   byte  bDescriptorType;
   word  wDescriptorLength;
}USB_HID_HEADER;

typedef struct _USB_HID_DSC
{
    byte bLength;            
    byte bDscType;  
    word bcdHID;
    byte bCountryCode;
    byte bNumDsc;
    USB_HID_HEADER header[1];    //此例只使用一个   
} USB_HID_DSC;

//这个是配置描述符定义（包含附属描述符）
#define CFG01 rom struct             \
{   USB_CFG_DSC     cd01;            \
    USB_INTF_DSC    i00a00;          \
    USB_HID_DSC     hid_i00a00;      \
    USB_EP_DSC      ep01i_i00a00;    \
    USB_EP_DSC      ep01o_i00a00;    \
} cfg01

#define mUSBBufferReady(buffer_dsc)                                         \
{                                                                           \
  buffer_dsc.Stat._byte &= _DTSMASK;          /* Save only DTS bit */     \
  buffer_dsc.Stat.DTS = !buffer_dsc.Stat.DTS; /* Toggle DTS bit    */     \
  buffer_dsc.Stat._byte |= _USIE|_DTSEN;      /* Turn ownership to SIE */ \
}


#define mUSBBufferReady(buffer_dsc)                                         \
{                                                                           \
  buffer_dsc.Stat._byte &= _DTSMASK;          /* Save only DTS bit */     \
  buffer_dsc.Stat.DTS = !buffer_dsc.Stat.DTS; /* Toggle DTS bit    */     \
  buffer_dsc.Stat._byte |= _USIE|_DTSEN;      /* Turn ownership to SIE */ \
}
