
#define USE_USB_BUS_SENSE_IO
//-------------------------------USB�˵㶨��
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
//-------------------------------USB���ֱ�׼��������
#define GET_STATUS  0         // ��������һ���豸���ӿڻ�˵������״̬
#define CLR_FEATURE 1         // ���������ֹһ���豸���ӿڻ�˵��ϵ�����
#define SET_FEATURE 3         // ������������һ�����豸���ӿڻ�˵��ϵ�����
#define SET_ADR     5         // ����ָ��һ����ַ�����豸ͨ��
#define GET_DSC     6         // ��������һ��ָ����������
#define SET_DSC     7         // ��������һ�������������Ǹ���һ�����ڵ�������
#define GET_CFG     8         // ��������Ŀǰ�豸���õ���ֵ
#define SET_CFG     9         // ָʾ�豸ʹ��ѡ�������
#define GET_INTF    10        // ����豸֧�ֵ�����֧�ֶ������������õĶ˿ڣ���������Ŀǰ������
#define SET_INTF    11        // ����豸֧�ֶ������������õĽӿڣ����������豸ʹ��һ��ָ��������
#define SYNCH_FRAME 12        // �豸�뱨��һ���˵��ͬ��֡

#define DEVICE_REMOTE_WAKEUP    0x01   // �豸Զ�̻���
#define ENDPOINT_HALT           0x00   // �˵����

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
//-------------------------------USB���Ͷ���
/*
 MUID = Microchip USB Class ID
 ����ָ��usb���Ǹ����ͻ��EP0���ݵ���ͨ
*/
#define MUID_NULL               0  //��
#define MUID_USB9               1  //USB���ƴ���
#define MUID_HID                2  //HID 
#define MUID_CDC                3  //��������

//------------------------------- �˵�״̬����
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
//-------------------------------������������״̬����
/* Buffer Descriptor Status Register Initialization Parameters */
#define _BSTALL     0x04        //������ֹͣʼ��
#define _DTSEN      0x08        //��������ͬ��ʹ��
#define _INCDIS     0x10        //��ַ������ֹ
#define _KEN        0x20        //���ֱ�������Ȩʹ��
#define _DAT0       0x00        //���ڴ���һ��
#define _DAT1       0x40        //���ڴ���һ��
#define _DTSMASK    0x40        //���DTSENû��ʹ�ܣ�������1
#define _USIE       0x80        //SIEӵ�л�����
#define _UCPU       0x00        //CPUӵ�л�����

//-------------------------------���ƴ��䳣������
/* Control Transfer States */
// ���ƴ���״̬����
#define WAIT_SETUP          0   // �ȴ�״̬
#define CTRL_TRF_TX         1   // ���ƴ��䷢������״̬
#define CTRL_TRF_RX         2   // ���ƴ����������״̬

/* USB PID: Token Types - See chapter 8 in the USB specification */
//PID ����
#define SETUP_TOKEN         0b00001101   // �������豸�Ŀ��ƹܵ���������  
#define OUT_TOKEN           0b00000001   // �������豸����������
#define IN_TOKEN            0b00001001   // �豸����������������

/* bmRequestType Definitions */
#define HOST_TO_DEV         0   //�������豸�����ݷ���
#define DEV_TO_HOST         1   //�豸������ (���ݷ���) 

#define STANDARD            0x00   // ��׼���
#define CLASS               0x01   // �ض�USB����壬HID������������һ��
#define VENDOR              0x02   // �����Զ�������

#define RCPT_DEV            0
#define RCPT_INTF           1
#define RCPT_EP             2
#define RCPT_OTH            3

//-------------------------------HID��������
/* HID */
#define HID_INTF_ID             0x00    //HID�ӿڣɣĺ�
#define HID_UEP                 UEP1  
#define HID_BD_OUT              ep1Bo   //HID�������˵�
#define HID_INT_OUT_EP_SIZE     8       //��������˵㻺������С
#define HID_BD_IN               ep1Bi   //HID��������˵�
#define HID_INT_IN_EP_SIZE      8       //�������˵㻺������С
#define HID_NUM_OF_DSC          1       //HID����������
#define HID_RPT01_SIZE          47      //HID������������С

#define HID_INTF                    0x03

#define HID_INPUT_REPORT_BYTES   2      //��������С
#define HID_OUTPUT_REPORT_BYTES  2      //���뱨���С
#define HID_FEATURE_REPORT_BYTES 2      //���������С

/* Class Descriptor Types */
#define DSC_HID         0x21
#define DSC_RPT         0x22
#define DSC_PHY         0x23

// HID�ض�����
#define GET_REPORT      0x01
#define GET_IDLE        0x02
#define GET_PROTOCOL    0x03
#define SET_REPORT      0x09
#define SET_IDLE        0x0A
#define SET_PROTOCOL    0x0B


//--------------------------------�˵����Լ�EP0�������ֽ�������
#define EP0_BUFF_SIZE      8   // 8, 16, 32, or 64
#define MAX_NUM_INT        1   // Ϊ���ٽ������ã��ж϶˵�������

#define MAX_EP_NUMBER      1   // �˵�Ŷ��壬�����õ��˵�0��1

//--------------------------------�˵�N�ĳ�ʼ�������� 
#define EP_CTRL     0x06   // ����SETUP���������������
#define EP_OUT      0x0C   // ����ʹ�����
#define EP_IN       0x0A   // ����ʹ������
#define EP_OUT_IN   0x0E   // ����ʹ���������
#define HSHK_EN     0x10   // ʹ�������ź�

#define _RAM        0      // ������������
#define _ROM        1

//---------------------------------USB����Ͽ���־
 #define USB_BUS_ATTACHED    1   //USB�ϵ� 
 #define USB_BUS_DETACHED    0   //USB�ϵ�


/* USB Device States - To be used with [byte usb_device_state]
USB�豸״̬���壬����Ӧ�ôﵽCONFIGURED_STATE״̬����USB�������������
 */
#define DETACHED_STATE          0   //����״̬
#define ATTACHED_STATE          1   //����״̬
#define POWERED_STATE           2   //ʹ��״̬
#define DEFAULT_STATE           3   //ȱʡ״̬
#define ADR_PENDING_STATE       4   //��ַδ��״̬
#define ADDRESS_STATE           5   //��ַ����״̬
#define CONFIGURED_STATE        6   //����״̬

//************************************************************************
#define _PPBM0      0x00            // ƹ�һ���ģʽ0
#define _PPBM1      0x01            // ƹ�һ���ģʽ1
#define _PPBM2      0x02            // ƹ�һ���ģʽ2
#define _LS         0x00            // ����USBģʽ
#define _FS         0x04            // ȫ��USBģʽ
#define _TRINT      0x00            // ʹ���ڲ��շ���
#define _TREXT      0x08            // ʹ���ⲿ�շ���
#define _PUEN       0x10            // ʹ���ڲ���������
#define _OEMON      0x40            // ʹ��OE�źż�����
#define _UTEYE      0x80            // ʹ����ͼ


//***********************************************************************
#define MODE_PP                 _PPBM0
#define UCFG_VAL                _PUEN|_TRINT|_FS|MODE_PP  //USB���üĴ�������


//--------------------------------------�������Ͷ���
typedef unsigned char   byte;           // 8-bit
typedef unsigned int    word;           // 16-bit
typedef unsigned long   dword;          // 32-bit

//-----------------------------------USB�豸״̬����
typedef union _USB_DEVICE_STATUS
     {
          byte _byte;
          struct
             {
                unsigned RemoteWakeup:1;// [0]Disabled [1]Enabled: See usbdrv.c,usb9.c
                unsigned ctrl_trf_mem:1;// [0]RAM      [1]ROM
             };
     } USB_DEVICE_STATUS; 
     
//-----------------------------------�ֽ�λ��������        
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

//------------------------------------�ֶ�������
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
#define LSB(a)      ((a).v[0])  //������ֽڣ������WORD�͵Ŀ���ֱ�����������ߵ��ֽڣ�
#define MSB(a)      ((a).v[1])  //������ֽ�

//-------------------------------------ָ�����Ͷ��壨˫�ֽڵģ�
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


//----------------------------------USB�������������Ͷ���

typedef union _BD_STAT   //����������״̬�Ĵ���λ����
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
        unsigned PID:4;                 // PID�����壬ռ4λ
        unsigned :2;
    };
} BD_STAT;                              // ��������״̬�Ĵ���

typedef union _BDT
{
    struct
    {
        BD_STAT Stat;    //����������״̬
        byte Cnt;        //�����������ֽڼ���
        byte ADRL;       //��������ַ���ֽ�
        byte ADRH;       //��������ַ���ֽ�
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        byte* ADR;       //��������ַ��ADRռ�����ֽڣ���������һ���ֽ�
    };
} BDT;                   //������������


//----------------------------���ƴ������ñ�׼�����ֽ����Ͷ���
typedef union _CTRL_TRF_SETUP
{
    //  ���м�ӵ�ַ
    struct
    {
        byte _byte[EP0_BUFF_SIZE];   //#define EP0_BUFF_SIZE 8  ,8���ֽڵ�EP0������
    };
    
    // ��׼���豸������USB�����豸ʱ�õ��ĸ���λ���涼�ж��壬�����USB��׼����
    struct
    {
        byte bmRequestType; //�������ݵĸ�ʽ����׼����������
        byte bRequest;      //һ���ֽ�����ָ������
        word wValue;        //��������������Ϣ���豸��ÿһ�����󶼿����Լ������������ֽڵ�����
        word wIndex;        //��������������Ϣ���豸һ��������������ֵ��ƫ��
        word wLength;       //�������ݽ׶��н������������ֽڵ���Ŀ������������豸�Ĵ��䣬��ʾ�����豸�������ȷ�ֽ���Ŀ
                            //������豸�������Ĵ��䣬����һ�����ֵ�������0����ʾû�����ݽ׶�
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
        unsigned Recipient:5;    //�豸 �ӿ� �˵� �����������߶��壬һ����00�豸����
        unsigned RequestType:2;  //�Ƿ��Ǳ�׼����ı�־
        unsigned DataDir:1;      //��־ �������豸�����豸������
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
        byte bDscIndex;           //������ֵ
        byte bDscType;            //Device,Configuration,String�����������ã��ַ���������������
        word wLangID;             //Language ID������ID��
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        BYTE bDevADR;              //Device Address 0-127��������ַ
        byte bDevADRH;             //Must equal zero       ����=0
        unsigned :8;
        unsigned :8;
        unsigned :8;
        unsigned :8;
    };
    struct
    {
        unsigned :8;
        unsigned :8;
        byte bCfgValue;             //Configuration Value 0-255,����ȡֵ��Χ
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
        byte bIntfID;                //Interface Number Value 0-255�ӿ�ֵ
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
    //��׼�豸��������ݶ���
    
} CTRL_TRF_SETUP;


//----------------------------------USB�������Ͷ���

typedef union _CTRL_TRF_DATA  //��8���ֽ�
{
    /** Array for indirect addressing ****************************************/
    struct
    {
        byte _byte[EP0_BUFF_SIZE];
    };
    
    // 8���ֽڵļ�ӵ�ַ ***************************************/
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


//********************USB ����������***********************
/************** USB ����������************** 
************************ ******************/

//1.USB�豸���������Ͷ���
typedef struct _USB_DEV_DSC
{
    byte bLength;       //��������С   
    byte bDscType;      //���� 0x01 
    word bcdUSB;        //USB �汾���� 1.0��0100H��1.1��0110H��2.0��0200H
    byte bDevCls;       //�����
    byte bDevSubCls;    //�����
    byte bDevProtocol;  //Э����
    byte bMaxPktSize0;  //�˵�0�������Ϣ����С 
    word idVendor;      //����ID
    word idProduct;     //��ƷID
    word bcdDevice;     //�豸�汾���� ��BCD��
    byte iMFR;          //�����ߵ��ַ���������������ֵ
    byte iProduct;      //��Ʒ���ַ���������������ֵ
    byte iSerialNum;    //��ŵ��ַ���������������ֵ
    byte bNumCfg;       //�������õ���Ŀ
} USB_DEV_DSC;
// end  USB_DEV_DSC

//2.USB��������������
//ÿһ�����������������丽������,����һ�������ӿ�������,�Լ�ѡ���ԵĶ˵�������
typedef struct _USB_CFG_DSC
{
    byte bLength;       // ��������С          
    byte bDscType;      // ���� 0x02
    word wTotalLength;  // �����÷����ص��������ݴ�С(�ֽ�)
    byte bNumIntf;      // ������֧�ֵĽӿ���Ŀ 
    byte bCfgValue;     // SET_Configuration��Get_Configuration����ı�ʶ����
                        // ���SET_Configurationq����ֵΪ0���豸�����������״̬  
    byte iCfg;          // �����õ��ַ���������
    byte bmAttributes;  // λͼ�����Դ/���ߵ�Դ�Լ�Զ�̻�������
    byte bMaxPower;     // ��Ҫ���ߵ�Դ����ʾ��Ϊ��������/2��
} USB_CFG_DSC;
// end  USB_CFG_DSC

//3.USB�ӿ�����������
typedef struct _USB_INTF_DSC
{
    byte bLength;       // ��������С
    byte bDscType;      // ����0x04
    byte bIntfNum;      // ʶ��˽ӿڵ�����
    byte bAltSetting;   // ����ѡ��һ��������õ���ֵ
    byte bNumEPs;       // ���˶˵�0�⣬֧�ֵĶ˵���Ŀ
    byte bIntfCls;      // �����  
    byte bIntfSubCls;   // �������
    byte bIntfProtocol; // Э���� 
    byte iIntf;         // �˽ӿڵ��ַ���������������ֵ 
} USB_INTF_DSC;

//4.USB �˵�������
typedef struct _USB_EP_DSC
{
    byte bLength;       // ��������С 
    byte bDscType;      // ����0x05
    byte bEPAdr;        // �˵���Ŀ�뷽��
    byte bmAttributes;  // ֧�ֵĴ�������
    word wMaxPktSize;   // ֧�ֵ������Ϣ����С
    byte bInterval;     // ����ӳ�/��ѯʱ��/NAK����
} USB_EP_DSC;

//5.HID
//����HID���������������ظ��ֽڣ���󲿷ֿ�ѡ����ʹ�ö�����������豸��
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
    USB_HID_HEADER header[1];    //����ֻʹ��һ��   
} USB_HID_DSC;

//������������������壨����������������
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
