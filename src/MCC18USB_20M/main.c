/*
����18F4550��HID�������
�˳���ֻ��ɴ���������2�ֽ����ݣ�Ȼ���豸������ԭ�ⲻ����������
��������MCC18 
�汾��MPLAB-C18-v3_02
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Zhongxin An        2007.7.3        HID 
*/
#include <p18cxxx.h>
#include "typedefint.h"
#include "USb data.h"

#include "cfg_bit.h"

#define USB_sense  PORTAbits.RA1  //USB �ϵ���˿�
#define tris_usb_bus_sense TRISAbits.TRISA1


/**************************
* ��������
***************************/
#pragma udata
//---------������������---------
void InitializeSystem(void);//ϵͳ��ʼ��
void USBTasks(void);  //USB����
void ProcessIO(void); //�û����� 

void mInitializeUSBDriver(void);//��ʼ��USB�豸
void UserInit(void);  //�û���ʼ������
void ClearArray(byte* startAdr,byte count); //��ַ��0����
void mDisableEP1to15(void);  //�˵���0����
//------------------------------

//--------USB���ƴ��亯����--------
void USBPrepareForNextSetupTrf(void);  // Ϊ�´�USB���ƴ���SETUP��׼��
void USBCtrlEPService(void);           // USB ���ƶ˵��������������ж��е���  

void USBCtrlTrfSetupHandler(void); // USB���ƴ������ô����� 
void USBCtrlTrfOutHandler(void);   // USB���������������������-�豸��
void USBCtrlTrfInHandler(void);    // USB�����������ݣ������豸-������

void USBCtrlEPServiceComplete(void); // USB���ƴ���˵������� 

void USBCtrlTrfTxService(void);      // USB���ƴ��������豸������
void USBCtrlTrfRxService(void);      // USB����������������������豸

//---------------------------------

//--------USB Э�麯����-----------
void USBCheckStdRequest(void);     // ���USB�ı�׼����

void USBStdGetDscHandler(void);    // USB��ñ�׼������������
void USBStdSetCfgHandler(void);    // USBָʾ�豸ʹ����ѡ������ô���
void USBStdGetStatusHandler(void); // USB�������״̬������
void USBStdFeatureReqHandler(void);// ����������������ֵ������

void mUSBCheckAdrPendingState(void); // USB����ַ״̬�Ƿ��Ѿ�����  
//---------------------------------

//-----------HID ������-------------
void HIDInitEP(void);              // HID�˵����ú���
void USBCheckHIDRequest(void);     // USB HID������

void HIDGetReportHandler(void);    // ������ñ�������
void HIDSetReportHandler(void);    // �豸��ñ�������

byte HIDRxReport(char *buffer, byte len); // HIDȡ���������ݳ��Ⱥ��� 
void HIDTxReport(char *buffer, byte len); // HID���͵���������
//----------------------------------

//-----------USB�û�ר����----------
void GetInputReport0(void);        // �����������
void GetFeatureReport0(void);      // �����������

void HandleControlOutReport(void); // HID���ͱ����������ͣ�
void ReportLoopback(void);         // b�����ѯ����

void user(void);                   // �û��Զ��庯��
//---------------------------------- 

//-------USB �����Լ��豸������-------
void USBCheckBusStatus(void);      // USB���߼�⺯��
void USBDriverService(void);       // USB�豸�������
//--------------------------------------

extern void _startup (void);       // See c018i.c in your C18 compiler dir
#pragma code _RESET_INTERRUPT_VECTOR = 0x000800  //��λ
void _reset (void)
{
    _asm goto _startup _endasm
}
#pragma code

#pragma code
/***************************
* ������
****************************/
void main()
{
  InitializeSystem();   //ϵͳ��ʼ������
  while(1)
   {
     USBTasks();        //USB ������
  
     ProcessIO();       // 
   }
}//end main()

//------------------------------------------------------------�������Ӻ�����
/***************************
* ϵͳ��ʼ������
****************************/
void InitializeSystem()
{
  WDTCONbits.SWDTEN=0;    //��ֹ���Ź�
  ADCON1=0x0f;            //��ģ��˿�ȫ������Ϊ���ֶ˿�
  HLVDCONbits.HLVDEN=0;   //��ֹ��ѹ���
  INTCON=0;               //�жϽ�ֹ
  CMCON=0x07;             //��ֹ�Ƚ���


    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = 1; //�˶˿�����״̬Ϊ�ͣ����Ϊ�������
    #endif

  mInitializeUSBDriver();   //��ʼ��USB�豸
  UserInit();               //�û���ʼ�� 
}//end InitializeSystem()
/***************************
* �û���ʼ��
****************************/
void UserInit()
{             
 // TRISB=0;                    //����Ϊ�������,ָʾ״̬�˿�
 // PORTB=0;
  
  TRISD=0; //gcat
  PORTD=0x0; //gcat

  //TRISB=0; //gcat
  //PORTB=0x0; //gcat

  // LATD &= 0xF0; TRISD &= 0xF0;  //LEDָʾ��

  TRISCbits.TRISC1=0;     //����Ϊ���
  PORTCbits.RC1=0;
} //end UserInit()
/***************************
* USB �豸��ʼ������
****************************/
void mInitializeUSBDriver()
{
  UCFG = UCFG_VAL;                    //USB���üĴ���ֵ��=0x14ʹ��Ƭ��������Ƭ���շ���ȫ�٣���ֹƹ�һ���
  usb_device_state = DETACHED_STATE;  //USB���ڷ���״̬
  usb_stat._byte = 0x00;              //USB״̬��0
  usb_active_cfg = 0x00;              //USB����������0  
}//end mInitializeUSBDriver()
/****************************
* USB������
*****************************/
void USBTasks()
{
  USBCheckBusStatus();   //USB�������״̬����
  if(UCFGbits.UTEYE!=1)  //�����ֹ��ͼ����
   {
     USBDriverService(); //USB�豸�������
   }
}//end USBTasks()

/*****************************
* �û�����
*****************************/
void ProcessIO()
{
// ���USBû�н�������״̬����USB���ڹ���״̬����
 if((usb_device_state < CONFIGURED_STATE)||(UCONbits.SUSPND==1)) return;
  ReportLoopback();    //��ѯ����
}//end ProcessIO()
//----------------------------------------------------------END �������Ӻ�����

//-----------------------------------------------------------2��main����������
/*************************
* USB�������״̬����
**************************/
void USBCheckBusStatus()
{
  if(USB_sense==USB_BUS_ATTACHED) //���USB�豸�ϵ�
   {
     if(UCONbits.USBEN == 0)      //���USB���ڴ��ڽ�ֹ״̬��ʹ��
      {  
          UCON = 0;
          UIE = 0;                            // �������USB�ж�
          UCONbits.USBEN = 1;                 // ʹ��USBģ��
          usb_device_state = ATTACHED_STATE;  // USB�����ϵ�״̬
      }//end if
   }
  else
   {
      if(UCONbits.USBEN == 1)  // ���USB����ʹ��״̬����ֹ
       {
          UCON = 0;                           // ��ֹUSBģ��
          UIE = 0;                            // ������USB�ж�
          usb_device_state = DETACHED_STATE;  // USB�豸���ڶϿ�״̬
       }//end if

   }//end if(USB_sense==USB_BUS_ATTACHED)

  if(usb_device_state == ATTACHED_STATE)  //���USB�����ϵ�״̬
   {
      if(!UCONbits.SE0) //δ��⵽����0
       { 
          UIR = 0;                        // �������USB�жϱ�־
          UIE = 0;                        // ��λ����USB�ж�
          UIEbits.URSTIE = 1;             // ����λ�ж� 
          UIEbits.IDLEIE = 1;             // ������м���ж�   
          usb_device_state = POWERED_STATE;//POWERED_STATE=2��USB������Ч״̬  
       }//end if  

   }//end if(USB_sense == USB_BUS_ATTACHED)

}//end USBCheckBusStatus()
/*************************
* USB�豸������
*************************/
void USBDriverService()
{
  if(usb_device_state == DETACHED_STATE) return;  //���״̬Ϊ0����,��ʱUSB��û���ϵ�

  //����A
  //���߻����ж���������ж����ڹ����ſ���
  if(UIRbits.ACTVIF && UIEbits.ACTVIE) //USB�ӹ����л���,���߻�ж�
   {
      UCONbits.SUSPND = 0;             //USBģ���˳�����״̬��������������״̬
      UIEbits.ACTVIE = 0;              //���־����ֹ���߻�ж�
      UIRbits.ACTVIF = 0;
   }//end if

  if(UCONbits.SUSPND==1) return;       //������ߴ��ڹ���״̬�򷵻أ������д���

  //����B��
  //USB���߸�λ�ж�����
  if(UIRbits.URSTIF && UIEbits.URSTIE) //ȷ�ϸ�λ�ж�����
   {
     UEIR = 0;          // ������еĴ����жϱ�־
     UIR = 0;           // ���USB�жϱ�־
     UEIE = 0b10011111; // ��������USB�����ж�
     UIE = 0b01111011;  // �����������ж����������USB�ж�
    
     UADDR = 0x00;      // ��λ��Ĭ�ϵ�ַ
     mDisableEP1to15(); // ��λ����EP0�������˵�
     UEP0 = EP_CTRL|HSHK_EN; //�˵�0ʹ��Ϊ���ƶ˿�

     while(UIRbits.TRNIF == 1) // ���һ��δ������
      {
        UIRbits.TRNIF = 0;
      }
     UCONbits.PKTDIS = 0;  // ȷ��SIE���ƺͰ�����ʹ��
     USBPrepareForNextSetupTrf();//USBΪ��һ�ο��ƴ����SETUP����׼��

     usb_stat.RemoteWakeup = 0;      // ���Ĭ��״̬��־ 
     usb_active_cfg = 0;             // Clear active configuration
     usb_device_state = DEFAULT_STATE; //=3 ����Ĭ��״̬
   }//end if


   //����C �������USB�жϵĴ���
   if(UIRbits.IDLEIF && UIEbits.IDLEIE)  // ����������߿����жϣ���USB��Ҫ����
    {
        UIEbits.ACTVIE = 1; // ʹ�����߻�ж�
        UIRbits.IDLEIF = 0; // ������߿����жϱ�־
        UCONbits.SUSPND = 1;// ����USB
        //����Ҳ���Լ��ж�Դ�������ߵ�Ƭ�����˴�û����ӣ����Ѻ��ͨ��Զ�̻��ѽ���ͨ��
        /*
         // ��ΪӦ�ó���û���ܹ����ѵģ�������ʱ���ӣ�����Զ�̻���û��
        */ 
    }//end if

   if(UIRbits.SOFIF && UIEbits.SOFIE)    //������������ж�
    {
       UIRbits.SOFIF = 0;   //���־λ
    }//end if
   
   if(UIRbits.STALLIF && UIEbits.STALLIE) //������������ж�
    {
       if(UEP0bits.EPSTALL == 1)// ����˵�ֹͣ����Ϊһ����������жϺ󣬱�־����δ���Ľ���
        {                       // �˵���Զ���ֹ
          USBPrepareForNextSetupTrf();  // ׼����һ��SETUP����
          UEP0bits.EPSTALL = 0;         // ʹ�ܶ˵�0
        }//end if
       UIRbits.STALLIF = 0;             // �����ж��ź���0
    }//end if

   if(UIRbits.UERRIF && UIEbits.UERRIE) // �������USB�����ж�
    {
       UIRbits.UERRIF=0; 
    }//end if
 
   if(usb_device_state < DEFAULT_STATE)  return;   // ���û�н���Ĭ��״̬�˳�
                                                   // �����ڲ���һ�θ�λ����ܽ���Ĭ��̬

   if(UIRbits.TRNIF && UIEbits.TRNIE)              // ������������ж�
    {
       USBCtrlEPService();  // USB���ƶ˵��������Ĭ����EP0��
       UIRbits.TRNIF = 0;   // ��־��0
    }//end if
   
}//end USBDriverService()

/**********************
* ��λ�˵�
***********************/
void mDisableEP1to15()
{
  ClearArray((byte*)&UEP1,15);
}//end mDisableEP1to15()

/***********************
* ��ַ������0 ����
************************/
void ClearArray(byte* startAdr,byte count)
{
    *startAdr;
    while(count)
    {
        _asm
        clrf POSTINC0,0   //��18��ִ࣬����󣬵�ַ�Զ���1����UEP��ַ��������
        _endasm
        count--;
    }//end while
}//end ClearArray
//------------------------------------------------------end 2��main �Ӻ�������


//-------------------------------------------------------���ƴ��亯����

/**************************************
* USB Ϊ��һ�εĿ��ƴ����SETUP����׼��
***************************************/
void USBPrepareForNextSetupTrf()
{
    ctrl_trf_state = WAIT_SETUP;            // See usbctrltrf.h
    ep0Bo.Cnt = EP0_BUFF_SIZE;              //��������ȥ��С,8���ֽ�
    ep0Bo.ADR = (byte*)&SetupPkt;           //ȡ�������׵�ַ
    ep0Bo.Stat._byte = _USIE|_DAT0|_DTSEN;  // EP0 OUT ����������״̬����
    ep0Bi.Stat._byte = _UCPU;               // EP0 IN  ��������ʼ������������CPU��
                                            // �����SIE���������ݷ���
}//end USBPrepareForNextSetupTrf()

/****************************
* USB ���ƶ˵�������
****************************/
void USBCtrlEPService()
{
  if(USTAT == EP00_OUT)  //�����һ����OUT��SETUP���ƣ������Ƕ˵�0
   {
     if(ep0Bo.Stat.PID == SETUP_TOKEN) // EP0 SETUP,�������������
        USBCtrlTrfSetupHandler();       // USB���ƴ������ô�����
     else
        USBCtrlTrfOutHandler();         // OUT���񣬴�������������
   }
  else if(USTAT == EP00_IN)               // EP0 IN����������
       USBCtrlTrfInHandler();          // IN���������ݵ�����
  
}//end USBCtrlEPService()

/*******************************
* USB���ƴ������ô�����
********************************/
void USBCtrlTrfSetupHandler()
{
   ctrl_trf_state = WAIT_SETUP;  // USB���ƴ���״̬Ϊ�ȴ�
   ctrl_trf_session_owner = MUID_NULL; //��ʱUSB EP0�˵����ݲ����κ�������
   wCount._word = 0;  //�ֽڼ�������0�����ڼ�¼�������ݵĴ�С
 
   USBCheckStdRequest();  //���USB�ı�׼����

   if(ctrl_trf_session_owner == MUID_NULL)  // ����ϱ�û�����󣬻����󲻴��ڣ����HID����
   {
     USBCheckHIDRequest();  // USB HID������ 
   }//end if
   
   USBCtrlEPServiceComplete();  // ���ƴ������
}//end USBCtrlTrfSetupHandler()

/*******************************
* USB�����������������
* ���ݴ�����-�豸
********************************/
void USBCtrlTrfOutHandler()
{
//˵�����˳����ǿ��ƴ�����������������ݽ׶�

  if(ctrl_trf_state == CTRL_TRF_RX)  // �������״̬�ǣ����ݶ�ȡ�������������豸��
   {
     USBCtrlTrfRxService();          // USB�����������
     if (ctrl_trf_session_owner == MUID_HID)  // �����ʱ�Ĵ�������HID����
      {
        HandleControlOutReport();    // HID�����������������������豸��
      }

    if(ep0Bo.Stat.DTS == 0)          // DATA0��DATA1 ����
      ep0Bo.Stat._byte = _USIE|_DAT1|_DTSEN;
    else
      ep0Bo.Stat._byte = _USIE|_DAT0|_DTSEN;
   }//end if
   else  // �������ǿ��ƶ���״̬�׶�
    USBPrepareForNextSetupTrf();  // ׼����һ�εĿ��ƴ���
}//end USBCtrlTrfOutHandler()

/********************************
* USB�����������봦����
* ���ݴ��豸������
*********************************/
void USBCtrlTrfInHandler()
{

  mUSBCheckAdrPendingState();  // ��Ϊ����ǿ���д���������趨��ַ����������һ������������״̬��������Ҫ����ַ״̬
  if(ctrl_trf_state == CTRL_TRF_TX)   // ���ݷ���
  {
   USBCtrlTrfTxService(); // ���ݼ��ص������ͻ�����
    if(ep0Bi.Stat.DTS == 0)
      ep0Bi.Stat._byte = _USIE|_DAT1|_DTSEN;
     else
      ep0Bi.Stat._byte = _USIE|_DAT0|_DTSEN;
   }
    else // �������ǿ���д��״̬�׶� 	
      USBPrepareForNextSetupTrf();
}//end USBCtrlTrfInHandler()
/*****************************
* ���ƴ�����ɺ���
******************************/
void USBCtrlEPServiceComplete()
{
 //˵�������ƺͰ�����λ���յ�һ�����ô������λ����ֹ����Ҫ�ڸı�ep0Bi.Stat or ep0Bo.Statǰ��0��ʹ�ܣ�
 //���ƴ����һ�������ý׶� 
   UCONbits.PKTDIS = 0;    //ʹ�����ƺͰ�����
   
   if(ctrl_trf_session_owner == MUID_NULL)  // ���󲻱�֧�֣���ֱ�ӷ����˵�ֹͣ�ź�
    { 
      /*
      ������������󲻱�֧�֣���֪����ô��Ӧ����ֹͣ�˵㷢���źţ�ͬʱ׼����EP0
      ������һ�ε�����
      */
      ep0Bo.Cnt = EP0_BUFF_SIZE;      // ���û�������С����������׼��������һ�ε�����
      ep0Bo.ADR = (byte*)&SetupPkt;   // ���û�������ַ

      ep0Bo.Stat._byte = _USIE|_BSTALL; // �˵��SIE����ֹ�˵㷢����ֹ�˵�����
      ep0Bi.Stat._byte = _USIE|_BSTALL;
    }
   else                                  // ����Ļ��������ܱ�ִ�е�
    {
       if(SetupPkt.DataDir == 1)         // ������������豸������
        {
          if(SetupPkt.wLength < wCount._word)  // wLength���������󳤶�
             wCount._word = SetupPkt.wLength;  // �������������ֽڷ�������
           USBCtrlTrfTxService();              // USB���ƴ������ݴ��豸�������������
           ctrl_trf_state = CTRL_TRF_TX;       // ���ڴ���CTRL_TRF_TX�豸��������״̬

           ep0Bo.Cnt = EP0_BUFF_SIZE;
           ep0Bo.ADR = (byte*)&SetupPkt;   
           ep0Bo.Stat._byte = _USIE;               // DTSEN = 0������ʹ��ͬ��,׼�������������������ź�

           ep0Bi.ADR = (byte*)&CtrlTrfData;
           ep0Bi.Stat._byte = _USIE|_DAT1|_DTSEN;  // �������ͣ��������ݷ���
        }
       else                                        // �������ݴ��������豸
        {
           ctrl_trf_state = CTRL_TRF_RX;           // ���ƴ������״̬
          /*
          1.IN �˵�׼����Ӧ��һ�ε���ֹ
          */
            ep0Bi.Cnt = 0;  // ׼��״̬�׶ε��豸��Ӧ���豸��״̬�׶λ�Ӧ0���ȵ����ݰ�
            ep0Bi.Stat._byte = _USIE|_DAT1|_DTSEN; // ����DATA1���ݰ� 

          /*
          2.׼��OUT�˵�������ݣ������Ѿ����������ݴ�С�Լ�������
          */
          ep0Bo.Cnt = EP0_BUFF_SIZE;
          ep0Bo.ADR = (byte*)&CtrlTrfData;
          ep0Bo.Stat._byte = _USIE|_DAT1|_DTSEN;
        }
    }//end if
}//end USBCtrlEPServiceComplete()

/******************************
* USB���ƴ��䣨�����豸��������
******************************/
//�˳���ֻ������ݵ�׼������
void USBCtrlTrfTxService()
{
  WORD byte_to_send;                    // ��������ֽ�
  if(wCount._word < EP0_BUFF_SIZE)      // �жϷ��͵��ֽ����Ƿ�С��EP0_BUFF_SIZE
    byte_to_send._word = wCount._word;  // ���С������ʵ��ֵ
  else
    byte_to_send._word = EP0_BUFF_SIZE; // ������EP0_BUFF_SIZE��С���ֽ�
    
     /*
      ������,���ط����ֽ���Ŀ,���8���ֽ�,��������8���ֽ�
     */
    ep0Bi.Stat.BC9 = 0;    // �����ֽڸ���λ��0
    ep0Bi.Stat.BC8 = 0;
    ep0Bi.Stat._byte |= MSB(byte_to_send);  // ���ô����ֽ���Ŀ
    ep0Bi.Cnt = LSB(byte_to_send);
// ���ܷ����ֽ��м������η��͵��ֽ���
    wCount._word = wCount._word - byte_to_send._word;

    pDst.bRam = (byte*)&CtrlTrfData;   // �趨������ָ�룬��������������400���ڣ�
    
    if(usb_stat.ctrl_trf_mem == _ROM)  // ������ݴ����ROM��,ȡ������
     {
       while(byte_to_send._word)
        {
          *pDst.bRam = *pSrc.bRom;
           pDst.bRam++;
           pSrc.bRom++;
           byte_to_send._word--;
        }
     }  
    else
     {
       while(byte_to_send._word)
        {
          *pDst.bRam = *pSrc.bRam;
           pDst.bRam++;
           pSrc.bRam++;
           byte_to_send._word--;
        }//end while(byte_to_send._word)
     }//end if
}//end  USBCtrlTrfTxService() 

/********************************
* USB����������������������豸
********************************/
void USBCtrlTrfRxService()
{
   WORD byte_to_read;    // ���������ֽ�������

   MSB(byte_to_read) = 0x03 & ep0Bo.Stat._byte;  // ��������λ��ȡ�������ֽ�����������������
   LSB(byte_to_read) = ep0Bo.Cnt;


   wCount._word = wCount._word + byte_to_read._word; // �����ֽ����Ӳ���
   pSrc.bRam = (byte*)&CtrlTrfData;  
   
    while(byte_to_read._word)  // ����ȡ��
    {
        *pDst.bRam = *pSrc.bRam;
        pDst.bRam++;
        pSrc.bRam++;
        byte_to_read._word--;
    }//end while(byte_to_read._word) 
   
}//end USBCtrlTrfRxService()

//---------------------------------------------------------end ���ƴ��亯����


//---------------------------------------------------------USB ����Э�飨����)��������

/*******************************
* ���USB�ı�׼����
*********************************/
void USBCheckStdRequest()
{
  if(SetupPkt.RequestType != STANDARD) return;   // �������USB��׼���󷵻�
   
  switch(SetupPkt.bRequest)  //ѡ����������
   {
     case SET_ADR:   //=5  ���������ַ����
      //˵��: �豸����ִ�д�����,�������Ѿ�����һ��0����������Ϣ��,����������״̬�׶�.
      //      ��������״̬�׶ε�������Ϣ����Ĭ�ϵ�ַ0,�����豸�����ڸı����ĵ�ַǰ��Ⲣ��Ӧ����Ϣ��
      //  wVAlue�ֶ����ݣ����豸��ַ
      //  wIndex�ֶ�����: 0    
       ctrl_trf_session_owner = MUID_USB9;     // MUID_USB9=1 USB9��USB��������
       usb_device_state = ADR_PENDING_STATE;   // ֻ����״̬�����뵽��ַδ��״̬
     break;

     case  GET_DSC:  //6   �����������
      //˵������������7�����ͣ�ÿһ�豸������һ���豸��������
      //      �Լ�����һ��������������һ���ӿ�������
      // ������Դ���豸
      // ���ݳ��ȣ������ֽڵ���Ŀ��������
      // wValue:   ���ֽ������������ͣ����ֽ�����������ֵ
      // wIndex:   ������ַ������������ֶ�������ID��������0
       USBStdGetDscHandler();                  //USB��׼�������������
     break;  

     case  SET_CFG:   //9   ָʾ�豸ʹ����ѡ�������
      //˵������������������ָ��һ��֧�ֵ��������ú��豸���������״̬��
      //      ����׼������Ҫ�豸������״̬
      //���ݳ��ȣ�0
      //wValue�ֶΣ����ֽ�ָʾһ���������á�������ֶε���ֵ�����豸��֧�ֵ�һ�����ã��豸��ѡ��
      //            ����������á������0��ʾ��δ�������ã������ַ״̬����Ҫ�µ���������������
      //wIndex�ֶΣ�0
      //֧��״̬��  ��ַ������
       USBStdSetCfgHandler();   //USB��׼�����豸���ô���
     break;  

     case  GET_CFG:    //8   ��������Ŀǰ�豸���õ���ֵ
      //˵��������豸û���������ã��򴫻�0
      //���ݳ��ȣ�1
      //wValue�ֶΣ�0
      //wIndex�ֶΣ�0   
      ctrl_trf_session_owner = MUID_USB9;  // USB9��USB��������
      pSrc.bRam = (byte*)&usb_active_cfg;  // �����׵�ַ
      usb_stat.ctrl_trf_mem = _RAM;        // ���ݶ�����RAM��
      wCount._word = 1;                    // �����ֽڼ���=1
     break;   

     case   GET_STATUS: // 0  ��������һ���豸���ӿڻ�˵������״̬
      //˵����������豸����ֻ������λ��λ0�������Դ�ֶΡ��������ܸı����ֵ��
      //      λ1��Զ�̻����ֶΣ���������0������ǽӿڵ���������λ�����������
      //      �˵�����ֻ��λ0�����壬λ0=1��һ����̬ͣ(Halt).
      //�����ֽڣ�2
      //wValue�ֶΣ�0
      //wIndex�ֶΣ��豸=0������ǽӿ�Ϊ�ӿںţ�����Ƕ˵�=�˵��
      USBStdGetStatusHandler();  // �������״̬������
     break;      

     case  CLR_FEATURE:  //1 �����������һ�����豸���ӿڻ�˵��ϵ�����
                         //  ����������ִ��
     case SET_FEATURE:   //3 ������������һ�����豸���ӿڻ�˵��ϵ�����
      //˵����USB�淶�����������ԣ���DEVICE_REMOTE_WAKEUP����ֵΪ1��Ӧ�����豸��
      //      �Լ�ENDPOINT_HALT(��ֵΪ0��Ӧ���ڶ˵㣩
      //���ݣ���
      //wValue�ֶΣ�Ҫ���õ����� DEVICE_REMOTE_WAKEUP  1 �豸
      //                         ENDPOINT_HALT         0 �˵�
      //                         TEST_MODE             2 �豸
      //wIndex�ֶΣ�������豸�����Դ��ֶ�Ϊ0������ǽӿڴ��ֶ�Ϊ�ӿں���
      //            ����Ƕ˵����ԣ����ֶ��Ƕ˵����
      USBStdFeatureReqHandler();  //����������������ֵ������
     break;
  
     case GET_INTF:     //10  ����豸������֧�ֶ������������õĽӿڣ���������Ŀǰ��״̬        
      //˵����wIndex�ֶεļ��ں��룬��ָ�ӿ��������е�bInterface�ֶΡ�ÿһ���ӿڵĺ��벻ͬ
      //      �����ֶε����ݣ���ָ�ӿ��������е�bAlternateInterface�ֶ����ݡ������ֶα�ʾĿǰ
      //      ʹ�õĽӿ�����������������ص����á�
      //wValue�ֶΣ�0
      //wIndex�ֶΣ��ӿں���  
      ctrl_trf_session_owner = MUID_USB9;
      pSrc.bRam = (byte*)&usb_alt_intf+SetupPkt.bIntfID;  // ȡ������,Ҳ���ǽӿں���,һ���һ���ӿ�0
      usb_stat.ctrl_trf_mem = _RAM;
      wCount._word = 1;   // �����ֽ�     
     break;

     case  SET_INTF:   //11  ����豸������֧�ֶ������������õĽӿڣ����������豸ʹ��һ��ָ��������
      ctrl_trf_session_owner = MUID_USB9;
      usb_alt_intf[SetupPkt.bIntfID] = SetupPkt.bAltID;  // ѡ������Ľӿں������ǰ��
     break;
 
     case SET_DSC:     //7   ��������һ�������������Ǹ���һ�����ڵ�������
     case SYNCH_FRAME: //12  �豸�����뱨��һ���˵��ͬ��֡
     default:
     break;
   }// end switch
}//end USBCheckStdRequest()

/********************************
*  USB��׼��������ô�����
*********************************/
void USBStdGetDscHandler()
{
  if(SetupPkt.bmRequestType == 0x80) // ����Ƿ���GET_DSC
   {
     switch(SetupPkt.bDscType)       // ѡ������������
      {
        case DSC_DEV:                // �豸������
          ctrl_trf_session_owner = MUID_USB9;  // ���ƴ����SETUP����
          pSrc.bRom = (rom byte*)&device_dsc;  // ȡ�豸�����������׵�ַ,bRom���ֽ�ָ��1�ֽ�����
          wCount._word = sizeof(device_dsc);   // ȡ���豸��������С
        break;
 
        case DSC_CFG:                   // ����������
          ctrl_trf_session_owner = MUID_USB9;  // ���ƴ����SETUP����
          pSrc.bRom = *(USB_CD_Ptr+SetupPkt.bDscIndex); //USB_CD_Ptr�ǵ�ַָ������,���CFG0��ַ,����ƫ�ƺ�ȡ*�ѵ�ַ����Ҳ���ǰ�CFG0��ַ����ָ��brom
          wCount._word = *(pSrc.wRom+1); //pSrc.wRom��ָ�룬���ĵ�ַ��pSrc.bRom��ͬ����ָ��2���ֽ�����
                                         //��1��Խ�����ֽ����ݵ�CFG01��sizeof(cfg01),�ֽ����������õ����ֽ�������
        break;

        case DSC_STR:                       // �ַ���������   
          ctrl_trf_session_owner = MUID_USB9;
          pSrc.bRom = *(USB_SD_Ptr+SetupPkt.bDscIndex);
          wCount._word = *pSrc.bRom;         // Set data count
        break;
      } //end switch
   }//end if
  usb_stat.ctrl_trf_mem = _ROM;
}//end USBStdGetDscHandler()  

/********************************
*USBָʾ�豸ʹ����ѡ������ô���
*********************************/
void USBStdSetCfgHandler()
{
   ctrl_trf_session_owner = MUID_USB9;           // ���ƴ����USB9����������Ժ���Է���״̬
   mDisableEP1to15();                            // ��λ�˵�1����15
   ClearArray((byte*)&usb_alt_intf,MAX_NUM_INT); // �彻�����飨�ж϶˵㣩����ʹ��һ����ʵ���Բ���
   usb_active_cfg = SetupPkt.bCfgValue;          // ȡ��������������������
   if(SetupPkt.bCfgValue == 0)                   // ��ʾ��δ�������ã������ַ״̬
    {
      usb_device_state = ADDRESS_STATE;          // �豸�����ַ״̬
    }//end if
   else
    {
      usb_device_state = CONFIGURED_STATE;       // �����������״̬
      HIDInitEP();                               // HID�˵�����
                                                 // һ����������״̬��������ȷ���豸���������ݴ���׶�
    } //enf if
}// end USBStdSetCfgHandler()

/******************************
* USB���״̬������
******************************/
void USBStdGetStatusHandler()
{
  CtrlTrfData._byte0 = 0;      // ׼�������ֽڵ�������
  CtrlTrfData._byte1 = 0;

  switch(SetupPkt.Recipient)   // ѡ�����豸�ӿڻ��Ƕ˵�
   {
     case  RCPT_DEV:           // =0 ��ȡ�豸״̬
      ctrl_trf_session_owner = MUID_USB9;   //�鴫��Э�飨��������
      CtrlTrfData._byte0|=0b000000001;      // SetB byte0.0,��ʾʹ�������Դ
      if(usb_stat.RemoteWakeup == 1)        // usb_stat defined in usbmmap.c
        CtrlTrfData._byte0|=0b00000010;     // Set bit1���Ƿ�֧�֣�Զ�̻���
     break;
   
     case  RCPT_INTF:          // =1 ��ȡ�ӿ�״̬
      ctrl_trf_session_owner = MUID_USB9; // ���������ֶ�=0��
     break;

     case  RCPT_EP:            // =2 ����ǻ�ȡ�˵�״̬
      ctrl_trf_session_owner = MUID_USB9;
      pDst.bRam = (byte*)&ep0Bo+(SetupPkt.EPNum*8)+(SetupPkt.EPDir*4); // ȡ��״̬�Ĵ�����ַ
      if(*pDst.bRam & _BSTALL) // �ж϶˵��Ƿ�ֹͣ
       CtrlTrfData._byte0=0x01;// ���ֹͣ��ʾ�˵����һ����̬ͣ  
     break;
   }// end switch
} // end USBStdGetStatusHandler()

/***********************************
*����������������ֵ������
************************************/
void USBStdFeatureReqHandler()
{
  if((SetupPkt.bFeature == DEVICE_REMOTE_WAKEUP)&&(SetupPkt.Recipient == RCPT_DEV))  //RCPT_DEV=0,�����豸��Զ�̻���״̬״̬ 
  {
     ctrl_trf_session_owner = MUID_USB9;
     if(SetupPkt.bRequest == SET_FEATURE)  // �ж��Ƿ����������
       usb_stat.RemoteWakeup = 1;          // �������Զ�̻��ѣ�״̬λ��1
     else  //������CLEAR_FEATURE
       usb_stat.RemoteWakeup = 0;   
  }//end if 
  
  if((SetupPkt.bFeature == ENDPOINT_HALT)&&(SetupPkt.Recipient == RCPT_EP)&&(SetupPkt.EPNum != 0))// ��������Ķ˵㲻�Ƕ˵�0����
  {
    ctrl_trf_session_owner = MUID_USB9;
    pDst.bRam = (byte*)&ep0Bo+(SetupPkt.EPNum*8)+(SetupPkt.EPDir*4); //ȷ���˵�״̬��ַ
    if(SetupPkt.bRequest == SET_FEATURE)//����Ƕ˵���������
      *pDst.bRam = _USIE|_BSTALL;   //�Ѷ˵�״̬�Ĵ�������ΪSIE���в�ʹ�ܻ�����ֹͣ
    else   //������CLEAR_FEATURE
     {
       if(SetupPkt.EPDir == 1)    // �����IN
        *pDst.bRam = _UCPU;       // �˵��CPU
       else
        *pDst.bRam = _USIE|_DAT0|_DTSEN;  //���������Ļ����˵��SIE��DATA0ʹ������ͬ��λ
     }// end if 
  }//end if            
}//end USBStdFeatureReqHandler()

/********************************
* USB����ַ״̬
********************************/
void mUSBCheckAdrPendingState()  
{
 if(usb_device_state==ADR_PENDING_STATE) 
  {                                       
     UADDR = SetupPkt.bDevADR._byte;     
     if(UADDR > 0)                       
     usb_device_state=ADDRESS_STATE; 
     else                                
     usb_device_state=DEFAULT_STATE; 
  }//end if
}//end mUSBCheckAdrPendingState()

//--------------------------------------------------------- end USB ����Э�飨����)��������


//--------------------------------------------------------- HID ������
/*******************************
* HID �˵����ú���
*******************************/
void HIDInitEP()
{
   hid_rpt_rx_len=0;                            // HID��������ֽ�����0
  
   HID_UEP = EP_OUT_IN|HSHK_EN; 
   ClearArray((byte*)&usb_alt_intf,MAX_NUM_INT);// ��һ�����飬Ϊ��������MAX_NUM_INT=1,�ж϶˵����=1
   
   HID_UEP = EP_OUT_IN|HSHK_EN;                 // ��ʼ���˵�1
   HID_BD_OUT.Cnt = sizeof(hid_report_out);     // ���û�������С
   HID_BD_OUT.ADR = (byte*)&hid_report_out;     // ���û������׵�ַ
   HID_BD_OUT.Stat._byte = _USIE|_DAT0|_DTSEN;  // �˵�1��OUT����
                                                
   HID_BD_IN.ADR = (byte*)&hid_report_in;       // �˵�1��IN�˵��ַ����
   HID_BD_IN.Stat._byte = _UCPU|_DAT1;          // �˵�1��IN״̬����
}// end HIDInitEP()


/***********************************
* HID�������
***********************************/
void USBCheckHIDRequest()   //HID����������USB״̬���ǿգ������⣬��Ϊ����USB�ı�׼����ʱ��
{
  if(SetupPkt.Recipient != RCPT_INTF) return;  // ��������ǶԽӿ����˳�����HID ���ǶԶ˿�˵��
  if(SetupPkt.bIntfID != HID_INTF_ID) return;  // ���HID�ӿڵĺŲ�������Ľӿ�ID��ͬ�˳�
   
  if(SetupPkt.bRequest == GET_DSC)  // ������������������
   {
/*
#define DSC_HID         0x21   // HID
#define DSC_RPT         0x22   // ����
#define DSC_PHY         0x23   // ʵ��
*/
     switch(SetupPkt.bDscType)
      {
        case 0x21:   // ���HID������
         ctrl_trf_session_owner = MUID_HID;  // �����HID����
         if(usb_active_cfg == 1)             // ����������״̬�Ժ����������
          {
           pSrc.bRom = (rom byte*)&cfg01.hid_i00a00;  // ȡ��HID���������׵�ַ
          }
         wCount._word = sizeof(USB_HID_DSC); //ȡ����������С
        break;

        case 0x22:       //����������
         ctrl_trf_session_owner = MUID_HID;  // �����HID����
         if(usb_active_cfg == 1)   
         {              
          pSrc.bRom = (rom byte*)&hid_rpt01; // ������������ROM�ڵ��׵�ַ
          wCount._word = sizeof(hid_rpt01);  // ������������С
         }                
        break;

        case 0x23:
        default:
        break;
      }//end switch
    usb_stat.ctrl_trf_mem = _ROM;   // ���ݴ洢��ROM��
   }//end if

  if(SetupPkt.RequestType != CLASS) return; // ��������ض�USB�������󷵻�
  switch(SetupPkt.bRequest)    // ѡ��HID�ض�����
   {
     case  GET_REPORT:  // 01 ��������ʹ�ÿ��ƴ��䣬�����豸��ȡ����
      HIDGetReportHandler(); // ����������ݴ�����
     break;
    
     case SET_REPORT:   // 09 �����豸���ƴ��䣬�豸�������������
      HIDSetReportHandler(); // �豸�������ݴ�����
     break; 

     case GET_IDLE:     // 02 �����������豸��ȡĿǰ��������
      ctrl_trf_session_owner = MUID_HID; // ���ƴ����ϵͳ��HID����
      pSrc.bRam = (byte*)&idle_rate;     // ������4MSΪ��λ����һ��������4MS
      usb_stat.ctrl_trf_mem = _RAM;      // Set memory type ���ݴ洢��RAM��
      wCount._word = 1;
     break;   
  
     case SET_IDLE:
      //˵����0x0a �����ݴ���һ���������û�иı�ʱ�������ж�����˵�ı���Ƶ������ʡ����
      //      ���HID֧�ִ�������ֻ�������ݸı�ʱ�Żᷢ�ͱ���  
       ctrl_trf_session_owner = MUID_HID;
       idle_rate = MSB(SetupPkt.W_Value);
     break;  

     case GET_PROTOCOL:                    // 0x03 ���������˽��豸Ŀǰ����������Э����Ǳ���Э��
       ctrl_trf_session_owner = MUID_HID;  // ���ƴ����ϵͳ��HID����   
       pSrc.bRam = (byte*)&active_protocol;// Set source
       usb_stat.ctrl_trf_mem = _RAM;
       wCount._word = 1;  // ���ݳ���
     break;

     case SET_PROTOCOL:   // 0x0b ����ָ��ʹ������Э����Ǳ���Э��
       ctrl_trf_session_owner = MUID_HID;
       active_protocol = LSB(SetupPkt.W_Value); //ȡ��Э������
     break;  
   }//end switch
}//end USBCheckHIDRequest()


/**********************************
* ����������ݴ�����
**********************************/
void HIDGetReportHandler()
{
  switch(MSB(SetupPkt.W_Value))
   {
     case 1:      // ���뱨��
     switch(LSB(SetupPkt.W_Value))
      {
        case 0:                   // ѡ�񱨱�ID
          ctrl_trf_session_owner = MUID_HID;   // ���ƴ����ϵͳ��HID����
          GetInputReport0();                   // ׼�����ݷ��͵�����
        break;
        case 1:                   // ����ֻ��һ������
        break;  
      }
     break;

     case 3:    // �����������
      switch(LSB(SetupPkt.W_Value))
       {
         case 0:
          ctrl_trf_session_owner = MUID_HID;
          GetFeatureReport0();
         break;
         case 1:
         break;
       }   
     break;
   }//end switch 
  usb_stat.ctrl_trf_mem = _RAM;               // Set memory type�����ݶ�����RAM��
}//end HIDGetReportHandler()

/***********************************
* �豸������ݴ�����
***********************************/
void HIDSetReportHandler()
{
   switch(MSB(SetupPkt.W_Value))
    {
      case 2:      // �豸�������
       switch(LSB(SetupPkt.W_Value))  //ѡ��ID��
        {
          case 0:  // �����0�ű���
           ctrl_trf_session_owner = MUID_HID;
           pDst.bRam = (byte*)&hid_report_out; //�������ݴ洢�������������ݷ��͵��Ժ��洢��������
          break;
        }
      break;
  
      case 3:      // ������������ֵ
       switch(LSB(SetupPkt.W_Value))
        {
          case 0:
           ctrl_trf_session_owner = MUID_HID;
            pDst.bRam = (byte*)&hid_report_feature;  //���ô洢������ 
          break;
        } 
      break;  
    }//end switch
}//end HIDSetReportHandler()

/*******************************
* HIDȡ�����ֽڳ���
*******************************/
byte HIDRxReport(char *buffer, byte len) 
{
  hid_rpt_rx_len = 0;
  if(!HID_BD_OUT.Stat.UOWN)  // ���������������������˵���CPU����
  {
    if(len > HID_BD_OUT.Cnt) // �����ڴ����������ֽ�������ʵ�ʽ��յ��ֽ���
     len = HID_BD_OUT.Cnt;   // 
    for(hid_rpt_rx_len = 0; hid_rpt_rx_len < len; hid_rpt_rx_len++)  // ���ݶ�ȡ
    {
      buffer[hid_rpt_rx_len] = hid_report_out[hid_rpt_rx_len]; // �����ݶ���������   
    }  
    HID_BD_OUT.Cnt = sizeof(hid_report_out);  // ׼����һ�ε����ݽ��գ���ʼ��
    HID_BD_OUT.ADR = (byte*)&hid_report_out; 
    mUSBBufferReady(HID_BD_OUT);   // ׼������һ�εĽ��գ�����DATA�˵��SIE����
  }//end if
  return hid_rpt_rx_len;           // ���ؽ����ֽڵĳ��ȣ��������û���
}//end

/********************************
* HID �������ݵ�����
********************************/
void HIDTxReport(char *buffer, byte len)
{
  byte i;
  if(len > HID_INT_IN_EP_SIZE)    //ȡ�����������ֽ�
    len = HID_INT_IN_EP_SIZE;

  for (i = 0; i < len; i++)
   {
     hid_report_in[i] = buffer[i];  //������д�뷢�ͻ�����
   }  
  HID_BD_IN.Cnt = len;
  mUSBBufferReady(HID_BD_IN);       //���ݿ�ʼ����
}//end HIDTxRepor


//--------------------------------------------------------- end HID������


//--------------------------------------------------------- �û���������
/********************************
* ׼�����ݷ��͵�����
********************************/
void GetInputReport0()
{
  byte count;
  pSrc.bRam = (byte*)&hid_report_in;     // ����ָ��ָ��IN REPORT������׼��������
  wCount._word = HID_INPUT_REPORT_BYTES; // ���������ֽ�������2���ֽ�
}//end GetInputReport0()

/********************************
*
********************************/
void GetFeatureReport0()
{
  byte count;
  pSrc.bRam = (byte*)&hid_report_feature;       // ׼���������ݵ�ַ��ַ
  wCount._word = HID_FEATURE_REPORT_BYTES;      // ���������ֽ�
}//end GetFeatureReport0()

/********************************
* HID��������������ͣ�
*********************************/
void HandleControlOutReport()
{
	byte count;              //�ֽڼ�����
	switch (MSB(SetupPkt.W_Value))
    {
		case 0x02: // Output report 
    		switch(LSB(SetupPkt.W_Value))
		    {
				case 0: // Report ID 0
					for (count = 1; count <= HID_OUTPUT_REPORT_BYTES; count = count + 1)
					{
						hid_report_in[count-1] = hid_report_out[count-1];
					}				
					break;		
			} // end switch(LSB(SetupPkt.W_Value))

		case 0x03:
    		switch(LSB(SetupPkt.W_Value))
		    {
				case 0: // Report ID 0	
		
				break;
			} // end switch(LSB(SetupPkt.W_Value))		

	} // end switch(MSB(SetupPkt.W_Value))

} // end HandleControlOutReport

/******************************
* �����ѯ
******************************/
//����Ƿ������������ı���
void ReportLoopback()
{
  byte count;   // ��������ֽ�
  //��ȡ���������ֽڵ���Ŀ
  number_of_bytes_read = HIDRxReport(receive_buffer, HID_OUTPUT_REPORT_BYTES); 
  
  if (number_of_bytes_read > 0) // ��������ֽڲ�Ϊ0�������ݴ���
   {
/* gcat
     //ȡ�������������ݵ����ͻ�����
     for(count =0 ; count < HID_OUTPUT_REPORT_BYTES; count++) 
      { 
       transmit_buffer[count] = receive_buffer[count]; //2���ֽڵ����ݵ����ͻ�����
         
      }//end for      
     while(HID_BD_IN.Stat.UOWN)   // ������ͻ�������ʱ���ڷ������ݽ׶εȴ�
      {
       USBDriverService();
      }//end while
      HIDTxReport(transmit_buffer, HID_INPUT_REPORT_BYTES);  // ���ݷ���
*/
//�Զ��庯����ʵ���ض�����
      user();
      
   } //end if
}//end ReportLoopback()

/*******************************
*�û�����
*******************************/
void user()
{
  PORTD=receive_buffer[0];
  //PORTB=receive_buffer[1];
/* gcat
  if((receive_buffer[0]==0x01) &&(receive_buffer[1]==0x02))
   {
     PORTCbits.RC1=1;
   }
  else
   {
     PORTCbits.RC1=0;
   }
*/
}
//--------------------------------------------------------- end�û���������

