/*
基于18F4550的HID程序设计
此程序只完成从主机发送2字节数据，然后设备把数据原封不动返回主机
编译器：MCC18 
版本：MPLAB-C18-v3_02
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Zhongxin An        2007.7.3        HID 
*/
#include <p18cxxx.h>
#include "typedefint.h"
#include "USb data.h"

#include "cfg_bit.h"

#define USB_sense  PORTAbits.RA1  //USB 上电检测端口
#define tris_usb_bus_sense TRISAbits.TRISA1


/**************************
* 函数定义
***************************/
#pragma udata
//---------主函数定义区---------
void InitializeSystem(void);//系统初始化
void USBTasks(void);  //USB任务
void ProcessIO(void); //用户函数 

void mInitializeUSBDriver(void);//初始化USB设备
void UserInit(void);  //用户初始化程序
void ClearArray(byte* startAdr,byte count); //地址清0函数
void mDisableEP1to15(void);  //端点清0函数
//------------------------------

//--------USB控制传输函数区--------
void USBPrepareForNextSetupTrf(void);  // 为下次USB控制传输SETUP做准备
void USBCtrlEPService(void);           // USB 控制端点服务程序，在事务中断中调用  

void USBCtrlTrfSetupHandler(void); // USB控制传输设置处理函数 
void USBCtrlTrfOutHandler(void);   // USB控制输出函数（数据主机-设备）
void USBCtrlTrfInHandler(void);    // USB控制输入数据（数据设备-主机）

void USBCtrlEPServiceComplete(void); // USB控制传输端点服务完成 

void USBCtrlTrfTxService(void);      // USB控制传输数据设备到主机
void USBCtrlTrfRxService(void);      // USB控制数据输出数据主机到设备

//---------------------------------

//--------USB 协议函数区-----------
void USBCheckStdRequest(void);     // 检测USB的标准请求

void USBStdGetDscHandler(void);    // USB获得标准描述符处理函数
void USBStdSetCfgHandler(void);    // USB指示设备使用所选择的配置处理
void USBStdGetStatusHandler(void); // USB主机获得状态处理函数
void USBStdFeatureReqHandler(void);// 主机请求启用特征值处理函数

void mUSBCheckAdrPendingState(void); // USB检测地址状态是否已经配置  
//---------------------------------

//-----------HID 函数区-------------
void HIDInitEP(void);              // HID端点设置函数
void USBCheckHIDRequest(void);     // USB HID请求检测

void HIDGetReportHandler(void);    // 主机获得报表处理函数
void HIDSetReportHandler(void);    // 设备获得报表处理函数

byte HIDRxReport(char *buffer, byte len); // HID取出接收数据长度函数 
void HIDTxReport(char *buffer, byte len); // HID发送到主机数据
//----------------------------------

//-----------USB用户专用区----------
void GetInputReport0(void);        // 主机获得数据
void GetFeatureReport0(void);      // 主机获得特征

void HandleControlOutReport(void); // HID发送报表（主机发送）
void ReportLoopback(void);         // b报表查询函数

void user(void);                   // 用户自定义函数
//---------------------------------- 

//-------USB 总线以及设备服务函数-------
void USBCheckBusStatus(void);      // USB总线检测函数
void USBDriverService(void);       // USB设备服务程序
//--------------------------------------

extern void _startup (void);       // See c018i.c in your C18 compiler dir
#pragma code _RESET_INTERRUPT_VECTOR = 0x000800  //复位
void _reset (void)
{
    _asm goto _startup _endasm
}
#pragma code

#pragma code
/***************************
* 主函数
****************************/
void main()
{
  InitializeSystem();   //系统初始化函数
  while(1)
   {
     USBTasks();        //USB 任务函数
  
     ProcessIO();       // 
   }
}//end main()

//------------------------------------------------------------主函数子函数区
/***************************
* 系统初始化函数
****************************/
void InitializeSystem()
{
  WDTCONbits.SWDTEN=0;    //禁止看门狗
  ADCON1=0x0f;            //把模拟端口全部设置为数字端口
  HLVDCONbits.HLVDEN=0;   //禁止低压检测
  INTCON=0;               //中断禁止
  CMCON=0x07;             //禁止比较器


    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = 1; //此端口正常状态为低，如果为高则接入
    #endif

  mInitializeUSBDriver();   //初始化USB设备
  UserInit();               //用户初始化 
}//end InitializeSystem()
/***************************
* 用户初始化
****************************/
void UserInit()
{             
 // TRISB=0;                    //设置为输出引脚,指示状态端口
 // PORTB=0;
  
  TRISD=0; //gcat
  PORTD=0x0; //gcat

  //TRISB=0; //gcat
  //PORTB=0x0; //gcat

  // LATD &= 0xF0; TRISD &= 0xF0;  //LED指示灯

  TRISCbits.TRISC1=0;     //设置为输出
  PORTCbits.RC1=0;
} //end UserInit()
/***************************
* USB 设备初始化函数
****************************/
void mInitializeUSBDriver()
{
  UCFG = UCFG_VAL;                    //USB配置寄存器值，=0x14使能片上上拉，片上收发，全速，禁止乒乓缓冲
  usb_device_state = DETACHED_STATE;  //USB处于分离状态
  usb_stat._byte = 0x00;              //USB状态清0
  usb_active_cfg = 0x00;              //USB现行配置清0  
}//end mInitializeUSBDriver()
/****************************
* USB任务函数
*****************************/
void USBTasks()
{
  USBCheckBusStatus();   //USB检测总线状态函数
  if(UCFGbits.UTEYE!=1)  //如果禁止眼图测试
   {
     USBDriverService(); //USB设备服务程序
   }
}//end USBTasks()

/*****************************
* 用户函数
*****************************/
void ProcessIO()
{
// 如果USB没有进入配置状态或者USB处于挂起状态返回
 if((usb_device_state < CONFIGURED_STATE)||(UCONbits.SUSPND==1)) return;
  ReportLoopback();    //查询报表
}//end ProcessIO()
//----------------------------------------------------------END 主函数子函数区

//-----------------------------------------------------------2级main函数程序区
/*************************
* USB检测总线状态函数
**************************/
void USBCheckBusStatus()
{
  if(USB_sense==USB_BUS_ATTACHED) //如果USB设备上电
   {
     if(UCONbits.USBEN == 0)      //如果USB现在处于禁止状态，使能
      {  
          UCON = 0;
          UIE = 0;                            // 清除所有USB中断
          UCONbits.USBEN = 1;                 // 使能USB模块
          usb_device_state = ATTACHED_STATE;  // USB进入上电状态
      }//end if
   }
  else
   {
      if(UCONbits.USBEN == 1)  // 如果USB处于使能状态，禁止
       {
          UCON = 0;                           // 禁止USB模块
          UIE = 0;                            // 清所有USB中断
          usb_device_state = DETACHED_STATE;  // USB设备处于断开状态
       }//end if

   }//end if(USB_sense==USB_BUS_ATTACHED)

  if(usb_device_state == ATTACHED_STATE)  //如果USB处于上电状态
   {
      if(!UCONbits.SE0) //未检测到单端0
       { 
          UIR = 0;                        // 清除所有USB中断标志
          UIE = 0;                        // 复位所有USB中断
          UIEbits.URSTIE = 1;             // 允许复位中断 
          UIEbits.IDLEIE = 1;             // 允许空闲检测中断   
          usb_device_state = POWERED_STATE;//POWERED_STATE=2，USB总线有效状态  
       }//end if  

   }//end if(USB_sense == USB_BUS_ATTACHED)

}//end USBCheckBusStatus()
/*************************
* USB设备服务函数
*************************/
void USBDriverService()
{
  if(usb_device_state == DETACHED_STATE) return;  //如果状态为0返回,此时USB并没有上电

  //任务A
  //总线活动检测中断任务，这个中断是在挂起后才开启
  if(UIRbits.ACTVIF && UIEbits.ACTVIE) //USB从挂起中唤醒,总线活动中断
   {
      UCONbits.SUSPND = 0;             //USB模块退出挂起状态，进入正常工作状态
      UIEbits.ACTVIE = 0;              //清标志并禁止总线活动中断
      UIRbits.ACTVIF = 0;
   }//end if

  if(UCONbits.SUSPND==1) return;       //如果总线处于挂起状态则返回，不进行处理

  //任务B：
  //USB总线复位中断输入
  if(UIRbits.URSTIF && UIEbits.URSTIE) //确认复位中断输入
   {
     UEIR = 0;          // 清除所有的错误中断标志
     UIR = 0;           // 清除USB中断标志
     UEIE = 0b10011111; // 启动所有USB错误中断
     UIE = 0b01111011;  // 启动除活动检测中断以外的所有USB中断
    
     UADDR = 0x00;      // 复位到默认地址
     mDisableEP1to15(); // 复位除了EP0的其他端点
     UEP0 = EP_CTRL|HSHK_EN; //端点0使能为控制端口

     while(UIRbits.TRNIF == 1) // 清除一切未决传输
      {
        UIRbits.TRNIF = 0;
      }
     UCONbits.PKTDIS = 0;  // 确保SIE令牌和包处理使能
     USBPrepareForNextSetupTrf();//USB为下一次控制传输的SETUP做好准备

     usb_stat.RemoteWakeup = 0;      // 清除默认状态标志 
     usb_active_cfg = 0;             // Clear active configuration
     usb_device_state = DEFAULT_STATE; //=3 进入默认状态
   }//end if


   //任务C 完成其他USB中断的处理
   if(UIRbits.IDLEIF && UIEbits.IDLEIE)  // 如果产生总线空闲中断，则USB需要挂起
    {
        UIEbits.ACTVIE = 1; // 使能总线活动中断
        UIRbits.IDLEIF = 0; // 清除总线空闲中断标志
        UCONbits.SUSPND = 1;// 挂起USB
        //下面也可以加中断源，后休眠单片机，此处没有添加，唤醒后可通过远程唤醒进行通信
        /*
         // 因为应用程序没有能够唤醒的，所以暂时不加，所以远程唤醒没有
        */ 
    }//end if

   if(UIRbits.SOFIF && UIEbits.SOFIE)    //如果产生令牌中断
    {
       UIRbits.SOFIF = 0;   //清标志位
    }//end if
   
   if(UIRbits.STALLIF && UIEbits.STALLIE) //如果产生握手中断
    {
       if(UEP0bits.EPSTALL == 1)// 如果端点停止，因为一般产生握手中断后，标志着这次传输的结束
        {                       // 端点会自动禁止
          USBPrepareForNextSetupTrf();  // 准备下一次SETUP传输
          UEP0bits.EPSTALL = 0;         // 使能端点0
        }//end if
       UIRbits.STALLIF = 0;             // 握手中断信号清0
    }//end if

   if(UIRbits.UERRIF && UIEbits.UERRIE) // 如果产生USB错误中断
    {
       UIRbits.UERRIF=0; 
    }//end if
 
   if(usb_device_state < DEFAULT_STATE)  return;   // 如果没有进入默认状态退出
                                                   // 这是在产生一次复位后才能进入默认态

   if(UIRbits.TRNIF && UIEbits.TRNIE)              // 如果产生事务中断
    {
       USBCtrlEPService();  // USB控制端点服务函数（默认是EP0）
       UIRbits.TRNIF = 0;   // 标志清0
    }//end if
   
}//end USBDriverService()

/**********************
* 复位端点
***********************/
void mDisableEP1to15()
{
  ClearArray((byte*)&UEP1,15);
}//end mDisableEP1to15()

/***********************
* 地址数据清0 函数
************************/
void ClearArray(byte* startAdr,byte count)
{
    *startAdr;
    while(count)
    {
        _asm
        clrf POSTINC0,0   //见18汇编，执行完后，地址自动加1，而UEP地址是连续的
        _endasm
        count--;
    }//end while
}//end ClearArray
//------------------------------------------------------end 2级main 子函数定义


//-------------------------------------------------------控制传输函数区

/**************************************
* USB 为下一次的控制传输的SETUP做好准备
***************************************/
void USBPrepareForNextSetupTrf()
{
    ctrl_trf_state = WAIT_SETUP;            // See usbctrltrf.h
    ep0Bo.Cnt = EP0_BUFF_SIZE;              //定义数据去大小,8个字节
    ep0Bo.ADR = (byte*)&SetupPkt;           //取数据区首地址
    ep0Bo.Stat._byte = _USIE|_DAT0|_DTSEN;  // EP0 OUT 缓冲描述符状态设置
    ep0Bi.Stat._byte = _UCPU;               // EP0 IN  缓冲区初始化，缓冲区归CPU，
                                            // 如果归SIE则将启动数据发送
}//end USBPrepareForNextSetupTrf()

/****************************
* USB 控制端点服务程序
****************************/
void USBCtrlEPService()
{
  if(USTAT == EP00_OUT)  //如果上一次是OUT或SETUP令牌，并且是端点0
   {
     if(ep0Bo.Stat.PID == SETUP_TOKEN) // EP0 SETUP,如果是配置设置
        USBCtrlTrfSetupHandler();       // USB控制传输设置处理函数
     else
        USBCtrlTrfOutHandler();         // OUT事务，从主机接收数据
   }
  else if(USTAT == EP00_IN)               // EP0 IN，输入事务
       USBCtrlTrfInHandler();          // IN事务传送数据到主机
  
}//end USBCtrlEPService()

/*******************************
* USB控制传输设置处理函数
********************************/
void USBCtrlTrfSetupHandler()
{
   ctrl_trf_state = WAIT_SETUP;  // USB控制传输状态为等待
   ctrl_trf_session_owner = MUID_NULL; //此时USB EP0端点数据不归任何类所有
   wCount._word = 0;  //字节计数器清0，用于记录传送数据的大小
 
   USBCheckStdRequest();  //检测USB的标准请求

   if(ctrl_trf_session_owner == MUID_NULL)  // 如果上边没有请求，或请求不存在，检测HID请求
   {
     USBCheckHIDRequest();  // USB HID请求检测 
   }//end if
   
   USBCtrlEPServiceComplete();  // 控制传输完成
}//end USBCtrlTrfSetupHandler()

/*******************************
* USB控制数据输出处理函数
* 数据从主机-设备
********************************/
void USBCtrlTrfOutHandler()
{
//说明：此程序是控制传输的数主机请求数据阶段

  if(ctrl_trf_state == CTRL_TRF_RX)  // 如果控制状态是，数据读取（数据主机到设备）
   {
     USBCtrlTrfRxService();          // USB控制数据输出
     if (ctrl_trf_session_owner == MUID_HID)  // 如果此时的传输属于HID所有
      {
        HandleControlOutReport();    // HID控制输出报表（主机输出报表到设备）
      }

    if(ep0Bo.Stat.DTS == 0)          // DATA0与DATA1 交替
      ep0Bo.Stat._byte = _USIE|_DAT1|_DTSEN;
    else
      ep0Bo.Stat._byte = _USIE|_DAT0|_DTSEN;
   }//end if
   else  // 否则这是控制读的状态阶段
    USBPrepareForNextSetupTrf();  // 准备下一次的控制传输
}//end USBCtrlTrfOutHandler()

/********************************
* USB控制数据输入处理函数
* 数据从设备到主机
*********************************/
void USBCtrlTrfInHandler()
{

  mUSBCheckAdrPendingState();  // 因为这个是控制写（假设是设定地址）传输的最后一步，可以算是状态，在这里要检测地址状态
  if(ctrl_trf_state == CTRL_TRF_TX)   // 数据发送
  {
   USBCtrlTrfTxService(); // 数据加载到到发送缓冲区
    if(ep0Bi.Stat.DTS == 0)
      ep0Bi.Stat._byte = _USIE|_DAT1|_DTSEN;
     else
      ep0Bi.Stat._byte = _USIE|_DAT0|_DTSEN;
   }
    else // 否则这是控制写的状态阶段 	
      USBPrepareForNextSetupTrf();
}//end USBCtrlTrfInHandler()
/*****************************
* 控制传输完成函数
******************************/
void USBCtrlEPServiceComplete()
{
 //说明：令牌和包处理位在收到一个设置传输后将置位（禁止），要在改变ep0Bi.Stat or ep0Bo.Stat前清0（使能）
 //控制传输第一步：设置阶段 
   UCONbits.PKTDIS = 0;    //使能令牌和包接收
   
   if(ctrl_trf_session_owner == MUID_NULL)  // 请求不被支持，则直接发出端点停止信号
    { 
      /*
      如果主机的请求不被支持，或不知道怎么回应，则停止端点发出信号，同时准备好EP0
      接收下一次的请求
      */
      ep0Bo.Cnt = EP0_BUFF_SIZE;      // 设置缓冲器大小，这样可以准备接收下一次的令牌
      ep0Bo.ADR = (byte*)&SetupPkt;   // 设置缓冲器地址

      ep0Bo.Stat._byte = _USIE|_BSTALL; // 端点归SIE，禁止端点发出禁止端点信令
      ep0Bi.Stat._byte = _USIE|_BSTALL;
    }
   else                                  // 否则的话请求是能被执行的
    {
       if(SetupPkt.DataDir == 1)         // 如果请求数据设备到主机
        {
          if(SetupPkt.wLength < wCount._word)  // wLength是主机请求长度
             wCount._word = SetupPkt.wLength;  // 按照主机请求字节发送数据
           USBCtrlTrfTxService();              // USB控制传输数据从设备到主机服务程序
           ctrl_trf_state = CTRL_TRF_TX;       // 现在处于CTRL_TRF_TX设备传送数据状态

           ep0Bo.Cnt = EP0_BUFF_SIZE;
           ep0Bo.ADR = (byte*)&SetupPkt;   
           ep0Bo.Stat._byte = _USIE;               // DTSEN = 0，即不使能同步,准备接收主机发送来的信号

           ep0Bi.ADR = (byte*)&CtrlTrfData;
           ep0Bi.Stat._byte = _USIE|_DAT1|_DTSEN;  // 启动发送，启动数据发送
        }
       else                                        // 否则数据从主机到设备
        {
           ctrl_trf_state = CTRL_TRF_RX;           // 控制传输接收状态
          /*
          1.IN 端点准备回应上一次的终止
          */
            ep0Bi.Cnt = 0;  // 准备状态阶段的设备回应，设备在状态阶段回应0长度的数据包
            ep0Bi.Stat._byte = _USIE|_DAT1|_DTSEN; // 发送DATA1数据包 

          /*
          2.准备OUT端点接收数据，这样已经设置了数据大小以及数据区
          */
          ep0Bo.Cnt = EP0_BUFF_SIZE;
          ep0Bo.ADR = (byte*)&CtrlTrfData;
          ep0Bo.Stat._byte = _USIE|_DAT1|_DTSEN;
        }
    }//end if
}//end USBCtrlEPServiceComplete()

/******************************
* USB控制传输（数据设备到主机）
******************************/
//此程序只完成数据的准备工作
void USBCtrlTrfTxService()
{
  WORD byte_to_send;                    // 定义计数字节
  if(wCount._word < EP0_BUFF_SIZE)      // 判断发送的字节数是否小于EP0_BUFF_SIZE
    byte_to_send._word = wCount._word;  // 如果小于则发送实际值
  else
    byte_to_send._word = EP0_BUFF_SIZE; // 否则发送EP0_BUFF_SIZE大小的字节
    
     /*
      接下来,加载发送字节数目,最多8个字节,数据区就8个字节
     */
    ep0Bi.Stat.BC9 = 0;    // 计数字节高两位清0
    ep0Bi.Stat.BC8 = 0;
    ep0Bi.Stat._byte |= MSB(byte_to_send);  // 设置传输字节数目
    ep0Bi.Cnt = LSB(byte_to_send);
// 从总发送字节中减掉本次发送的字节数
    wCount._word = wCount._word - byte_to_send._word;

    pDst.bRam = (byte*)&CtrlTrfData;   // 设定数据区指针，发送数据区（在400区内）
    
    if(usb_stat.ctrl_trf_mem == _ROM)  // 如果数据存放在ROM区,取出数据
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
* USB控制数据输出数据主机到设备
********************************/
void USBCtrlTrfRxService()
{
   WORD byte_to_read;    // 接收数据字节数计数

   MSB(byte_to_read) = 0x03 & ep0Bo.Stat._byte;  // 保留低两位，取出接收字节数，是主机给定的
   LSB(byte_to_read) = ep0Bo.Cnt;


   wCount._word = wCount._word + byte_to_read._word; // 接收字节数加操作
   pSrc.bRam = (byte*)&CtrlTrfData;  
   
    while(byte_to_read._word)  // 数据取出
    {
        *pDst.bRam = *pSrc.bRam;
        pDst.bRam++;
        pSrc.bRam++;
        byte_to_read._word--;
    }//end while(byte_to_read._word) 
   
}//end USBCtrlTrfRxService()

//---------------------------------------------------------end 控制传输函数区


//---------------------------------------------------------USB 传输协议（请求)函数定义

/*******************************
* 检测USB的标准请求
*********************************/
void USBCheckStdRequest()
{
  if(SetupPkt.RequestType != STANDARD) return;   // 如果不是USB标准请求返回
   
  switch(SetupPkt.bRequest)  //选择请求类型
   {
     case SET_ADR:   //=5  主机请求地址配置
      //说明: 设备不会执行此请求,除非它已经设置一个0长度数据信息包,来完成请求的状态阶段.
      //      主机传送状态阶段的令牌信息包给默认地址0,所以设备必须在改变它的地址前检测并响应此信息包
      //  wVAlue字段内容：新设备地址
      //  wIndex字段内容: 0    
       ctrl_trf_session_owner = MUID_USB9;     // MUID_USB9=1 USB9是USB驱动所有
       usb_device_state = ADR_PENDING_STATE;   // 只更新状态，进入到地址未定状态
     break;

     case  GET_DSC:  //6   获得描述符表
      //说明：描述符有7种类型，每一设备至少有一个设备描述符，
      //      以及至少一个配置描述符和一个接口描述符
      // 数据来源：设备
      // 数据长度：传回字节的数目主机给定
      // wValue:   高字节是描述符类型，低字节是描述符数值
      // wIndex:   如果是字符串描述符此字段是语言ID，其他是0
       USBStdGetDscHandler();                  //USB标准获得描述符表处理
     break;  

     case  SET_CFG:   //9   指示设备使用所选择的配置
      //说明：在完成这个请求来指定一个支持的设置配置后，设备会进入配置状态。
      //      许多标准请求需要设备在设置状态
      //数据长度：0
      //wValue字段：低字节指示一个设置配置。如果此字段的数值符合设备所支持的一个配置，设备会选择
      //            该请求的配置。如果是0表示尚未设置配置，进入地址状态。需要新的请求来设置配置
      //wIndex字段：0
      //支持状态：  地址、配置
       USBStdSetCfgHandler();   //USB标准设置设备配置处理
     break;  

     case  GET_CFG:    //8   主机请求目前设备配置的数值
      //说明：如果设备没有设置配置，则传回0
      //数据长度：1
      //wValue字段：0
      //wIndex字段：0   
      ctrl_trf_session_owner = MUID_USB9;  // USB9是USB驱动所有
      pSrc.bRam = (byte*)&usb_active_cfg;  // 数据首地址
      usb_stat.ctrl_trf_mem = _RAM;        // 数据定义在RAM区
      wCount._word = 1;                    // 数据字节计数=1
     break;   

     case   GET_STATUS: // 0  主机请求一个设备、接口或端点的特征状态
      //说明：如果是设备请求只有两个位。位0是自身电源字段。主机不能改变此数值。
      //      位1是远程唤醒字段，在重置事0。如果是接口的请求。所有位保留。如果是
      //      端点请求，只有位0有意义，位0=1是一个暂停态(Halt).
      //数据字节：2
      //wValue字段：0
      //wIndex字段：设备=0。如果是接口为接口号，如果是端点=端点号
      USBStdGetStatusHandler();  // 主机获得状态处理函数
     break;      

     case  CLR_FEATURE:  //1 主机请求禁用一个在设备，接口或端点上的特征
                         //  将继续往下执行
     case SET_FEATURE:   //3 主机请求启用一个在设备，接口或端点上的特征
      //说明：USB规范定义两个特性：即DEVICE_REMOTE_WAKEUP（数值为1，应用在设备）
      //      以及ENDPOINT_HALT(数值为0，应用在端点）
      //数据：无
      //wValue字段：要启用的特性 DEVICE_REMOTE_WAKEUP  1 设备
      //                         ENDPOINT_HALT         0 端点
      //                         TEST_MODE             2 设备
      //wIndex字段：如果是设备的特性此字段为0，如果是接口此字段为接口号码
      //            如果是端点特性，此字段是端点号码
      USBStdFeatureReqHandler();  //主机请求启用特征值处理函数
     break;
  
     case GET_INTF:     //10  如果设备的配置支持多个互不相关设置的接口，主机请求目前的状态        
      //说明：wIndex字段的几口号码，是指接口描述符中的bInterface字段。每一个接口的号码不同
      //      数据字段的内容，是指接口描述符中的bAlternateInterface字段内容。数据字段表示目前
      //      使用的接口中两个或多个互不相关的设置。
      //wValue字段：0
      //wIndex字段：接口号码  
      ctrl_trf_session_owner = MUID_USB9;
      pSrc.bRam = (byte*)&usb_alt_intf+SetupPkt.bIntfID;  // 取出数据,也就是接口号码,一般就一个接口0
      usb_stat.ctrl_trf_mem = _RAM;
      wCount._word = 1;   // 计数字节     
     break;

     case  SET_INTF:   //11  如果设备的配置支持多个互不相关设置的接口，主机请求设备使用一个指定的设置
      ctrl_trf_session_owner = MUID_USB9;
      usb_alt_intf[SetupPkt.bIntfID] = SetupPkt.bAltID;  // 选择替代的接口号替代以前的
     break;
 
     case SET_DSC:     //7   主机新增一个描述符，或是更新一个存在的描述符
     case SYNCH_FRAME: //12  设备设置与报告一个端点的同步帧
     default:
     break;
   }// end switch
}//end USBCheckStdRequest()

/********************************
*  USB标准描述符获得处理函数
*********************************/
void USBStdGetDscHandler()
{
  if(SetupPkt.bmRequestType == 0x80) // 检测是否是GET_DSC
   {
     switch(SetupPkt.bDscType)       // 选择描述符类型
      {
        case DSC_DEV:                // 设备描述符
          ctrl_trf_session_owner = MUID_USB9;  // 控制传输归SETUP所有
          pSrc.bRom = (rom byte*)&device_dsc;  // 取设备描述符区的首地址,bRom两字节指针1字节数据
          wCount._word = sizeof(device_dsc);   // 取出设备描述符大小
        break;
 
        case DSC_CFG:                   // 配置描述符
          ctrl_trf_session_owner = MUID_USB9;  // 控制传输归SETUP所有
          pSrc.bRom = *(USB_CD_Ptr+SetupPkt.bDscIndex); //USB_CD_Ptr是地址指针数组,存放CFG0地址,加上偏移后取*把地址内容也就是把CFG0地址给到指针brom
          wCount._word = *(pSrc.wRom+1); //pSrc.wRom是指针，他的地址与pSrc.bRom相同但是指向2个字节数据
                                         //加1就越过两字节数据到CFG01的sizeof(cfg01),字节这样把配置的总字节数给出
        break;

        case DSC_STR:                       // 字符串描述符   
          ctrl_trf_session_owner = MUID_USB9;
          pSrc.bRom = *(USB_SD_Ptr+SetupPkt.bDscIndex);
          wCount._word = *pSrc.bRom;         // Set data count
        break;
      } //end switch
   }//end if
  usb_stat.ctrl_trf_mem = _ROM;
}//end USBStdGetDscHandler()  

/********************************
*USB指示设备使用所选择的配置处理
*********************************/
void USBStdSetCfgHandler()
{
   ctrl_trf_session_owner = MUID_USB9;           // 控制传输归USB9，设置这个以后可以返回状态
   mDisableEP1to15();                            // 复位端点1——15
   ClearArray((byte*)&usb_alt_intf,MAX_NUM_INT); // 清交替数组（中断端点）本例使用一个其实可以不用
   usb_active_cfg = SetupPkt.bCfgValue;          // 取出主机给定的设置配置
   if(SetupPkt.bCfgValue == 0)                   // 表示尚未设置配置，进入地址状态
    {
      usb_device_state = ADDRESS_STATE;          // 设备进入地址状态
    }//end if
   else
    {
      usb_device_state = CONFIGURED_STATE;       // 否则进入配置状态
      HIDInitEP();                               // HID端点设置
                                                 // 一旦进入配置状态后主机已确认设备将进入数据传输阶段
    } //enf if
}// end USBStdSetCfgHandler()

/******************************
* USB获得状态处理函数
******************************/
void USBStdGetStatusHandler()
{
  CtrlTrfData._byte0 = 0;      // 准备两个字节的数据区
  CtrlTrfData._byte1 = 0;

  switch(SetupPkt.Recipient)   // 选择是设备接口还是端点
   {
     case  RCPT_DEV:           // =0 获取设备状态
      ctrl_trf_session_owner = MUID_USB9;   //归传输协议（请求）所有
      CtrlTrfData._byte0|=0b000000001;      // SetB byte0.0,表示使用自身电源
      if(usb_stat.RemoteWakeup == 1)        // usb_stat defined in usbmmap.c
        CtrlTrfData._byte0|=0b00000010;     // Set bit1，是否支持，远程唤醒
     break;
   
     case  RCPT_INTF:          // =1 获取接口状态
      ctrl_trf_session_owner = MUID_USB9; // 两个数据字段=0；
     break;

     case  RCPT_EP:            // =2 如果是获取端点状态
      ctrl_trf_session_owner = MUID_USB9;
      pDst.bRam = (byte*)&ep0Bo+(SetupPkt.EPNum*8)+(SetupPkt.EPDir*4); // 取出状态寄存器地址
      if(*pDst.bRam & _BSTALL) // 判断端点是否停止
       CtrlTrfData._byte0=0x01;// 如果停止表示端点进入一个暂停态  
     break;
   }// end switch
} // end USBStdGetStatusHandler()

/***********************************
*主机请求启用特征值处理函数
************************************/
void USBStdFeatureReqHandler()
{
  if((SetupPkt.bFeature == DEVICE_REMOTE_WAKEUP)&&(SetupPkt.Recipient == RCPT_DEV))  //RCPT_DEV=0,设置设备的远程唤醒状态状态 
  {
     ctrl_trf_session_owner = MUID_USB9;
     if(SetupPkt.bRequest == SET_FEATURE)  // 判断是否是这个请求
       usb_stat.RemoteWakeup = 1;          // 如果配置远程唤醒，状态位置1
     else  //否则是CLEAR_FEATURE
       usb_stat.RemoteWakeup = 0;   
  }//end if 
  
  if((SetupPkt.bFeature == ENDPOINT_HALT)&&(SetupPkt.Recipient == RCPT_EP)&&(SetupPkt.EPNum != 0))// 加上请求的端点不是端点0条件
  {
    ctrl_trf_session_owner = MUID_USB9;
    pDst.bRam = (byte*)&ep0Bo+(SetupPkt.EPNum*8)+(SetupPkt.EPDir*4); //确定端点状态地址
    if(SetupPkt.bRequest == SET_FEATURE)//如果是端点特征配置
      *pDst.bRam = _USIE|_BSTALL;   //把端点状态寄存器设置为SIE所有并使能缓冲器停止
    else   //否则是CLEAR_FEATURE
     {
       if(SetupPkt.EPDir == 1)    // 如果是IN
        *pDst.bRam = _UCPU;       // 端点归CPU
       else
        *pDst.bRam = _USIE|_DAT0|_DTSEN;  //如果是输出的话，端点归SIE用DATA0使能数据同步位
     }// end if 
  }//end if            
}//end USBStdFeatureReqHandler()

/********************************
* USB检测地址状态
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

//--------------------------------------------------------- end USB 传输协议（请求)函数定义


//--------------------------------------------------------- HID 函数区
/*******************************
* HID 端点设置函数
*******************************/
void HIDInitEP()
{
   hid_rpt_rx_len=0;                            // HID报表接收字节数清0
  
   HID_UEP = EP_OUT_IN|HSHK_EN; 
   ClearArray((byte*)&usb_alt_intf,MAX_NUM_INT);// 清一个数组，为交替设置MAX_NUM_INT=1,中断端点个数=1
   
   HID_UEP = EP_OUT_IN|HSHK_EN;                 // 初始化端点1
   HID_BD_OUT.Cnt = sizeof(hid_report_out);     // 设置缓冲区大小
   HID_BD_OUT.ADR = (byte*)&hid_report_out;     // 设置缓冲区首地址
   HID_BD_OUT.Stat._byte = _USIE|_DAT0|_DTSEN;  // 端点1的OUT设置
                                                
   HID_BD_IN.ADR = (byte*)&hid_report_in;       // 端点1的IN端点地址设置
   HID_BD_IN.Stat._byte = _UCPU|_DAT1;          // 端点1的IN状态设置
}// end HIDInitEP()


/***********************************
* HID检测请求
***********************************/
void USBCheckHIDRequest()   //HID检测请求，如果USB状态不是空，不会检测，因为那是USB的标准请求时刻
{
  if(SetupPkt.Recipient != RCPT_INTF) return;  // 如果请求不是对接口则退出，对HID 都是对端口说话
  if(SetupPkt.bIntfID != HID_INTF_ID) return;  // 如果HID接口的号不与请求的接口ID相同退出
   
  if(SetupPkt.bRequest == GET_DSC)  // 如果是主机获得描述符
   {
/*
#define DSC_HID         0x21   // HID
#define DSC_RPT         0x22   // 报表
#define DSC_PHY         0x23   // 实体
*/
     switch(SetupPkt.bDscType)
      {
        case 0x21:   // 获得HID描述符
         ctrl_trf_session_owner = MUID_HID;  // 传输归HID所有
         if(usb_active_cfg == 1)             // 即进入配置状态以后才做的事情
          {
           pSrc.bRom = (rom byte*)&cfg01.hid_i00a00;  // 取出HID描述符的首地址
          }
         wCount._word = sizeof(USB_HID_DSC); //取得描述符大小
        break;

        case 0x22:       //报表描述符
         ctrl_trf_session_owner = MUID_HID;  // 传输归HID所有
         if(usb_active_cfg == 1)   
         {              
          pSrc.bRom = (rom byte*)&hid_rpt01; // 报表描述符在ROM内的首地址
          wCount._word = sizeof(hid_rpt01);  // 报表描述符大小
         }                
        break;

        case 0x23:
        default:
        break;
      }//end switch
    usb_stat.ctrl_trf_mem = _ROM;   // 数据存储在ROM区
   }//end if

  if(SetupPkt.RequestType != CLASS) return; // 如果不是特定USB的类请求返回
  switch(SetupPkt.bRequest)    // 选择HID特定请求
   {
     case  GET_REPORT:  // 01 启用主机使用控制传输，来从设备读取数据
      HIDGetReportHandler(); // 主机获得数据处理函数
     break;
    
     case SET_REPORT:   // 09 启用设备控制传输，设备从主机获得数据
      HIDSetReportHandler(); // 设备接收数据处理函数
     break; 

     case GET_IDLE:     // 02 启用主机从设备读取目前的闲置率
      ctrl_trf_session_owner = MUID_HID; // 控制传输的系统归HID所有
      pSrc.bRam = (byte*)&idle_rate;     // 数据以4MS为单位，即一个数代表4MS
      usb_stat.ctrl_trf_mem = _RAM;      // Set memory type 数据存储在RAM区
      wCount._word = 1;
     break;   
  
     case SET_IDLE:
      //说明：0x0a 当数据从上一个报表后再没有改变时，闲置中断输入端点的报表频率来节省带宽
      //      如果HID支持此请求则只有在数据改变时才会发送报表  
       ctrl_trf_session_owner = MUID_HID;
       idle_rate = MSB(SetupPkt.W_Value);
     break;  

     case GET_PROTOCOL:                    // 0x03 启用主机了解设备目前作用是启动协议或是报表协议
       ctrl_trf_session_owner = MUID_HID;  // 控制传输的系统归HID所有   
       pSrc.bRam = (byte*)&active_protocol;// Set source
       usb_stat.ctrl_trf_mem = _RAM;
       wCount._word = 1;  // 数据长度
     break;

     case SET_PROTOCOL:   // 0x0b 主机指定使用启动协议或是报表协议
       ctrl_trf_session_owner = MUID_HID;
       active_protocol = LSB(SetupPkt.W_Value); //取出协议类型
     break;  
   }//end switch
}//end USBCheckHIDRequest()


/**********************************
* 主机获得数据处理函数
**********************************/
void HIDGetReportHandler()
{
  switch(MSB(SetupPkt.W_Value))
   {
     case 1:      // 输入报表
     switch(LSB(SetupPkt.W_Value))
      {
        case 0:                   // 选择报表ID
          ctrl_trf_session_owner = MUID_HID;   // 控制传输的系统归HID所有
          GetInputReport0();                   // 准备数据发送到主机
        break;
        case 1:                   // 本例只有一个报表
        break;  
      }
     break;

     case 3:    // 主机获得特征
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
  usb_stat.ctrl_trf_mem = _RAM;               // Set memory type，数据都放在RAM区
}//end HIDGetReportHandler()

/***********************************
* 设备获得数据处理函数
***********************************/
void HIDSetReportHandler()
{
   switch(MSB(SetupPkt.W_Value))
    {
      case 2:      // 设备获得数据
       switch(LSB(SetupPkt.W_Value))  //选择ID号
        {
          case 0:  // 如果是0号报表
           ctrl_trf_session_owner = MUID_HID;
           pDst.bRam = (byte*)&hid_report_out; //设置数据存储区，当主机数据发送到以后会存储到缓冲区
          break;
        }
      break;
  
      case 3:      // 主机设置特征值
       switch(LSB(SetupPkt.W_Value))
        {
          case 0:
           ctrl_trf_session_owner = MUID_HID;
            pDst.bRam = (byte*)&hid_report_feature;  //设置存储缓冲区 
          break;
        } 
      break;  
    }//end switch
}//end HIDSetReportHandler()

/*******************************
* HID取接收字节长度
*******************************/
byte HIDRxReport(char *buffer, byte len) 
{
  hid_rpt_rx_len = 0;
  if(!HID_BD_OUT.Stat.UOWN)  // 接收主机数据任务结束后端点会归CPU所有
  {
    if(len > HID_BD_OUT.Cnt) // 调整期待接收数据字节数等于实际接收的字节数
     len = HID_BD_OUT.Cnt;   // 
    for(hid_rpt_rx_len = 0; hid_rpt_rx_len < len; hid_rpt_rx_len++)  // 数据读取
    {
      buffer[hid_rpt_rx_len] = hid_report_out[hid_rpt_rx_len]; // 把数据读到缓冲区   
    }  
    HID_BD_OUT.Cnt = sizeof(hid_report_out);  // 准备下一次的数据接收，初始化
    HID_BD_OUT.ADR = (byte*)&hid_report_out; 
    mUSBBufferReady(HID_BD_OUT);   // 准备好下一次的接收，交替DATA端点归SIE所有
  }//end if
  return hid_rpt_rx_len;           // 返回接收字节的长度，如果事务没完成
}//end

/********************************
* HID 发送数据到主机
********************************/
void HIDTxReport(char *buffer, byte len)
{
  byte i;
  if(len > HID_INT_IN_EP_SIZE)    //取出发送数据字节
    len = HID_INT_IN_EP_SIZE;

  for (i = 0; i < len; i++)
   {
     hid_report_in[i] = buffer[i];  //把数据写入发送缓冲区
   }  
  HID_BD_IN.Cnt = len;
  mUSBBufferReady(HID_BD_IN);       //数据开始发送
}//end HIDTxRepor


//--------------------------------------------------------- end HID函数区


//--------------------------------------------------------- 用户函数数区
/********************************
* 准备数据发送到主机
********************************/
void GetInputReport0()
{
  byte count;
  pSrc.bRam = (byte*)&hid_report_in;     // 数据指针指向IN REPORT区，即准备好数据
  wCount._word = HID_INPUT_REPORT_BYTES; // 传输数据字节数，就2个字节
}//end GetInputReport0()

/********************************
*
********************************/
void GetFeatureReport0()
{
  byte count;
  pSrc.bRam = (byte*)&hid_report_feature;       // 准备特征数据地址地址
  wCount._word = HID_FEATURE_REPORT_BYTES;      // 传送两个字节
}//end GetFeatureReport0()

/********************************
* HID输出报表（主机发送）
*********************************/
void HandleControlOutReport()
{
	byte count;              //字节计数器
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
* 报表查询
******************************/
//检测是否有主机传来的报表
void ReportLoopback()
{
  byte count;   // 定义计数字节
  //获取读缓冲区字节的数目
  number_of_bytes_read = HIDRxReport(receive_buffer, HID_OUTPUT_REPORT_BYTES); 
  
  if (number_of_bytes_read > 0) // 如果接收字节不为0则有数据传入
   {
/* gcat
     //取出主机发送数据到发送缓冲区
     for(count =0 ; count < HID_OUTPUT_REPORT_BYTES; count++) 
      { 
       transmit_buffer[count] = receive_buffer[count]; //2个字节的数据到发送缓冲区
         
      }//end for      
     while(HID_BD_IN.Stat.UOWN)   // 如果发送缓冲区此时处于发送数据阶段等待
      {
       USBDriverService();
      }//end while
      HIDTxReport(transmit_buffer, HID_INPUT_REPORT_BYTES);  // 数据发送
*/
//自定义函数，实现特定功能
      user();
      
   } //end if
}//end ReportLoopback()

/*******************************
*用户函数
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
//--------------------------------------------------------- end用户函数数区

