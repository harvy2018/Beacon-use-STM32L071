
#include <stdio.h>
#include "menu.h"
#include "device.h"
#include "OLED_I2C.h"

   
void func_null(void){}

    
    
MenuItem  RFSwitchMenu[2]=
{
   {2, "RF ON",  RFSwitchON,  NULL,  SystemSetMenu}, 
   {2, "RF OFF", RFSwitchOFF, NULL,  SystemSetMenu} 
};

MenuItem  BTSwitchMenu[2]=
{
   {2, "BT ON",  BTSwitchON,  NULL,  SystemSetMenu}, 
   {2, "BT OFF", BTSwitchOFF, NULL,  SystemSetMenu} 
};

MenuItem  BUZZSwitchMenu[2]=
{
   {2, "Buzz ON",  BUZZSwitchON,  NULL,  SystemSetMenu}, 
   {2, "Buzz OFF", BUZZSwitchOFF, NULL,  SystemSetMenu} 
};

MenuItem  ClearDataMenu[1]=
{
   {1, " ",  ClearData,  NULL,  SystemSetMenu}
   
};



MenuItem  SystemSetMenu[4]=
{
   {4, "RF ON/OFF",  func_null, RFSwitchMenu,  MainMenu}, 
   {4, "BT ON/OFF",  func_null, BTSwitchMenu,  MainMenu},
   {4, "Buzz ON/OFF",  func_null, BUZZSwitchMenu,  MainMenu}, 
   {4, "ClearData!",  func_null, ClearDataMenu,  MainMenu} 
};


MenuItem RecordReviewMenu[1]={1, " ",  Record_Review, NULL, MainMenu};
MenuItem RecordStatisMenu[1]={1, " ",  Record_Statis, NULL, MainMenu};
MenuItem DeviceInfoMenu[1]=  {1, " ",  DeviceInfoShow, NULL, MainMenu};
MenuItem ScreenTestMenu[1]=  {1, " ",  ScreenTest, NULL, MainMenu};


MenuItem  MainMenu[5]=
{
   {5, "System Set",     func_null, SystemSetMenu,     NULL}, 
   {5, "Record Review", func_null, RecordReviewMenu, NULL}, 
   {5, "Record Count",  func_null, RecordStatisMenu,  NULL}, 
   {5, "Device Info",   func_null, DeviceInfoMenu,    NULL},
   {5, "Screen Test",   func_null, ScreenTestMenu,    NULL},
};


MenuItem (*MenuPoint) = MainMenu;

u8 DisplayStart = 0;
u8 UserChoose = 0;
u8 DisplayPoint = 0;
u8 MaxItems; 



void ShowMenu(void)
{

    u8 n = 0;

    MaxItems = MenuPoint[0].MenuCount;

    DisplayPoint = DisplayStart;   

    if (MaxItems >= MAX_MENU_ITEM)
    {
         for (n = 0;n < MAX_MENU_ITEM; n++)
         {
                    
            if(memcmp(MenuPoint[DisplayPoint].DisplayStr," ",1))
            {
                OLED_ShowStr(18,(n+1)<<1,MenuPoint[DisplayPoint].DisplayStr,2);  
             
                 if (DisplayPoint == UserChoose)
                 {
                   
                     OLED_ShowStr(0,(UserChoose%MAX_MENU_ITEM+1)<<1,"->",2);  

                 } 
             }             

             DisplayPoint +=1;
             
             if ((DisplayPoint) == (MaxItems))
             {
                 DisplayPoint = 0;

             }

         }

     }
     else
     {

         for (n = 0;n<MaxItems;n++)
         {
                     
            if(memcmp(MenuPoint[DisplayPoint].DisplayStr," ",1)) 
            {
                OLED_ShowStr(18,(n+1)<<1,MenuPoint[DisplayPoint].DisplayStr,2);  
             
                 if ((DisplayPoint) == UserChoose)
                 {
                   OLED_ShowStr(0,(UserChoose%MAX_MENU_ITEM+1)<<1,"->",2);  

                 } 
             }   
            
             DisplayPoint +=1;

             if ((DisplayPoint) == (MaxItems))
             {
                 DisplayPoint = 0;

             }          

         }

     }



}



void Menu_Process(u8 *ButtonState)  
{
   extern u8 ScreenTest_Flag;
    
    if(ButtonState[BACK]==1)  
     {
        if(ScreenTest_Flag)
        {              
            OLED_CLS();
            Battery_Mgr();
            ScreenTest_Flag=0; 
        }
        else
           OLED_CLS_B(); 
         
         if (MenuPoint[0].ParentMenus != NULL)
         {
             MenuPoint = MenuPoint[0].ParentMenus;
             UserChoose = 0;
             DisplayStart = 0;
         }
         
         if(ScreenTest_Flag)
             ScreenTest_Flag=0;
                 
     }
     
     if(ButtonState[OK]==1)  
     {
         OLED_CLS_B();
         
         if (MenuPoint[UserChoose].Subs != func_null)
         {
             (*MenuPoint[UserChoose].Subs)();

         }
         else if (MenuPoint[UserChoose].ChildrenMenus !=NULL)
         {

             MenuPoint = MenuPoint[UserChoose].ChildrenMenus;
             UserChoose = 0;
             DisplayStart = 0;
             DisplayPoint=0;

         }

     }
     
     
     if(ButtonState[NEXT]==1)  
     {
          
         OLED_CLS_B();
         
         UserChoose++;
         if (UserChoose == MaxItems)
         {
            UserChoose = 0;
         }
                 
     }
     
     if ((UserChoose < DisplayStart) || (UserChoose > (DisplayStart+MAX_MENU_ITEM-1)))
     {

        DisplayStart = UserChoose;

     }
     
     if(!memcmp(MenuPoint[DisplayPoint].DisplayStr," ",1))
     {
        
         (*MenuPoint[UserChoose].Subs)(ButtonState);
     }
     
     if(ButtonState[NEXT]==1 || ButtonState[OK]==1 || ButtonState[BACK]==1)
       ShowMenu();
     
}



