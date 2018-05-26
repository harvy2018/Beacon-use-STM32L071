
#ifndef _MENUITEM_H_
#define _MENUITEM_H_


#include "stm32f10x.h"


#define MAX_MENU_ITEM  3

typedef struct _MenuItem 
{ 
    unsigned char MenuCount;    
    char *DisplayStr; 
    void (*Subs)(); 
    struct _MenuItem *ChildrenMenus;
    struct _MenuItem *ParentMenus;
    
}MenuItem;


extern MenuItem  MainMenu[];
extern MenuItem  SystemSetMenu[];
extern MenuItem  RecordPreviewMenu[];
extern MenuItem  RecordStatisMenu[];
extern MenuItem  DeviceInfoMenu[];
extern MenuItem  RFSwitchMenu[];
extern MenuItem  BTSwitchMenu[];

void Menu_Process(u8 *ButtonState);
void ShowMenu(void);
void Record_Review(u8 *ButtonState);
void Record_Statis(u8 *ButtonState);
#endif