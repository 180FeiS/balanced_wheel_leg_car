#include "zf_common_headfile.h"
#include "flash.h"

static uint8 nag_flash_index_read = 0;

void flash_Nag_Write(){
   
    if(flash_check(0, N.Flash_page_index))flash_erase_page(0, N.Flash_page_index);                  
                       
    flash_write_page_from_buffer(0,N.Flash_page_index,FLASH_PAGE_LENGTH);
    if(N.End_f == 1)
    {    
     flash_union_buffer[MaxSize+2].uint32_type = N.Save_index;
     flash_write_page_from_buffer(0,Nag_End_Page,FLASH_PAGE_LENGTH);
    }
    
    
    flash_buffer_clear();
    

   
}

void flash_Nag_ResetReadState(void)
{
    nag_flash_index_read = 0;
}

void flash_Nag_Read(){
    flash_buffer_clear();

    if(0 == nag_flash_index_read)
    {
       flash_read_page_to_buffer(0,Nag_End_Page,FLASH_PAGE_LENGTH);
        N.Save_index = flash_union_buffer[MaxSize+2].uint32_type;       
        nag_flash_index_read = 1;
        flash_buffer_clear();
    }
    if(flash_check(0, N.Flash_page_index))
    {        
        flash_read_page_to_buffer(0, N.Flash_page_index,FLASH_PAGE_LENGTH);
    }
}
