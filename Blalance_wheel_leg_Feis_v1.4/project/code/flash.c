#include "zf_common_headfile.h"
#include "flash.h"

static uint8 nag_flash_index_read = 0;

static uint32 flash_Nag_EventChecksum(uint8 event_count)
{
    uint32 checksum = Nag_Event_Version ^ (uint32)event_count;
    uint8 event_index = 0;

    for (event_index = 0; event_index < event_count; event_index++)
    {
        uint32 packed_index = ((uint32)Nag_Event_Table[event_index].enter_index << 16) |
                              (uint32)Nag_Event_Table[event_index].exit_index;
        uint32 packed_meta = ((uint32)Nag_Event_Table[event_index].type << 8) |
                             (uint32)Nag_Event_Table[event_index].valid;
        checksum ^= packed_index;
        checksum ^= packed_meta;
    }
    return checksum;
}

static void flash_Nag_ClearEventTable(void)
{
    memset(Nag_Event_Table, 0, sizeof(Nag_Event_Table));
    N.Event_Count = 0;
    N.Event_Record_Pending = 0;
}

static void flash_Nag_WriteEventPage(void)
{
    uint8 event_index = 0;

    flash_buffer_clear();
    flash_union_buffer[0].uint32_type = Nag_Event_Magic;
    flash_union_buffer[1].uint32_type = Nag_Event_Version;
    flash_union_buffer[2].uint32_type = N.Event_Count;
    flash_union_buffer[3].uint32_type = flash_Nag_EventChecksum(N.Event_Count);

    for (event_index = 0; event_index < N.Event_Count; event_index++)
    {
        uint16 base = (uint16)(4 + event_index * 2);
        flash_union_buffer[base].uint32_type = ((uint32)Nag_Event_Table[event_index].enter_index << 16) |
                                               (uint32)Nag_Event_Table[event_index].exit_index;
        flash_union_buffer[base + 1].uint32_type = ((uint32)Nag_Event_Table[event_index].type << 8) |
                                                   (uint32)Nag_Event_Table[event_index].valid;
    }

    if (flash_check(0, Nag_Event_Page))
    {
        flash_erase_page(0, Nag_Event_Page);
    }
    flash_write_page_from_buffer(0, Nag_Event_Page, FLASH_PAGE_LENGTH);
    flash_buffer_clear();
}

static void flash_Nag_ReadEventPage(void)
{
    uint32 event_magic = 0;
    uint32 event_version = 0;
    uint32 event_count = 0;
    uint32 event_checksum = 0;
    uint32 expect_checksum = 0;
    uint8 event_index = 0;

    flash_Nag_ClearEventTable();

    if (!flash_check(0, Nag_Event_Page))
    {
        return;
    }

    flash_buffer_clear();
    flash_read_page_to_buffer(0, Nag_Event_Page, FLASH_PAGE_LENGTH);
    event_magic = flash_union_buffer[0].uint32_type;
    event_version = flash_union_buffer[1].uint32_type;
    event_count = flash_union_buffer[2].uint32_type;
    event_checksum = flash_union_buffer[3].uint32_type;

    if (event_magic != Nag_Event_Magic || event_version != Nag_Event_Version || event_count > Nag_Event_Max)
    {
        flash_buffer_clear();
        return;
    }

    for (event_index = 0; event_index < event_count; event_index++)
    {
        uint16 base = (uint16)(4 + event_index * 2);
        uint32 packed_index = flash_union_buffer[base].uint32_type;
        uint32 packed_meta = flash_union_buffer[base + 1].uint32_type;

        Nag_Event_Table[event_index].enter_index = (uint16)(packed_index >> 16);
        Nag_Event_Table[event_index].exit_index = (uint16)(packed_index & 0xFFFFu);
        Nag_Event_Table[event_index].type = (uint8)((packed_meta >> 8) & 0xFFu);
        Nag_Event_Table[event_index].valid = (uint8)(packed_meta & 0xFFu);

        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].enter_index >= N.Save_index ||
            Nag_Event_Table[event_index].exit_index >= N.Save_index ||
            Nag_Event_Table[event_index].exit_index < Nag_Event_Table[event_index].enter_index)
        {
            flash_Nag_ClearEventTable();
            flash_buffer_clear();
            return;
        }
    }

    N.Event_Count = (uint8)event_count;
    expect_checksum = flash_Nag_EventChecksum(N.Event_Count);
    if (expect_checksum != event_checksum)
    {
        flash_Nag_ClearEventTable();
    }
    flash_buffer_clear();
}

void flash_Nag_Write(){
   
    if(flash_check(0, N.Flash_page_index))flash_erase_page(0, N.Flash_page_index);                  
                       
    flash_write_page_from_buffer(0,N.Flash_page_index,FLASH_PAGE_LENGTH);
    if(N.End_f == 1)
    {    
     flash_buffer_clear();
     flash_union_buffer[MaxSize+2].uint32_type = N.Save_index;
     if(flash_check(0, Nag_End_Page))flash_erase_page(0, Nag_End_Page);
     flash_write_page_from_buffer(0,Nag_End_Page,FLASH_PAGE_LENGTH);
     flash_Nag_WriteEventPage();
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
        flash_Nag_ReadEventPage();
    }
    if(flash_check(0, N.Flash_page_index))
    {        
        flash_read_page_to_buffer(0, N.Flash_page_index,FLASH_PAGE_LENGTH);
    }
}
