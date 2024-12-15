#include "../ak_sim.h"

void Memory_Clear(void* Data, size_t DataSize) {
    uint8_t* DataAt = (uint8_t*)Data;
    while(DataSize--) {
        *DataAt = 0;
    } 
}

int main() {
    ak_sim_create_info CreateInfo;
    Memory_Clear(&CreateInfo, sizeof(ak_sim_create_info));
    ak_sim_context* Context = AK_Sim_Create_Context(&CreateInfo);
    AK_Sim_Delete_Context(Context);
}

#define AK_SIM_IMPLEMENTATION
#include "../ak_sim.h"