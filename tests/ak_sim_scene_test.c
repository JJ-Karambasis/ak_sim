#include <stdio.h>
#include <raylib.h>
#include "../ak_sim.h"

int main() {
    printf("Hello\n");

	SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);

    InitWindow(1920, 1080, "AK Sim");

    ak_sim_create_info CreateInfo = {0};
    ak_sim_context* SimContext = AK_Sim_Create_Context(&CreateInfo);

    const double TargetTime = 1.0/60.0;
    const double MaxTime = 1.0/3.0;
    double LastTime = GetTime();
    double Accumulator = 0.0;

    while (!WindowShouldClose())
    {
        double Time = GetTime();
        double DeltaTime = Time-LastTime;
        LastTime = Time;

        if(DeltaTime > MaxTime) {
            DeltaTime = MaxTime;
        }

        Accumulator += DeltaTime;
        while(Accumulator >= TargetTime) {
            Accumulator -= TargetTime;
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);
        DrawText("Congrats! You created your first window!", 190, 200, 20, LIGHTGRAY);
        EndDrawing();
    }

    AK_Sim_Delete_Context(SimContext);
    CloseWindow();

    return 0;
}

#define AK_SIM_IMPLEMENTATION
#include "../ak_sim.h"