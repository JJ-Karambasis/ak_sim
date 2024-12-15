#ifndef AK_SIM_H
#define AK_SIM_H

#include <stdint.h>
#include <stddef.h>

#ifndef AKSIMDEF
# ifdef AK_SIM_STATIC
# define AKSIMDEF static
# else
# define AKSIMDEF extern
# endif
#endif

typedef struct ak_sim_context ak_sim_context;

typedef void* ak_sim_allocate_memory_func(size_t Size, void* UserData);
typedef void  ak_sim_free_memory_func(void* Memory, void* UserData);

typedef struct {
    ak_sim_allocate_memory_func* AllocateMemory;
    ak_sim_free_memory_func* FreeMemory;
    void* UserData;
} ak_sim_allocator;

typedef struct {
    float Data[4]; /*Add an additional element for some padding*/
} ak_sim_v3;

AKSIMDEF ak_sim_v3 AK_Sim_V3(float x, float y, float z);

typedef struct {
    float Data[4];
} ak_sim_v4;

typedef struct {
    float Data[4];
} ak_sim_quat;

typedef struct {
    ak_sim_v3   Position;
    ak_sim_quat Orientation;
} ak_sim_transform;

typedef union {
    float     Data[9];
    ak_sim_v3 Cols[3];
} ak_sim_m3;

typedef union {
    float     Data[12];
    ak_sim_v3 Cols[4];
} ak_sim_m4x3;

typedef struct {
    uint32_t FirstVtx;
    uint32_t VtxCount;
} ak_sim_face;

typedef struct {
    ak_sim_v4 NormalD;
} ak_sim_plane;

typedef enum {
    AK_SIM_SHAPE_TYPE_CONVEX,
    AK_SIM_SHAPE_TYPE_MESH,
    AK_SIM_SHAPE_TYPE_COMPOUND,
    AK_SIM_SHAPE_TYPE_USER
} ak_sim_shape_type;

#define AK_SIM_SHAPE_TYPE_COUNT AK_SIM_SHAPE_TYPE_USER

typedef enum {
    AK_SIM_CONVEX_TYPE_SPHERE,
    AK_SIM_CONVEX_TYPE_CAPSULE,
    AK_SIM_CONVEX_TYPE_HULL,
    AK_SIM_CONVEX_TYPE_USER
} ak_sim_convex_type;

#define AK_SIM_CONVEX_TYPE_COUNT AK_SIM_CONVEX_TYPE_USER

typedef struct {
    float Radius;
} ak_sim_sphere;

typedef struct {
    float Radius;
    float HalfHeight;
} ak_sim_capsule;

typedef struct {
    ak_sim_v3*    Vertices;
    ak_sim_face*  Faces;
    ak_sim_plane* Planes;
    uint32_t      VtxCount;
    uint32_t      FaceCount;
} ak_sim_hull;

typedef struct {
    ak_sim_v3* Vertices;
    uint32_t*  Indices; /*Do we need to support 32 bit indices?*/
    uint32_t   VtxCount;
    uint32_t   IdxCount;
} ak_sim_triangle_mesh;

typedef struct {
    ak_sim_hull* Hull;
} ak_sim_hull_inst;

typedef struct {
    ak_sim_triangle_mesh* Mesh;
} ak_sim_triangle_mesh_inst;

typedef struct {
    ak_sim_convex_type Type;
    union {
        ak_sim_sphere    Sphere;
        ak_sim_capsule   Capsule;
        ak_sim_hull_inst Hull;
        void*            User;
    } Internal;
} ak_sim_convex;

typedef struct ak_sim_generic_shape ak_sim_generic_shape;
typedef struct {
    uint32_t              ShapeCount;
    ak_sim_generic_shape* Shapes;
} ak_sim_compound_shape;

typedef struct {
    ak_sim_shape_type Type;
    union {
        ak_sim_convex             Convex;
        ak_sim_triangle_mesh_inst TriangleMesh;
        ak_sim_compound_shape     Compound;
        void*                     User;
    } Internal;
} ak_sim_shape;

struct ak_sim_generic_shape {
    ak_sim_transform Transform;
    ak_sim_shape Shape;
};

typedef struct ak_sim_collision_collector ak_sim_collision_collector;

typedef void ak_sim_collision_func(ak_sim_collision_collector* Collector, ak_sim_shape* ShapeA, const ak_sim_m4x3* TransformA, ak_sim_v3 ScaleA, ak_sim_shape* ShapeB, const ak_sim_m4x3* TransformB, ak_sim_v3 ScaleB);

typedef struct {


    ak_sim_collision_func* CollisionFunc;
} ak_sim_shape_vtable;

typedef struct {

} ak_sim_convex_vtable;

typedef struct {
    ak_sim_shape_type      Type;
    ak_sim_collision_func* CollisionFunc;
} ak_sim_collision_registration;

typedef struct {
    ak_sim_shape_type              Type;
    uint32_t                       CollisionFuncCount;
    ak_sim_collision_registration* Collisions;
} ak_sim_shape_registration;

typedef struct {
    ak_sim_allocator            Allocator;
    ak_sim_shape_registration*  ShapeRegistrations;
    uint32_t                    ShapeRegistrationCount;
} ak_sim_create_info;

AKSIMDEF ak_sim_context* AK_Sim_Create_Context(const ak_sim_create_info* CreateInfo);
AKSIMDEF void AK_Sim_Delete_Context(ak_sim_context* Context);
AKSIMDEF void AK_Sim_Update(ak_sim_context* Context, float DeltaTime);

typedef struct {
    /*Properties for generic shapes*/
    ak_sim_shape_type     ShapeType;
    ak_sim_triangle_mesh* TriangleMesh;
    ak_sim_compound_shape CompoundShape;
    void*                 ShapeUserData;

    /*Properties for convex shapes*/
    ak_sim_convex_type ConvexType;
    ak_sim_sphere      Sphere;
    ak_sim_capsule     Capsule;
    ak_sim_hull*       Hull;
    void*              ConvexUserData;
} ak_sim_shape_info;

typedef struct {
    ak_sim_shape_info ShapeInfo;
    ak_sim_v3         Position;
    ak_sim_quat       Orientation;
    ak_sim_v3         Scale;
    void*             UserData;
} ak_sim_body_create_info;

typedef uint64_t ak_sim_body_id;
typedef struct {
    ak_sim_body_id ID;
    ak_sim_transform Transform;
    ak_sim_v3        Scale;
    ak_sim_shape     Shape;
    void* UserData;
} ak_sim_body;

AKSIMDEF ak_sim_body_id AK_Sim_Create_Body(ak_sim_context* Context, const ak_sim_body_create_info* CreateInfo);
AKSIMDEF void AK_Sim_Delete_Body(ak_sim_context* Context, ak_sim_body_id BodyID);

#endif

#ifdef AK_SIM_IMPLEMENTATION

#ifndef AK_SIM_ASSERT
#include <assert.h>
#define AK_SIM_ASSERT(c) assert(c)
#endif

#ifndef AK_SIM_MEMCPY
#include <string.h>
#define AK_SIM_MEMCPY(dst, src, size) memcpy(dst, src, size)
#endif

#ifndef AK_SIM_MEMSET
#include <string.h>
#define AK_SIM_MEMSET(dst, value, size) memset(dst, value, size)
#endif

#define AK_Sim__Align_Pow2(x, a) (((x) + (a)-1) & ~((a)-1))
#define AK_Sim__Is_Pow2(x) (((x) != 0) && (((x) & ((x) - 1)) == 0))
#define AK_Sim__Max(a, b) (((a) > (b)) ? (a) : (b))
#define AK_Sim__Min(a, b) (((a) < (b)) ? (a) : (b))

#ifndef AK_SIM_NO_STDLIB

#include <stdlib.h>
static void* AK_Sim__Stdio_Allocate_Memory(size_t Size, void* UserData) {
    return malloc(Size);
}

static void AK_Sim__Stdio_Free_Memory(void* Memory, void* UserData) {
    free(Memory);
}

static ak_sim_allocator AK_Sim__Get_Stdio_Allocator() {
    static ak_sim_allocator Allocator;
    if(!Allocator.AllocateMemory) {
        Allocator.AllocateMemory = AK_Sim__Stdio_Allocate_Memory;
        Allocator.FreeMemory = AK_Sim__Stdio_Free_Memory;
    }
    return Allocator;
}

#endif

static void* AK_Sim__Allocate_Memory(ak_sim_allocator* Allocator, size_t Size) {
    return Allocator->AllocateMemory(Size, Allocator->UserData);
}

static void AK_Sim__Free_Memory(ak_sim_allocator* Allocator, void* Memory) {
    Allocator->FreeMemory(Memory, Allocator->UserData);
}

#define AK_Sim__Allocate_Struct(allocator, type) (type*)AK_Sim__Allocate_Memory(allocator, sizeof(type))

static uint32_t AK_Sim__Ceil_Pow2_U32(uint32_t V) {
    V--;
    V |= V >> 1;
    V |= V >> 2;
    V |= V >> 4;
    V |= V >> 8;
    V |= V >> 16;
    V++;
    return V;
}

static ak_sim_m3 AK_Sim__Quat_To_M3(ak_sim_quat Q) {
    float qxqy = Q.Data[0]*Q.Data[1];
	float qwqz = Q.Data[3]*Q.Data[2];
	float qxqz = Q.Data[0]*Q.Data[2];
	float qwqy = Q.Data[3]*Q.Data[1];
	float qyqz = Q.Data[1]*Q.Data[2];
	float qwqx = Q.Data[3]*Q.Data[0];
	 
	float qxqx = Q.Data[0]*Q.Data[0];
	float qyqy = Q.Data[1]*Q.Data[1];
	float qzqz = Q.Data[2]*Q.Data[2];

    ak_sim_m3 Result;
    Result.Cols[0] = AK_Sim_V3(1 - 2*(qyqy+qzqz), 2*(qxqy+qwqz),     2*(qxqz-qwqy));
    Result.Cols[1] = AK_Sim_V3(2*(qxqy-qwqz),     1 - 2*(qxqx+qzqz), 2*(qyqz+qwqx));
    Result.Cols[2] = AK_Sim_V3(2*(qxqz+qwqy),     2*(qyqz-qwqx),     1 - 2*(qxqx+qyqy));
    return Result;
}

static ak_sim_m4x3 AK_Sim__Get_Matrix_Transform(const ak_sim_transform* Transform) {
    ak_sim_m3 Orientation = AK_Sim__Quat_To_M3(Transform->Orientation);

    ak_sim_m4x3 Result;
    Result.Cols[0] = Orientation.Cols[0];
    Result.Cols[1] = Orientation.Cols[1];
    Result.Cols[2] = Orientation.Cols[2];
    Result.Cols[3] = Transform->Position;

    return Result;
}

typedef struct ak_sim__arena_block ak_sim__arena_block;

struct ak_sim__arena_block {
    uint8_t* At;
    uint8_t* End;
    uint8_t* Start;
    ak_sim__arena_block* Next;
};

#define AK_SIM__DEFAULT_ARENA_BLOCK_SIZE (1024*1024)
typedef struct {
    ak_sim_allocator     BaseAllocator;
    ak_sim_allocator*    Allocator;
    ak_sim__arena_block* First;
    ak_sim__arena_block* Last;
    ak_sim__arena_block* Current;
} ak_sim__arena;

typedef struct {
    ak_sim__arena*       Arena;
    ak_sim__arena_block* Block;
    uint8_t*             BlockAt;
} ak_sim__temp_arena;

static void* AK_Sim__Arena_Push(ak_sim__arena* Arena, size_t Size);
static void* AK_Sim__Arena_Allocate(size_t Size, void* UserData) {
    ak_sim__arena* Arena = (ak_sim__arena*)UserData;
    return AK_Sim__Arena_Push(Arena, Size);
}

static void AK_Sim__Arena_Free(void* Memory, void* UserData) {
    /*Noop*/
}

static void AK_Sim__Arena_Create(ak_sim__arena* Arena, ak_sim_allocator* Allocator) {
    Arena->BaseAllocator.AllocateMemory = AK_Sim__Arena_Allocate;
    Arena->BaseAllocator.FreeMemory = AK_Sim__Arena_Free;
    Arena->BaseAllocator.UserData = Arena;
    Arena->Allocator = Allocator;
    Arena->First = NULL;
    Arena->Last = NULL;
    Arena->Current = NULL;
}

static void AK_Sim__Arena_Delete(ak_sim__arena* Arena) {
    ak_sim_allocator* Allocator = Arena->Allocator;
    ak_sim__arena_block* Block = Arena->First;
    while(Block) {
        ak_sim__arena_block* BlockToDelete = Block;
        Block = Block->Next;
        AK_Sim__Free_Memory(Allocator, BlockToDelete);
    }

    Arena->Allocator = NULL;
    Arena->First = NULL;
    Arena->Last = NULL;
    Arena->Current = NULL;
}

static ak_sim__arena_block* AK_Sim__Arena_Get_Current_Block(ak_sim__arena* Arena, size_t Size, size_t Alignment) {
    if(!Arena->Current) return NULL;

    ak_sim__arena_block* Block = Arena->Current;
    do {
        uint8_t* At = (uint8_t*)AK_Sim__Align_Pow2((size_t)Block->At, Alignment);
        if(At + Size <= Block->End) {
            return Block;
        }
        Block = Block->Next;
    } while(Block);

    return NULL;
}

static void* AK_Sim__Arena_Push_Aligned(ak_sim__arena* Arena, size_t Size, size_t Alignment) {
    AK_SIM_ASSERT(AK_Sim__Is_Pow2(Alignment));

    ak_sim__arena_block* CurrentBlock = AK_Sim__Arena_Get_Current_Block(Arena, Size, Alignment);
    if(!CurrentBlock) {
        size_t BlockSize = AK_Sim__Max(Alignment+Size, AK_SIM__DEFAULT_ARENA_BLOCK_SIZE);
        ak_sim__arena_block* Block = (ak_sim__arena_block*)AK_Sim__Allocate_Memory(Arena->Allocator, BlockSize+sizeof(ak_sim__arena_block));
        Block->Start = (uint8_t*)(Block+1);
        Block->At = Block->Start;
        Block->End = Block->Start+BlockSize;
        CurrentBlock = Block;

        if(!Arena->First) {
            Arena->First = Arena->Last = Block;
        }
        else {
            AK_SIM_ASSERT(Arena->Last);
            Arena->Last->Next = Block;
            Arena->Last = Block;
        }
    }

    Arena->Current = CurrentBlock;
    Arena->Current->At = (uint8_t*)AK_Sim__Align_Pow2((size_t)Arena->Current->At, Alignment);
    uint8_t* Result = Arena->Current->At;
    return Result;
}

static void* AK_Sim__Arena_Push(ak_sim__arena* Arena, size_t Size) {
    return AK_Sim__Arena_Push_Aligned(Arena, Size, 16);
}

static ak_sim__temp_arena AK_Sim__Arena_Begin_Temp(ak_sim__arena* Arena) {
    ak_sim__temp_arena Result;
    Result.Arena = Arena;
    Result.Block = Arena->Current;
    Result.BlockAt = Arena->Current ? Arena->Current->At : NULL;
    return Result;
}

static void AK_Sim__Arena_End_Temp(ak_sim__temp_arena* TempArena) {
    ak_sim__arena* Arena = TempArena->Arena;
    
    ak_sim__arena_block* Current = TempArena->Block ? TempArena->Block : Arena->First;
    while(Current) {
        Current->At = Current->Start; /*Reset all the blocks affected*/
        Current = Current->Next;
    }

    if(TempArena->Block) {
        Arena->Current = TempArena->Block;
        Arena->Current->At = TempArena->BlockAt;
    }
}

#define AK_Sim__Arena_Push_Struct(arena, type) (type*)AK_Sim__Arena_Push(arena, sizeof(type))
#define AK_Sim__Arena_Push_Array(arena, count, type) (type*)AK_Sim__Arena_Push(arena, sizeof(type)*(count))

typedef struct {
    ak_sim_allocator* Allocator;
    uint8_t* Data;
    size_t   DataSize;
    uint32_t Capacity;
    uint32_t Count;
} ak_sim__array;

static void AK_Sim__Array_Init(ak_sim__array* Array, ak_sim_allocator* Allocator, size_t DataSize) {
    AK_SIM_MEMSET(Array, 0, sizeof(ak_sim__array));
    Array->Allocator = Allocator;
    Array->Data = NULL;
    Array->DataSize = DataSize;
    Array->Capacity = 0;
    Array->Count = 0;
}

static void* AK_Sim__Array_Get(ak_sim__array* Array, uint32_t Index) {
    AK_SIM_ASSERT(Index < Array->Count);
    void* Result = Array->Data + Index*Array->DataSize;
    return Result;
}

static void AK_Sim__Array_Add(ak_sim__array* Array, const void* Data) {
    if(Array->Count == Array->Capacity) {
        uint32_t NewCapacity = Array->Capacity ? Array->Capacity*2 : 64;
        uint8_t* NewData = (uint8_t*)AK_Sim__Allocate_Memory(Array->Allocator, NewCapacity*Array->DataSize);

        if(Array->Data) {
            AK_SIM_MEMCPY(NewData, Array->Data, Array->Capacity*Array->DataSize);
            AK_Sim__Free_Memory(Array->Allocator, Array->Data);
        }

        Array->Data = NewData;
        Array->Capacity = NewCapacity;
    }

    uint32_t Index = Array->Count++;
    void* DstData = AK_Sim__Array_Get(Array, Index);
    AK_SIM_MEMCPY(DstData, Data, Array->DataSize);
}

typedef uint32_t ak_sim__key_hash_func(const void*);
typedef int ak_sim__key_comp_func(const void*, const void*);

typedef struct {
	uint32_t Hash;
	uint32_t ItemIndex;
	uint32_t BaseCount;
} ak_sim__hash_slot;

typedef struct {
	ak_sim_allocator*      Allocator;
	ak_sim__hash_slot*     Slots;
	uint32_t 	   	       SlotCapacity;
	uint8_t*      	       Keys;
	uint32_t*              ItemSlots;
	size_t 	   	           KeySize;
	uint32_t 		       ItemCapacity;
	uint32_t 		       ItemCount;
	ak_sim__key_hash_func* HashFunc;
	ak_sim__key_comp_func* CompareFunc;
} ak_sim__set;

#define AK_SIM__HASH_INVALID_SLOT ((uint32_t)-1)
static void AK_Sim__Set_Expand_Items(ak_sim__set* Set, uint32_t ItemCapacity) {
	size_t TotalKeySize = Set->KeySize * ItemCapacity; 
	size_t TotalSlotSize = sizeof(uint32_t) * ItemCapacity; 

	uint8_t* Data = (uint8_t*)AK_Sim__Allocate_Memory(Set->Allocator, TotalKeySize + TotalSlotSize);

	uint8_t* NewKeyData   = Data;
	uint32_t* NewSlotData  = (uint32_t*)(NewKeyData + TotalKeySize);

    uint32_t i;
	for (i = 0; i < ItemCapacity; i++) {
		NewSlotData[i] = AK_SIM__HASH_INVALID_SLOT;
	}

	AK_SIM_MEMCPY(NewKeyData, Set->Keys, Set->KeySize*Set->ItemCapacity);
	AK_SIM_MEMCPY(NewSlotData, Set->Slots, sizeof(uint32_t)*Set->ItemCapacity);

	if(Set->Keys)
		AK_Sim__Free_Memory(Set->Allocator, Set->Keys);
	
	Set->Keys = NewKeyData;
	Set->ItemSlots = NewSlotData;
	Set->ItemCapacity = ItemCapacity;
}

static void AK_Sim__Set_Expand_Slots(ak_sim__set* Set, uint32_t NewCapacity) {
	NewCapacity = AK_Sim__Ceil_Pow2_U32(NewCapacity);
	uint32_t SlotMask = NewCapacity - 1;

	ak_sim__hash_slot* NewSlots = (ak_sim__hash_slot*)AK_Sim__Allocate_Memory(Set->Allocator, NewCapacity * sizeof(ak_sim__hash_slot));
	
    uint32_t i;
    for (i = 0; i < NewCapacity; i++) {
		NewSlots[i].ItemIndex = AK_SIM__HASH_INVALID_SLOT;
	}

	for (i = 0; i < Set->SlotCapacity; i++) {
		if (Set->Slots[i].ItemIndex != AK_SIM__HASH_INVALID_SLOT) {
			uint32_t Hash = Set->Slots[i].ItemIndex;
			uint32_t BaseSlot = (Hash & SlotMask);
			uint32_t Slot = BaseSlot;
			while (NewSlots[Slot].ItemIndex != AK_SIM__HASH_INVALID_SLOT) {
				Slot = (Slot + 1) & SlotMask;
			}
			NewSlots[Slot].Hash = Hash;
			uint32_t ItemIndex = Set->Slots[i].ItemIndex;
			NewSlots[Slot].ItemIndex = ItemIndex;
			Set->ItemSlots[ItemIndex] = Slot;
			NewSlots[BaseSlot].BaseCount++;
		}
	}

	if (Set->Slots) {
		AK_Sim__Free_Memory(Set->Allocator, Set->Slots);
	}
	Set->Slots = NewSlots;
	Set->SlotCapacity = NewCapacity;
}

static void AK_Sim__Set_Init(ak_sim__set* Set, ak_sim_allocator* Allocator, size_t KeySize, ak_sim__key_hash_func* HashFunc,
                             ak_sim__key_comp_func* CompareFunc) {
    AK_SIM_MEMSET(Set, 0, sizeof(ak_sim__set));
	Set->Allocator = Allocator;
	Set->KeySize = KeySize;

	Set->HashFunc = HashFunc;
	Set->CompareFunc = CompareFunc;
	Set->ItemCount = 0;

	AK_Sim__Set_Expand_Items(Set, 64);
	AK_Sim__Set_Expand_Slots(Set, 196);
}

static uint32_t AK_Sim__Set_Find_Slot(ak_sim__set* Set, const void* Key, uint32_t Hash) {
	if (Set->SlotCapacity == 0 || !Set->Slots) return AK_SIM__HASH_INVALID_SLOT;

	uint32_t SlotMask = Set->SlotCapacity - 1;
	uint32_t BaseSlot = (Hash & SlotMask);
	uint32_t BaseCount = Set->Slots[BaseSlot].BaseCount;
	uint32_t Slot = BaseSlot;

	while (BaseCount > 0) {
		if (Set->Slots[Slot].ItemIndex != AK_SIM__HASH_INVALID_SLOT) {
			uint32_t SlotHash = Set->Slots[Slot].Hash;
			uint32_t SlotBase = (SlotHash & SlotMask);
			if (SlotBase == BaseSlot) {
				AK_SIM_ASSERT(BaseCount > 0);
				BaseCount--;

				if (SlotHash == Hash) {
					void* KeyComp = Set->Keys + Set->Slots[Slot].ItemIndex * Set->KeySize;
					if (Set->CompareFunc(Key, KeyComp)) {
						return Slot;
					}
				}
			}
		}

		Slot = (Slot + 1) & SlotMask;
	}

	return AK_SIM__HASH_INVALID_SLOT;
}

static void AK_Sim__Set_Add_By_Hash(ak_sim__set* Set, const void* Key, uint32_t Hash) {
	AK_SIM_ASSERT(AK_Sim__Set_Find_Slot(Set, Key, Hash) == AK_SIM__HASH_INVALID_SLOT);

	if (Set->ItemCount >= (Set->SlotCapacity - (Set->SlotCapacity / 3))) {
		uint32_t NewSlotCapacity = Set->SlotCapacity ? Set->SlotCapacity*2 : 128;
		AK_Sim__Set_Expand_Slots(Set, NewSlotCapacity);
	}

	uint32_t SlotMask = Set->SlotCapacity - 1;
	uint32_t BaseSlot = (Hash & SlotMask);

	/*Linear probing. This is not great for preventing collision*/
    uint32_t BaseCount = Set->Slots[BaseSlot].BaseCount;
    uint32_t Slot = BaseSlot;
    uint32_t FirstFree = Slot;
    while (BaseCount) 
    {
        if (Set->Slots[Slot].ItemIndex == AK_SIM__HASH_INVALID_SLOT && Set->Slots[FirstFree].ItemIndex != AK_SIM__HASH_INVALID_SLOT) FirstFree = Slot;
        uint32_t SlotHash = Set->Slots[Slot].Hash;
        uint32_t SlotBase = (SlotHash & SlotMask);
        if (SlotBase == BaseSlot) 
            --BaseCount;
        Slot = (Slot + 1) & SlotMask;
    }
    
    Slot = FirstFree;
	while (Set->Slots[Slot].ItemIndex != AK_SIM__HASH_INVALID_SLOT) {
		Slot = (Slot + 1) & SlotMask;
	}

	uint32_t Index = Set->ItemCount++;
	if (Index >= Set->ItemCapacity) {
		AK_Sim__Set_Expand_Items(Set, Set->ItemCapacity*2);
	}

	AK_SIM_ASSERT(Set->ItemCount <= Set->ItemCapacity);

	Set->Slots[Slot].Hash = Hash;
	Set->Slots[Slot].ItemIndex = Index;
	Set->Slots[BaseSlot].BaseCount++;

	size_t KeyByteAt = Index * Set->KeySize;
	AK_SIM_MEMCPY(Set->Keys + KeyByteAt, Key, Set->KeySize);
	Set->ItemSlots[Index] = Slot;
}

static void AK_Sim__Set_Add(ak_sim__set* Set, const void* Key) {
	uint32_t Hash = Set->HashFunc(Key);
	AK_Sim__Set_Add_By_Hash(Set, Key, Hash);
}

static int AK_Sim__Set_Find_By_Hash(ak_sim__set* Set, const void* Key, uint32_t Hash) {
	uint32_t Slot = AK_Sim__Set_Find_Slot(Set, Key, Hash);
	return Slot != AK_SIM__HASH_INVALID_SLOT;
}

static int AK_Sim__Set_Find(ak_sim__set* Set, const void* Key) {
	uint32_t Hash = Set->HashFunc(Key);
	int Result = AK_Sim__Set_Find_By_Hash(Set, Key, Hash);
	return Result;
}

typedef struct {
    union {
        uint64_t ID;
        struct {
            uint32_t Generation;
            uint32_t Index;
        } Internal;
    };
} ak_sim__pool_id;

#define AK_SIM__POOL_FREE_INDEX ((uint32_t)-1)
#define AK_Sim__Pool_Item_Size(pool) ((pool)->ItemSize+sizeof(ak_sim__pool_id))
#define AK_Sim__Pool_Get_ID(pool, index) ((ak_sim__pool_id*)((pool)->Data + AK_Sim__Pool_Item_Size(pool)*(index)))

typedef struct {
	ak_sim_allocator* Allocator;
	uint8_t* 	      Data;
	size_t   	      ItemSize;
	uint32_t 	      FirstFreeIndex;
	uint32_t 	      ItemCapacity;
	uint32_t   	      ItemCount;
	uint32_t 	      MaxUsed;
} ak_sim__pool;

static void AK_Sim__Pool_Init_With_Size(ak_sim__pool* Pool, ak_sim_allocator* Allocator, uint32_t InitialCapacity, size_t ItemSize) {
    AK_SIM_MEMSET(Pool, 0, sizeof(ak_sim__pool));
	Pool->Allocator    = Allocator;
	Pool->ItemCapacity = InitialCapacity;
	Pool->ItemSize     = ItemSize;
	Pool->ItemCount    = 0;
	Pool->MaxUsed      = 0;
	Pool->Data = (uint8_t*)AK_Sim__Allocate_Memory(Allocator, AK_Sim__Pool_Item_Size(Pool) * InitialCapacity);

    uint32_t i;
	for (i = 0; i < InitialCapacity; i++) {
		ak_sim__pool_id* ID = AK_Sim__Pool_Get_ID(Pool, i);
		ID->Internal.Index = AK_SIM__POOL_FREE_INDEX;
		ID->Internal.Generation = 1;
	}
	Pool->FirstFreeIndex = AK_SIM__POOL_FREE_INDEX;
}

static void AK_Sim__Pool_Delete(ak_sim__pool* Pool) {
    ak_sim_allocator* Allocator = Pool->Allocator;
	AK_Sim__Free_Memory(Allocator, Pool->Data);
	AK_SIM_MEMSET(Pool, 0, sizeof(ak_sim__pool));
}

static uint64_t AK_Sim__Pool_Allocate(ak_sim__pool* Pool) {
	uint32_t Index = 0;
	if (Pool->FirstFreeIndex != AK_SIM__POOL_FREE_INDEX) {
		Index = Pool->FirstFreeIndex;
		Pool->FirstFreeIndex = AK_Sim__Pool_Get_ID(Pool, Index)->Internal.Index;
	} else {
		Index = Pool->MaxUsed++;
		if (Index >= Pool->ItemCapacity) {
			uint32_t NewCapacity = Pool->ItemCapacity * 2;
			uint8_t* NewData = (uint8_t*)AK_Sim__Allocate_Memory(Pool->Allocator, AK_Sim__Pool_Item_Size(Pool)*NewCapacity);
			AK_SIM_MEMCPY(NewData, Pool->Data, AK_Sim__Pool_Item_Size(Pool)*Pool->ItemCapacity);
			AK_Sim__Free_Memory(Pool->Allocator, Pool->Data);
			Pool->Data = NewData;
			Pool->ItemCapacity = NewCapacity;
		}
	}

	ak_sim__pool_id* ID = AK_Sim__Pool_Get_ID(Pool, Index);
	ID->Internal.Index = Index;
	Pool->ItemCount++;
	return ID->ID;
}

static uint8_t* AK_Sim__Pool_Get(ak_sim__pool* Pool, uint64_t IDValue) {
    ak_sim__pool_id ID;
    ID.ID = IDValue;

	ak_sim__pool_id* PoolID = AK_Sim__Pool_Get_ID(Pool, ID.Internal.Index);
	if (PoolID->Internal.Generation == ID.Internal.Generation) {
		return (uint8_t*)(PoolID + 1);
	}
	return NULL;
}

static void AK_Sim__Pool_Free(ak_sim__pool* Pool, ak_sim__pool_id ID) {
	ak_sim__pool_id* PoolID = AK_Sim__Pool_Get_ID(Pool, ID.Internal.Index);
	if (PoolID->ID == ID.ID) {
		PoolID->Internal.Generation++;
		if (PoolID->Internal.Generation == 0) PoolID->Internal.Generation = 1;
		PoolID->Internal.Index = Pool->FirstFreeIndex;
		Pool->FirstFreeIndex = ID.Internal.Index;
		Pool->ItemCount--;
	}
}

typedef struct {
    ak_sim__pool* Pool;
    uint32_t Index;
} ak_sim__pool_iter;

static ak_sim__pool_iter AK_Sim__Pool_Begin_Iter(ak_sim__pool* Pool) {
    ak_sim__pool_iter Result;
    Result.Pool = Pool;
    Result.Index = AK_SIM__POOL_FREE_INDEX;
    
    uint32_t i;
    for(i = 0; i < Pool->MaxUsed; i++) {
        ak_sim__pool_id* ID = AK_Sim__Pool_Get_ID(Pool, i);
        if(ID->Internal.Index == i) {
            Result.Index = i;
        }
    }

    return Result;
}

static int AK_Sim__Pool_Iter_Is_Valid(ak_sim__pool_iter* Iter) {
    return Iter->Index != AK_SIM__POOL_FREE_INDEX;
}

static uint8_t* AK_Sim__Pool_Iter_Next(ak_sim__pool_iter* Iter) {
    uint32_t i = Iter->Index+1;
    ak_sim__pool_id* PoolID = AK_Sim__Pool_Get_ID(Iter->Pool, Iter->Index);
    AK_SIM_ASSERT(PoolID->Internal.Index == Iter->Index);
    uint8_t* Result = (uint8_t*)(PoolID+1);
    Iter->Index = AK_SIM__POOL_FREE_INDEX;

    for(; i < Iter->Pool->MaxUsed; i++) {
        PoolID = AK_Sim__Pool_Get_ID(Iter->Pool, i);
        if(PoolID->Internal.Index == i) {
            Iter->Index = i;
        }
    }

    return Result;
}

typedef struct {
    uint32_t MaxPerRow;
    ak_sim_collision_func** CollisionFuncs;
} ak_sim__collision_table;

static ak_sim_collision_func* AK_Sim__Collision_Table_Get_Func(ak_sim__collision_table* Table, ak_sim_shape_type TypeA, ak_sim_shape_type TypeB) {
    uint32_t Index = TypeA*Table->MaxPerRow + TypeB;
    AK_SIM_ASSERT(Index < Table->MaxPerRow*Table->MaxPerRow);
    ak_sim_collision_func* Func = Table->CollisionFuncs[Index];
    return Func;
}

struct ak_sim_context {
    ak_sim_allocator Allocator;
    ak_sim__arena Arena;
    ak_sim__arena TempArena;
    ak_sim__collision_table CollisionTable;
    ak_sim__pool BodyPool;
};

typedef struct {
    ak_sim_body_id AID;
    ak_sim_body_id BID;
} ak_sim__body_id_pair;

AKSIMDEF ak_sim_v3 AK_Sim_V3(float x, float y, float z) {
    ak_sim_v3 Result;
    Result.Data[0] = x;
    Result.Data[1] = y;
    Result.Data[2] = z;
    return Result;
}

static void AK_Sim__Register_Collision(ak_sim__collision_table* Table, ak_sim_shape_type TypeA, ak_sim_shape_type TypeB, ak_sim_collision_func* CollisionFunc) {
    uint32_t Index = TypeA*Table->MaxPerRow + TypeB;
    AK_SIM_ASSERT(Index < Table->MaxPerRow*Table->MaxPerRow);
    Table->CollisionFuncs[Index] = CollisionFunc;
}

static void AK_Sim__Convex_Collision(ak_sim_collision_collector* Collector, 
                                     ak_sim_shape* ShapeA, const ak_sim_m4x3* TransformA, ak_sim_v3 ScaleA,
                                     ak_sim_shape* ShapeB, const ak_sim_m4x3* TransformB, ak_sim_v3 ScaleB) {
}

static void AK_Sim__Convex_Mesh_Collision(ak_sim_collision_collector* Collector, 
                                          ak_sim_shape* ShapeA, const ak_sim_m4x3* TransformA, ak_sim_v3 ScaleA,
                                          ak_sim_shape* ShapeB, const ak_sim_m4x3* TransformB, ak_sim_v3 ScaleB) {
}

static void AK_Sim__Convex_Compound_Collision(ak_sim_collision_collector* Collector, 
                                              ak_sim_shape* ShapeA, const ak_sim_m4x3* TransformA, ak_sim_v3 ScaleA,
                                              ak_sim_shape* ShapeB, const ak_sim_m4x3* TransformB, ak_sim_v3 ScaleB) {
}

static void AK_Sim__Mesh_Convex_Collision(ak_sim_collision_collector* Collector, 
                                          ak_sim_shape* ShapeA, const ak_sim_m4x3* TransformA, ak_sim_v3 ScaleA,
                                          ak_sim_shape* ShapeB, const ak_sim_m4x3* TransformB, ak_sim_v3 ScaleB) {
}

static void AK_Sim__Mesh_Collision(ak_sim_collision_collector* Collector, 
                                   ak_sim_shape* ShapeA, const ak_sim_m4x3* TransformA, ak_sim_v3 ScaleA,
                                   ak_sim_shape* ShapeB, const ak_sim_m4x3* TransformB, ak_sim_v3 ScaleB) {
}

static void AK_Sim__Mesh_Compound_Collision(ak_sim_collision_collector* Collector, 
                                            ak_sim_shape* ShapeA, const ak_sim_m4x3* TransformA, ak_sim_v3 ScaleA,
                                            ak_sim_shape* ShapeB, const ak_sim_m4x3* TransformB, ak_sim_v3 ScaleB) {
}

static void AK_Sim__Compound_Convex_Collision(ak_sim_collision_collector* Collector, 
                                              ak_sim_shape* ShapeA, const ak_sim_m4x3* TransformA, ak_sim_v3 ScaleA,
                                              ak_sim_shape* ShapeB, const ak_sim_m4x3* TransformB, ak_sim_v3 ScaleB) {
}

static void AK_Sim__Compound_Mesh_Collision(ak_sim_collision_collector* Collector, 
                                            ak_sim_shape* ShapeA, const ak_sim_m4x3* TransformA, ak_sim_v3 ScaleA,
                                            ak_sim_shape* ShapeB, const ak_sim_m4x3* TransformB, ak_sim_v3 ScaleB) {
}

static void AK_Sim__Compound_Collision(ak_sim_collision_collector* Collector, 
                                       ak_sim_shape* ShapeA, const ak_sim_m4x3* TransformA, ak_sim_v3 ScaleA,
                                       ak_sim_shape* ShapeB, const ak_sim_m4x3* TransformB, ak_sim_v3 ScaleB) {
}

static ak_sim_collision_func* G_CollisionFunc[AK_SIM_SHAPE_TYPE_COUNT][AK_SIM_SHAPE_TYPE_COUNT] = {
    {AK_Sim__Convex_Collision, AK_Sim__Convex_Mesh_Collision, AK_Sim__Convex_Compound_Collision},
    {AK_Sim__Mesh_Convex_Collision, AK_Sim__Mesh_Collision, AK_Sim__Mesh_Compound_Collision},
    {AK_Sim__Compound_Convex_Collision, AK_Sim__Compound_Mesh_Collision, AK_Sim__Compound_Collision}
};

AKSIMDEF ak_sim_context* AK_Sim_Create_Context(const ak_sim_create_info* CreateInfo) {
    ak_sim_allocator Allocator = CreateInfo->Allocator;
    if(!Allocator.AllocateMemory || !Allocator.FreeMemory) {
#ifdef AK_SIM_NO_STDLIB
        return NULL;
#else
        Allocator = AK_Sim__Get_Stdio_Allocator();
#endif
    }

    ak_sim_context* Result = AK_Sim__Allocate_Struct(&Allocator, ak_sim_context);
    Result->Allocator = Allocator;
    AK_Sim__Arena_Create(&Result->Arena, &Result->Allocator);
    AK_Sim__Arena_Create(&Result->TempArena, &Result->Allocator);

    ak_sim__collision_table* CollisionTable = &Result->CollisionTable;
    uint32_t MaxPerRow = AK_SIM_SHAPE_TYPE_COUNT;

    uint32_t i;
    for(i = 0; i < CreateInfo->ShapeRegistrationCount; i++) {
        ak_sim_shape_registration* Registration = CreateInfo->ShapeRegistrations+i;
        AK_SIM_ASSERT(Registration->Type < AK_SIM_SHAPE_TYPE_COUNT);
        MaxPerRow = AK_Sim__Max(Registration->Type, MaxPerRow);
    }

    CollisionTable->MaxPerRow = MaxPerRow;
    CollisionTable->CollisionFuncs = AK_Sim__Arena_Push_Array(&Result->Arena, MaxPerRow*MaxPerRow, ak_sim_collision_func*);
    AK_SIM_MEMSET(CollisionTable->CollisionFuncs, 0, MaxPerRow*MaxPerRow*sizeof(ak_sim_collision_func*));

    /*First register default shape types*/
    for(i = 0; i < AK_SIM_SHAPE_TYPE_COUNT; i++) {
        uint32_t j;
        for(j = 0; j < AK_SIM_SHAPE_TYPE_COUNT; j++) {
            AK_Sim__Register_Collision(CollisionTable, i, j, G_CollisionFunc[i][j]);
        }
    }

    /* Then register custom shapes */
    for(i = 0; i < CreateInfo->ShapeRegistrationCount; i++) {
        ak_sim_shape_registration* Registration = CreateInfo->ShapeRegistrations+i;
        uint32_t j;
        for(j = 0; j < Registration->CollisionFuncCount; j++) {
            ak_sim_collision_registration* Collision = Registration->Collisions + j;
            AK_Sim__Register_Collision(CollisionTable, Registration->Type, Collision->Type, Collision->CollisionFunc);
        }
    }

    AK_Sim__Pool_Init_With_Size(&Result->BodyPool, &Result->Allocator, 512, sizeof(ak_sim_body));


    return Result;
}

AKSIMDEF void AK_Sim_Delete_Context(ak_sim_context* Context) {
    if(Context) {
        ak_sim_allocator* Allocator = &Context->Allocator;
        AK_Sim__Pool_Delete(&Context->BodyPool);
        AK_Sim__Arena_Delete(&Context->TempArena);
        AK_Sim__Arena_Delete(&Context->Arena);
        AK_Sim__Free_Memory(Allocator, Context);
    }
}

static uint32_t AK_Sim__Hash_U64(uint64_t x) {
    x ^= x >> 32;
	x *= 0xd6e8feb86659fd93;
	x ^= x >> 32;
	x *= 0xd6e8feb86659fd93;
	x ^= x >> 32;
	return (uint32_t)x;
}

static uint32_t AK_Sim__Body_Pair_Hash(const void* Key) {
    const ak_sim__body_id_pair* Pair = (const ak_sim__body_id_pair*)Key;
    return AK_Sim__Hash_U64((uint64_t)Pair->AID | ((uint64_t)Pair->BID << 32));
}

static int AK_Sim__Body_Pair_Compare(const void* KeyA, const void* KeyB) {
    const ak_sim__body_id_pair* A = (const ak_sim__body_id_pair*)KeyA;
    const ak_sim__body_id_pair* B = (const ak_sim__body_id_pair*)KeyB;
    return A->AID == B->AID && A->BID == B->BID;
}

typedef struct {
    ak_sim__set Set;
} ak_sim__body_id_pair_set;

struct ak_sim_collision_collector {
    ak_sim__arena* Arena;
};

static ak_sim_collision_collector AK_Sim__Begin_Collision_Collector(ak_sim__arena* Arena) {
    ak_sim_collision_collector Result;
    AK_SIM_MEMSET(&Result, 0, sizeof(ak_sim_collision_collector));
    Result.Arena = Arena;
    return Result;
}

static void AK_Sim__Update_Internal(ak_sim_context* Context, float DeltaTime, ak_sim__temp_arena* TempStorage) {
    ak_sim__arena* TempArena = TempStorage->Arena;
    
    /*todo: Better broadphase*/
    ak_sim__array PairArray;
    ak_sim__set PairSet;
    AK_Sim__Array_Init(&PairArray, &TempArena->BaseAllocator, sizeof(ak_sim__body_id_pair));
    AK_Sim__Set_Init(&PairSet, &TempArena->BaseAllocator, sizeof(ak_sim__body_id_pair), AK_Sim__Body_Pair_Hash, AK_Sim__Body_Pair_Compare);

    ak_sim__pool* BodyPool = &Context->BodyPool;
    ak_sim__pool_iter BodyIterA = AK_Sim__Pool_Begin_Iter(BodyPool);
    while(AK_Sim__Pool_Iter_Is_Valid(&BodyIterA)) {
        ak_sim_body* BodyA = (ak_sim_body*)AK_Sim__Pool_Iter_Next(&BodyIterA);
        ak_sim__pool_iter BodyIterB = AK_Sim__Pool_Begin_Iter(BodyPool);
        while(AK_Sim__Pool_Iter_Is_Valid(&BodyIterB)) {
            ak_sim_body* BodyB = (ak_sim_body*)AK_Sim__Pool_Iter_Next(&BodyIterB);
            if(BodyA != BodyB) {
                ak_sim__body_id_pair Pair;
                Pair.AID = AK_Sim__Min(BodyA->ID, BodyB->ID);
                Pair.BID = AK_Sim__Max(BodyA->ID, BodyB->ID);

                if(!AK_Sim__Set_Find(&PairSet, &Pair)) {
                    AK_Sim__Array_Add(&PairArray, &Pair);
                    AK_Sim__Set_Add(&PairSet, &Pair);
                }
            }
        }
    }

    ak_sim__collision_table* CollisionTable = &Context->CollisionTable;
    ak_sim_collision_collector CollisionCollector = AK_Sim__Begin_Collision_Collector(TempArena);

    uint32_t PairIndex;
    for(PairIndex = 0; PairIndex < PairArray.Count; PairIndex++) {
        ak_sim__body_id_pair* Pair = (ak_sim__body_id_pair*)AK_Sim__Array_Get(&PairArray, PairIndex);
        
        ak_sim_body* BodyA = (ak_sim_body*)AK_Sim__Pool_Get(BodyPool, Pair->AID);
        ak_sim_body* BodyB = (ak_sim_body*)AK_Sim__Pool_Get(BodyPool, Pair->BID);

        ak_sim_collision_func* CollisionFunc = AK_Sim__Collision_Table_Get_Func(CollisionTable, BodyA->Shape.Type, BodyB->Shape.Type);
        if(CollisionFunc) {
            ak_sim_m4x3 TransformA = AK_Sim__Get_Matrix_Transform(&BodyA->Transform);
            ak_sim_m4x3 TransformB = AK_Sim__Get_Matrix_Transform(&BodyB->Transform);

            CollisionFunc(&CollisionCollector, &BodyA->Shape, &TransformA, BodyA->Scale, &BodyB->Shape, &TransformB, BodyB->Scale);
        }
    }
}

AKSIMDEF void AK_Sim_Update(ak_sim_context* Context, float DeltaTime) {
    ak_sim__temp_arena TempArena = AK_Sim__Arena_Begin_Temp(&Context->TempArena);
    AK_Sim__Update_Internal(Context, DeltaTime, &TempArena);
    AK_Sim__Arena_End_Temp(&TempArena);
}

#endif