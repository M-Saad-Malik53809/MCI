#include "heap_driver.h"
#include <stdint.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

// Block map: 0 = free, 1 = used
static uint8_t block_map[BLOCK_COUNT];

// Helper macro to get address of block
#define BLOCK_ADDR(index) (HEAP_START_ADDR + (index * BLOCK_SIZE))
void heap_init(void)
{
    for (int i = 0; i < BLOCK_COUNT; i++) {
        block_map[i] = 0;  // mark all blocks free
    }
}
void* heap_alloc(size_t size)
{
    if (size == 0) return NULL;

    // Calculate required blocks (ceil division)
    size_t blocks_needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;

    size_t consecutive = 0;
    size_t start_index = 0;

    for (size_t i = 0; i < BLOCK_COUNT; i++) {

        if (block_map[i] == 0) {
            // free block
            if (consecutive == 0) {
                start_index = i;
            }
            consecutive++;

            if (consecutive == blocks_needed) {
                // Mark blocks as used
                for (size_t j = start_index; j < start_index + blocks_needed; j++) {
                    block_map[j] = 1;
                }

                // Return pointer to start address
                return (void*)BLOCK_ADDR(start_index);
            }
        } else {
            // reset if occupied
            consecutive = 0;
        }
    }

    // No sufficient space
    return NULL;
}
void heap_free(void* ptr)
{
    if (ptr == NULL) return;

    uint8_t *addr = (uint8_t *)ptr;
    size_t index = (addr - HEAP_START_ADDR) / BLOCK_SIZE;

    // Boundary check
    if (addr < HEAP_START_ADDR || addr >= (HEAP_START_ADDR + HEAP_SIZE)) {
        return; // invalid pointer
    }

    // Check alignment
    if ((addr - HEAP_START_ADDR) % BLOCK_SIZE != 0) {
        return; // not aligned
    }

    // Get starting block index

    // Free blocks until a free block is encountered
    for (size_t i = index; i < BLOCK_COUNT; i++) {
        if (block_map[i] == 1) {
            block_map[i] = 0;
        } else {
            break; // stop at first free block
        }
    }
}