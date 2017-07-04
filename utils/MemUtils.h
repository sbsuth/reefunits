#ifndef MEM_UTILS
#define MEM_UTILS

struct MemChecker 
{
    static uint8_t* heapptr;
    static uint8_t* stackptr;

    static void reset() {
        heapptr = 0;
        stackptr = 0;
    }

    static void log_mem() {
        stackptr = (uint8_t *)malloc(4);         
        if (!stackptr || (stackptr > heapptr))
            heapptr = stackptr;                   
        free(stackptr);      
        if ( !heapptr || ((uint8_t *)(SP) > stackptr))
            stackptr = (uint8_t*)SP;
    }

    static int free_ram() 
    {
        extern int __heap_start, *__brkval; 
        int v; 
        return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
    }

    static int gap() {
        return (stackptr - heapptr);
    }
};

// Hack.  Don't want to add a source file, so include this from only one place,
// or define NO_MEM_CHECKER_DATA before including.
#ifndef NO_MEM_CHECKER_DATA

uint8_t* MemChecker::heapptr;
uint8_t* MemChecker::stackptr;
#endif

#endif
