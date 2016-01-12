/*
copy function
*/
#include "main.h"
#include <string.h>
//#include "os_thread_helper_function.h"

void string_copy(char* destination,char* source){
  while(*source!='\0'){
    *destination++=*source++;
  }*destination='\0';
}

void* copy_osThreadDef_UD_t(threadDef* destination, osThreadDef_t* source){
 //threadDef* destination = (threadDef*)dest;
 //struct os_thread_def* source = (struct os_thread_def*)s;
  //TODO - use dynamic check for string length
 if((destination->name = (char*) pvPortMalloc(25*sizeof(char)))==NULL)
   while(1);    //TODO - fix error handler
 
 string_copy((destination->name),(source->name));
 destination->pthread   = source->pthread;
 destination->tpriority = source->tpriority;
 destination->instances = source->instances;
 destination->stacksize = source->stacksize;
 
 return destination;
}