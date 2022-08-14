/*-----------------------------------------------------------
* Example console I/O wrappers.
*----------------------------------------------------------*/

#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"

SemaphoreHandle_t xStdioMutex;
StaticSemaphore_t xStdioMutexBuffer;

void console_init( void )
{
    xStdioMutex = xSemaphoreCreateMutexStatic( &xStdioMutexBuffer );
}

void console_print( const char * fmt,
                    ... )
{
    va_list vargs;

    va_start( vargs, fmt );

    xSemaphoreTake( xStdioMutex, portMAX_DELAY );

    vprintf( fmt, vargs );

    xSemaphoreGive( xStdioMutex );

    va_end( vargs );
}