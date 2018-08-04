#include "debug.h"

#ifdef __DEBUG__

/**
 * Helper to print doubles
 */
char *ftoa(char *a, double f)
{
    char *ret = a;

    if (f < 0)
    {
       *a++ = '-';
    }

    f = fabs(f);

    long heiltal = (long)f;
    if ( ! itoa(heiltal, a, 10))
    {
       *a = 'X';
    }
    while (*a != '\0') a++;
    *a++ = '.';

    sprintf(a, "%04d", abs((long)((f - (double)heiltal) * 10000.0)));
    return ret;
}

#endif
