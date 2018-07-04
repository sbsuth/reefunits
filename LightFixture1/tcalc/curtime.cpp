#include <time.h>
#include <stdio.h>

int main()
{
    time_t t1 = time(0);
    struct tm* t1_rec = localtime(&t1);
    time_t t2 = mktime(t1_rec);
    printf("%d\n", t2 + t1_rec->tm_gmtoff);
}
