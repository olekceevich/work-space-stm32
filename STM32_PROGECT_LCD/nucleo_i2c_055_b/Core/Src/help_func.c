#include "help_func.h"
#include <string.h>

void float_to_string(float num, char *str, int afterpoint)
{
    int ipart = (int)num;
    float fpart = num - (float)ipart;
    int i = 0;

    // Обработка целой части
    int_to_string(ipart, str);
    i = strlen(str);

    if (afterpoint != 0) {
        str[i] = '.';
        fpart = fpart * pow(10, afterpoint);
        int_to_string((int)fpart, str + i + 1);
    }
}

void int_to_string(int num, char *str)
{
    int i = 0;
    int is_negative = 0;

    if (num < 0) {
        is_negative = 1;
        num = -num;
    }

    do {
        str[i++] = num % 10 + '0';
        num = num / 10;
    } while (num > 0);

    if (is_negative) {
        str[i++] = '-';
    }

    str[i] = '\0';

    // Переворачиваем строку
    int start = 0;
    int end = i - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}
