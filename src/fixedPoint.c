#include <stdint.h>
#include "comp.h"
typedef int32_t q23_8_t;


q23_8_t make_q(int32_t integer_part, int32_t fractial_part){
    if(integer_part < 0){
        // converting it to a positve value because if fractional_part
        // is represented like 0b1100 0000 192 (base 10) which represents
        // 2^-1 +2^-2 = 0.75 when integer part is negative for instance
        // -45 then | 0.75 = -44.25 but should be -45.75 so its first converted
        // to a positve value and then returned as its twos complement
        q23_8_t positive = ((-integer_part) << 8) | fractial_part;
        return -positive;
    }
    else{
        return (integer_part << 8) | fractial_part;
    }    
    
}
// prints a number in q23_8_t format
void print_q(q23_8_t x){
    if(x < 0){
        print("-");
        x = -x;   
    }
    int32_t integer_part = x >> 8;
    int32_t fractial_part = x & 0xff;
    int32_t fractial_dec = (fractial_part * 1000) >> 8;
    printNum(integer_part);
    print(".");
    printNum(fractial_dec);
    print("\n");
}