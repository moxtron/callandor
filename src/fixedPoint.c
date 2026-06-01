#include <stdint.h>
#include "comp.h"
typedef int32_t q23_8_t;

// returns a q23_8_t interpreted number represented by an int32 in memory
q23_8_t make_q(int32_t integer_part, int32_t fractial_part){
    if(integer_part < 0){
        // converting it to a positve value because if fractional_part
        // is represented like 0b1100 0000 192 (base 10) which represents
        // 2^-1 +2^-2 = 0.75 when integer part is negative for instance
        // -45 then | 0.75 = -44.25 but should be -45.75 so its first converted
        // to a positve value and then returned as twos complement
        q23_8_t positive = ((-integer_part) << 8) | fractial_part;
        return -positive;
    }
    else{
        // shift integer and fractional part together
        return (integer_part << 8) | fractial_part;
    }    
    
}
// prints a q23_8_t number in decimal format
void print_q(q23_8_t x){
    if(x < 0){
        // print minus sign if number is negative
        print("-");
        x = -x;   
    }
    // get integer part by shifting right and dropping 8 fractional bits
    int32_t integer_part = x >> 8;
    // get fractional bits by masking first 8 bits
    int32_t fractial_part = x & 0xff;
    // get fractional bits represented from 0-1000
    int32_t fractial_dec = (fractial_part * 1000) >> 8;
    printNum(integer_part);
    print(".");
    printNum(fractial_dec);
    print("\n");
}
// subtract two q23_8_t numbers (minuend - subtrahend)
q23_8_t subtractQ23_8(q23_8_t minuend, q23_8_t subtrahend){
    return minuend - subtrahend;
}
// add two q23_8_t numbers (addend + addend1)
q23_8_t addQ23_8(q23_8_t addend, q23_8_t added1){
    return addend + added1;
}
// multiply two q23_8_t numbers (multiplicand * multiplier)
q23_8_t multiplyQ23_8(q23_8_t multiplicand, q23_8_t multiplier){
    return (int64_t) multiplicand * multiplier >> 8;
}
// divide two q23_8_t numbers (dividend / divisor)
q23_8_t divide(q23_8_t dividend, q23_8_t divisor){
    return ((int64_t) dividend << 8) / divisor;
}