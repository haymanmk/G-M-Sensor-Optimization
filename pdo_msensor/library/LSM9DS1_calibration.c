#include "LSM9DS1_calibration.h"
#include "AT24C512.h"
#include "Ext_function.h"

// void write_gain_and_offset_matrix(unsigned char row, unsigned char col, float num){    
// 	char i; //Loop counter

// 	CHAR_TO_FLOAT char_to_float;
// 	char_to_float._float = num;
		
// 	for(i=0;i<4;i++){
// 	   write((unsigned short int)(12*row+4*col+i), char_to_float._char[i]);
//     }
// }

// float read_gain_and_offset_matrix(unsigned char row, unsigned char col){
// 	char i; //Loop counter
// 	CHAR_TO_FLOAT char_to_float;
    
//     for(i=0;i<4;i++){
// 		char_to_float._char[i] = read((unsigned short int)(12*row+4*col+i)); //[0]LSB -> [3]MSB
// 	}

// 	return char_to_float._float;
// }

// int power_of_10(char exp){
//     int result = 10;
//     char i = exp;

//     if(i<0){
//         return 0;
//     }
//     else if(i == 0){
//         return 1;
//     }

//     for(i=exp;i>1;i--){
//         result *= 10;
//     }
//     return result;
// }

// float byte_to_float(char *bytes, char index_start, char index_end){
//     char i = 0;
//     char sign_semicolon = -1;
//     char sign_dot = -1;
//     char flag_sign = 1;
//     float result = 0.0;
//     //EW32=120.123;

//     //Check sign
//     if(*(bytes+index_start) == '-'){
//         flag_sign = -1;
//         index_start++;
//     }
//     else if(*(bytes+index_start) == '+'){
//         flag_sign = 1;
//         index_start++;
//     }

//     //find semicolon
//     for(i=index_start;i<=index_end;i++){
//         if(*(bytes+i) == ';'){
//             sign_semicolon = i;
//             sign_dot = (sign_dot == -1)?sign_semicolon:sign_dot;
//             break;
//         }
//         else if(*(bytes+i) == '.'){
//             sign_dot = i;
//         }
//         else if(*(bytes+i) < '0' || *(bytes+i) > '9'){
//             return -999.0;
//         } 
//     }

//     //Cannot find EOL sign(semicolon)
//     if(sign_semicolon == -1){
//         return -999.0;
//     }
//     else if(sign_semicolon-sign_dot == 1){
//         return -999.0;
//     }

//     //integers
//     for(i=index_start;i<sign_dot;i++){
//         result += (power_of_10(sign_dot-i-1)*(*(bytes+i)-'0'));
//     }

//     //decimal
//     for(i=(sign_dot+1);i<sign_semicolon;i++){
//         result += (1.0/power_of_10(i-sign_dot))*(*(bytes+i)-'0');
//     }
//     result *= flag_sign;
//     return result;
// }

// float byte_to_float(char *bytes, int SIZE){
//     char sign = 0;
//     char *ptr_header = bytes;
//     char *ptr_dot = bytes;	
//     float result = 0.0;
//     int i; //Loop counter
	
//     if(!(SIZE > 0)){
//         return -999.0;
//     }
    
//     //Check sign
//     if((*ptr_header>='0' && *ptr_header<='9')){
//         sign = 1;
//     }
//     else if((*ptr_header == '+') || (*ptr_header == '.')){
//         ptr_header = ptr_header+1;
//         sign = 1;
//     }
//     else if(*ptr_header == '-'){
//         ptr_header = ptr_header+1;
//         sign = -1;
//     }
//     else{
//         return -999.0;
//     }
    
//     ptr_dot = ptr_dot+SIZE; //point to last index+1

//     //Find dot
//     for(i=0; i<SIZE ;i++){
//         if(*(bytes+i)=='.'){
//             ptr_dot = (bytes+i);
//             break;
//         }
//     }
  
//     //generate numeric
//     while(ptr_header != (bytes+SIZE)){
//         if(ptr_header == ptr_dot){
//             ptr_header += 1;
//             continue;
//         }
//         if(*ptr_header > '9' || *ptr_header < '0'){
//             return -999.0;
//         }
//         result += (((*ptr_header)-'0')*pow(10.0, ((float)(ptr_dot-ptr_header)/sizeof(char))-((ptr_dot - ptr_header > 0)?1.0:0.0)));
//         ptr_header += 1;
//     }
//     result *= sign;
//     return result;
// }