#include <stdio.h>

unsigned char* charOutput;

void Float2Char(float* floatValue) {
    charOutput = (unsigned char*)&floatValue;
    printf("%x\n", charOutput);
    for (int i=0; i<sizeof(floatValue); i++) {
        printf("%x ", (unsigned char)(*charOutput++));
    }
}

void Short2Char(short* shortValue, char* firstCharAddr) {
    short* result = (*shortValue * 350)/1000;
    printf("%d\n", sizeof(result));
    for (int i=0; i<sizeof(result); i++) {
        firstCharAddr[i] = (char)(*result++);
        printf("%X ", firstCharAddr[i]);
    }
    printf("\n");
}

int main(void) {
    float floatValue = 0.001;
    

    charOutput = (unsigned char*)&floatValue;

    printf("Float in Hex: %f\n", floatValue);
    for (int i=0; i<sizeof(floatValue); i++) {
        printf("%x ", (unsigned char)(*charOutput++));
    }

    printf("\n");
    printf("%x\n", charOutput);
    Float2Char(&floatValue);
    printf("\n");

    /*************/
    short shortValue = 12345;
    char charArr[2];
    Short2Char(&shortValue, charArr);

    getchar();
    return 0;
}