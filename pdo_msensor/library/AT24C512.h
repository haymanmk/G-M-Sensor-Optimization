#ifndef AT24C512_H
#define AT24C512_H

#ifdef __cplusplus
extern "C" { 
#endif

/* Declare variable */
extern idata char temp;

/* Declare function */
extern char read(unsigned short int index);
extern void write(unsigned short int index, char input);
#ifdef __cplusplus
}
#endif
#endif