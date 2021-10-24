#ifndef AT24C512_H
#define AT24C512_H

#ifdef __cplusplus
extern "C" { 
#endif

/* Declare variable */
extern idata char temp;

/* Declare function */
extern char read(unsigned short idata index);
extern void write(unsigned short idata index, char idata input);
#ifdef __cplusplus
}
#endif
#endif