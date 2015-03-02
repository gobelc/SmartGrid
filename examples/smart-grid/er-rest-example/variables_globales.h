

/*------------------------TIPOS DE DATOS----------------------*/

//Estructura que contiene los valores definitivos (calculos finales) de las variables medidas
typedef struct Medida {
	float Vrms;
	float Irms;
	float q;
	float p;
	float s;
	float fp;
} Medida;

/*------------------------------------------------------------*/

#ifndef SHAREFILE_INCLUIDO
#define SHAREFILE_INCLUIDO
#ifdef  ARCHIVO_PRINCIPAL

/*---------------Declaraciones----------------------*/
struct Medida datos; //Calculo de los valores finalesde las mediciones

uint8_t status_relays; /*(0x00: r0 apagado; r1 apagado
								0x0F: r0 encendido; r1 apagado
								0xF0: r0 apagado; r1 encendido
								0xFF: r0 encendido; r1 encendido)*/

uint8_t comando_relays; /*(0x00: r0 apagado; r1 apagado
								0x0F: r0 encendido; r1 apagado
								0xF0: r0 apagado; r1 encendido
								0xFF: r0 encendido; r1 encendido)*/

#else

/*---------------Externs----------------------------*/
extern struct Medida datos; //Calculo los valores finales y los guardo en la estructura

extern uint8_t status_relays; /*(0x00: r0 apagado; r1 apagado
								0x0F: r0 encendido; r1 apagado
								0xF0: r0 apagado; r1 encendido
								0xFF: r0 encendido; r1 encendido)*/

extern uint8_t comando_relays; /*(0x00: r0 apagado; r1 apagado
								0x0F: r0 encendido; r1 apagado
								0xF0: r0 apagado; r1 encendido
								0xFF: r0 encendido; r1 encendido)*/

#endif
#endif




