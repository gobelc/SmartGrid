/*
 * Copyright (c) 2013, Institute for Pervasive Computing, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *      Erbium (Er) REST Engine example.
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 */

/*NODO DE MEDICION Y CARGA:
 *Se trata de un servidor CoAP que cumple dos funciones fundamentales:
 *1- Medición y reporte de datos hacia el controlador central
 *2- Control de la carga en base a instrucciónes recibidas de parte del controlador central
 *
 *La estructura de comunicación está hecha en base al ejemplo er-example-server.c. Utiliza 6LoWPAN ruteado con RPL
 *y CoAP a nivel de aplicación
 */

//#include <stdio.h>
//--------------Includes generales------------
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"
#include "rest-engine.h"
#include "sys/rtimer.h"

/*--------------------------------------------------------------------------------------------------*/
#define ARCHIVO_PRINCIPAL
#include "variables_globales.h"

/*--------------------------------------------------------------------------------------------------*/

//---------------Metering--------------------
#include "core/cfs/cfs.h"
#include <math.h>
#include "lib/sensors.h"
#include "dev/leds.h"
#include "dev/z1-phidgets.h"
#include "dev/relay-phidget.h"
#include "dev/button-sensor.h"

#define SENSOR_CORRIENTE_1 0
#define SENSOR_CORRIENTE_2 0
#define SENSOR_VOLTAJE 3
#define VREF5 5 //Tensión de referencia para sensores de 5V
#define VREF3 3 //Tensión de referencia para sensores de 3V
#define TAM_VENTANA 250 //Cantidad de muestras tomadas en un muestreo
#define OVERLAP_VENTANA 50 //Muestras solapadas para cálculo de potencia reactiva
#define PERIODO_MUESTREO 32 //Es el período de muestreo medido en ticks. Cada tick vale 1/32768 [seg] (RTIMER_SECOND=32768 -> 1seg)
#define MILISEG_POR_TICK 0.030518 //Cada tick equivale a 0.030518 msegs
#define TIEMPO_ENTRE_MEDIDAS 3 //Cada cuanto tiempo se realiza una medición.

#define OFFSET_MOTE_3V 0.0215 //Por experimento vimos que el mote mete un offset (mV)
#define OFFSET_MOTE_5V 0.054 //Por experimento vimos que el mote mete un offset (mV)
#define VDC_3V (1 - OFFSET_MOTE_3V)//Tensión de continua de referencia que manda la placa de preprocesamiento.0.0215
#define VDC_5V (1 - OFFSET_MOTE_5V)//Tensión de continua de referencia que manda la placa de preprocesamiento.0.0215

#define P6_DO_RELAY_0 2 //Salida del relay 0 (P6.x)
#define P6_DO_RELAY_1 4 //Salida del relay 1 (P6.x)


//-------------------------------------------

//Opción para debug estándar de Contiki
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]", (lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3], (lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif

//Opción para activar reporte de muestras adquiridas
#define DEBUG_MUESTRAS 0
#if DEBUG_MUESTRAS
#include <stdio.h>
#define PRINT_MUESTRAS(...) printf(__VA_ARGS__)
#define CON_MUESTRAS
#else
#define CON_MUESTRAS //
#define PRINT_MUESTRAS(...)
#endif

//Opción para activar interface con el usuario y reporte de medidas por serial
#define PRINT_STANDARD 0
#if (!DEBUG_MUESTRAS && PRINT_STANDARD)
#include <stdio.h>
#define PRINT_STD(...) printf(__VA_ARGS__)
#else
#define PRINT_STD(...)
#endif

/*
 * Resources to be activated need to be imported through the extern keyword.
 * The build system automatically compiles the resources in the corresponding sub-directory.
 */

extern resource_t res_mediciones_SG;
extern resource_t res_comando_SG;

PROCESS(comunicacion, "HAN Smart Grid node");
PROCESS(medicion, "Medicion de consumo");
PROCESS(control_carga, "Control de Carga");
AUTOSTART_PROCESSES(&comunicacion, &medicion, &control_carga);

/*---------------------DECLARACIÓN DE VARIABLES----------------------------*/

static double Gi1=1; //Ganancia V/A Amplificador n°1 corriente
static double Gi2=1; //Ganancia V/A Amplificador n°2 corriente
static double Gv=1; //Ganancia V/V Voltaje transductor vs Voltaje de Red

static double CurrentThreshold=2.3; //¿Qué es?
static double B[251]; //¿Qué es?

static float Vrms=0; //BORRAR?????????
static float Irms=0; //BORRAR?????????
static float p=0; //BORRAR?????????
static float q=0; //BORRAR?????????
static float s=0; //BORRAR?????????
static float fp=0; //BORRAR?????????


static int flag_corriente; //1 para Corriente 1 ; 0 para Corriente 2.

// Coeficientes del Filtro Pasabajos
static double coef_1=1/4;
static double coef_2=1/4;
static double coef_3=1/4;
static double coef_4=1/4;

//Buffers que contienen los valores muestreados
static uint16_t buff_C1[TAM_VENTANA];
static uint16_t buff_C2[TAM_VENTANA];
static uint16_t buff_Corr[TAM_VENTANA];
static uint16_t buff_V[TAM_VENTANA];
static uint16_t buff_Time_Stamp[TAM_VENTANA];

static int muestra; //Contiene el número de muesta actual durante el muestreo

static struct rtimer tmuestreo; //Timer de tiempo real que genera interrupciones para tomar las muestras (cuenta el período de muestreo)
static struct etimer periodico; //Timer utilizado para contar tiempos entre mediciones



/*---------------------FIN DECLARACIÓN DE VARIABLES----------------------------*/

/*------------------------------------------------------------------------------*/
/*----------------------------FUNCIONES MEDICIÓN--------------------------------*/
/*------------------------------------------------------------------------------*/

void shift(double a[], int n) {
 	 	     int i;
 	 	     for(i = 0; i!= n; i++){
 	 	        a[i] = a[i+1];
 	 	     }
 	 }

int signo(float valoraso){
 		if (valoraso>0)
 			return (int)1;
 		else {
 			return (int)0;
 		}
 	}

int desfasaje(int SENSOR1, int SENSOR2){

	int deltaN=0;

	float value1=phidgets.value(SENSOR1);
	float value2=phidgets.value(SENSOR2);

	int sign_1;
	sign_1=signo(value1);
	int sign_2;
	sign_2=signo(value2);

	if (sign_1==1)
		while (signo(value1)){
			value1=phidgets.value(SENSOR1);
			value2=phidgets.value(SENSOR2);
			sign_2=signo(value2);
		}
		if (sign_2==1)
			while (sign_2==1){
				value2=phidgets.value(SENSOR2);
				sign_2=signo(value2);
				deltaN++;
			}
		if (sign_2==0)
			while (sign_2==0){
				value2=phidgets.value(SENSOR2);
				sign_2=signo(value2);
			}
			while (sign_2==1){
				value2=phidgets.value(SENSOR2);
				sign_2=signo(value2);
				deltaN++;
			}

	if (sign_1==0)
			while (!signo(value1)){
				value1=phidgets.value(SENSOR1);
				value2=phidgets.value(SENSOR2);
				sign_2=signo(value2);
			}
			if (sign_2==0)
				while (sign_2==0){
					value2=phidgets.value(SENSOR2);
					sign_2=signo(value2);
					deltaN++;
				}
			if (sign_2==1)
				while (sign_2==1){
					value2=phidgets.value(SENSOR2);
					sign_2=signo(value2);
				}
				while (sign_2==0){
					value2=phidgets.value(SENSOR2);
					sign_2=signo(value2);
					deltaN++;
				}
return deltaN;
}


 double filtrado(x,coef_1,coef_2,coef_3,coef_4){
		// FIR de media movil
		double x_3;
		double x_2;
		double x_1;
		double y=x*coef_1+x_1*coef_2+x_2*coef_3+x_3*coef_4;
		x_3=x_2;
		x_2=x_1;
		x_1=y;
		return 2*y; // Ajusto ganancia del filtro, dado que modulo de h[n]=1/2.
 }

 /*FUNCIÓN muestreo: Al ser llamada esta función se encarga de tomar una tanda de muestras. Además, mientras no se
  * haya completado la totalidad de las muestras, la propia función agenda una próxima ejecución de ella misma
  * a través de un rtimer, un PERIODO_MUESTREO despues de la ejecución actual. De esta manera las muestras se toman
  * regularmente y a gran velocidad. Al llegar a la última muestra, la función le postea un evento poll al proceso
  * principal, y el mismo continúa.
  */

 static char muestreo(struct rtimer *rt, void* ptr){

	 if (muestra < TAM_VENTANA){

		 uint8_t ret;
		 ret = rtimer_set(&tmuestreo, RTIMER_NOW()+PERIODO_MUESTREO, 1, (void (*)(struct rtimer *, void *))muestreo, NULL);
		 if(ret){
			 PRINT_STD("Error Timer: %u\n", ret);
		 }
	 }
	 else {
		 process_poll(&medicion);
	 }

	 //Se realiza la lectura de los diferentes sensores. Esto es, se toman las muestras:
	 buff_C1[muestra] = phidgets.value(SENSOR_CORRIENTE_1);
	 buff_C2[muestra] = phidgets.value(SENSOR_CORRIENTE_2);
	 buff_V[muestra] = phidgets.value(SENSOR_VOLTAJE);

#if DEBUG_MUESTRAS
	 buff_Time_Stamp[muestra] = RTIMER_NOW();
#endif

	 muestra++; //Incremento la cuenta de muestra actual

	 return 1;

 }

 /*----------------------------FIN FUNCIONES MEDICIÓN-----------------------------*/



 /*------------------------------------------------------------------------------*/
 /*-----------------------------------PROCESOS-----------------------------------*/
 /*------------------------------------------------------------------------------*/

//Thred del proceso principal
PROCESS_THREAD(comunicacion, ev, data)
{

	PROCESS_BEGIN();

	//----------Tareas de diagnóstico y comunicación------------------

	PROCESS_PAUSE();

	PRINTF("Starting Smart Grid Server\n");

#ifdef RF_CHANNEL
	PRINTF("RF channel: %u\n", RF_CHANNEL);
#endif
#ifdef IEEE802154_PANID
	PRINTF("PAN ID: 0x%04X\n", IEEE802154_PANID);
#endif

	PRINTF("uIP buffer: %u\n", UIP_BUFSIZE);
	PRINTF("LL header: %u\n", UIP_LLH_LEN);
	PRINTF("IP+UDP header: %u\n", UIP_IPUDPH_LEN);
	PRINTF("REST max chunk: %u\n", REST_MAX_CHUNK_SIZE);

	/* Initialize the REST engine. */
	rest_init_engine();

	/*
	 * Bind the resources to their Uri-Path.
	 * WARNING: Activating twice only means alternate path, not two instances!
	 * All static variables are the same for each URI path.
	 */

	/*Activo los recursos que voy a utilizar*/
	rest_activate_resource(&res_mediciones_SG, "Reportes/Mediciones");
	rest_activate_resource(&res_comando_SG, "Comandos/Relays");

//--------FIN Tareas de diagnóstico y comunicación----------------


	/*while(1){
		/*En este proceso no se hace ninguna tarea periodica. Solo es importante en el arranque.
	}*/

	PROCESS_END();
}

//Thred del proceso de medicion
PROCESS_THREAD(medicion, ev, data)
{

  PROCESS_BEGIN();

  extern const struct sensors_sensor phidgets;

  PRINT_STD("\n");
  PRINT_STD("----------------------------------------\n");
  PRINT_STD("Iniciando funciones de medicion...\n");
  PRINT_STD("----------------------------------------\n");

  etimer_set(&periodico, TIEMPO_ENTRE_MEDIDAS*CLOCK_SECOND); //Expirado este tiempo se realizará la primer medición

//PRINT_STD("por entrar por 1ra vez al while\n"); //DEBUG

  while(1) {

	PROCESS_WAIT_UNTIL(etimer_expired(&periodico)); //Cuando este temporizador expira, se comienza una nueva medición

	/*Al comenzar la nueva medición, lo primero que hago es resetear el timer, de modo de garantizar que el tiempo
	*entre muestras se respete independientemente del tiempo insumido por las operaciones siguientes.
	*/
	etimer_reset(&periodico);

	static uint16_t i=0; //Variable auxiliar para for
	muestra = 1; //Inicializo el número de muestra actual

	SENSORS_ACTIVATE(phidgets); //Activo sensores

	char status_timer = muestreo(&tmuestreo, NULL); //Disparo el primer muestreo, luego sigue por autoinvocación

	/*Mientras se muestrea, el proceso quedará detenido en esta línea. Cuando se tome la última muestra,
	*la función "muestreo" hará un poll de este proceso y lo destrancará...
	*/
	PROCESS_WAIT_UNTIL(ev == PROCESS_EVENT_POLL);
	/*SENSORS_DEACTIVATE(phidgets); /*Desactivo sensores. Esta línea quedó comentada porque al desactivar los
	sensores me desactivaba también los relays.*/

	//Esta raya la imprimo por efecto visual, para que se note cuando llega una nueva medida en la consola
	PRINT_STD("\n");
	PRINT_STD("----------------------------------------\n");
	//PRINT_STD("termino muestreo\n"); //DEBUG

	//Variables auxiliares para cálculos
	static float VrmsAux=0;
	static float IrmsAux=0;
	static float Paux=0;
	static float Qaux=0;

	//Inicializo variables auxiliares
	VrmsAux=0;
	IrmsAux=0;
	Paux=0;
	Qaux=0;

	//Variables auxiliares para almacenar muestras en float y pasadas a voltaje.
	static float V_Samp_Volts; //Muestras de V pasadas a Volts
	static float C_Samp_Volts; //Muestras de C pasadas a volts
	static float C_Samp_Defasada_Volts; //Muestras de C desfasada pasadas a Volts
	static float Time_Stamp_mS; //Time stamp de cada muestra en usegundos.
	static int Time_Stamp_mS_Parte_Entera;
	static int Time_Stamp_mS_Parte_Decimal;
	static char Aux_Time_hay_Overflow;

	Aux_Time_hay_Overflow = 0; //Inicializo el flag de overflow

	PRINT_MUESTRAS("\n\n\n");
	PRINT_MUESTRAS("Tension [mV]; Corriente [mV]; Time stamp [mS];\n\n");
	//Para cada muestra calculo el valor medido por el sensor en VOLTS
	for (i=1; i<=TAM_VENTANA-OVERLAP_VENTANA; i++){

		V_Samp_Volts = ((buff_V[i]*VREF3)/4096.0)-VDC_3V;
		C_Samp_Volts = ((buff_C1[i]*VREF5)/4096.0)-VDC_5V;
		C_Samp_Defasada_Volts = ((buff_C1[i+OVERLAP_VENTANA]*VREF5)/4096.0)-VDC_5V;

/*Esta parte solo es necesaria cuando se quieren ver las muestras una a una con time stamp*/
#if DEBUG_MUESTRAS
		if((buff_Time_Stamp[i] < buff_Time_Stamp[i-1])&&(Aux_Time_hay_Overflow=0)){
			Aux_Time_hay_Overflow=1;
		}
		if(Aux_Time_hay_Overflow){
			Time_Stamp_mS = (buff_Time_Stamp[i]-buff_Time_Stamp[1]+65536)*MILISEG_POR_TICK; //Calculo el tiempo en mS a partir de 0uS
		}
		else{
			Time_Stamp_mS = (buff_Time_Stamp[i]-buff_Time_Stamp[1])*MILISEG_POR_TICK; //Calculo el tiempo en mS a partir de 0uS
		}

		//Artilugio para imprimir decimales
		Time_Stamp_mS_Parte_Entera = floor(Time_Stamp_mS);
		Time_Stamp_mS_Parte_Decimal = (int)((Time_Stamp_mS-Time_Stamp_mS_Parte_Entera) * 10000); //Me quedo con 4 decimales

		PRINT_MUESTRAS("%d; %d; %d.%d;\n",(int)(V_Samp_Volts*1000),(int)(C_Samp_Volts*1000),Time_Stamp_mS_Parte_Entera,Time_Stamp_mS_Parte_Decimal);
#endif

		/*IMPORTANTE: Acá falta incluir la elección de sensor de corriente y también falta
		hacer el cálculo de la corriente a partir del voltaje. Como está hecho hoy, la medición de
		corriente me da un valor de tensión, pero en ningún lado está la relación tensión-corriente
		que impone la placa preprocesadora*/
		VrmsAux = VrmsAux+powf(V_Samp_Volts,2);
		IrmsAux = IrmsAux+powf(C_Samp_Volts,2);
		Paux = Paux + V_Samp_Volts*C_Samp_Volts;
		Qaux = Qaux + V_Samp_Volts*C_Samp_Defasada_Volts;

	} //fin del for

	//struct Medida datos; //Calculo los valores finales y los guardo en la estructura

	datos.Vrms=sqrtf(VrmsAux/(TAM_VENTANA-OVERLAP_VENTANA));
	datos.Irms=sqrtf(IrmsAux/(TAM_VENTANA-OVERLAP_VENTANA));
	datos.p=Paux/(TAM_VENTANA-OVERLAP_VENTANA);
	datos.q=Qaux/(TAM_VENTANA-OVERLAP_VENTANA);
	datos.s=datos.Vrms*datos.Irms;
	datos.fp=datos.p/datos.s;

	//Se imprimen todos los valores medidos

	PRINT_STD("Medicion de Energia:\n");
	PRINT_STD("----------------------------------------\n");
	PRINT_STD("VRMS: %d [mV]\nIRMS: %d [mA]\nP: %d [mW]\nQ: %d [mVAR]\nS: %d [mVA]\nFP (%): %d\n",(int)(datos.Vrms*1000),(int)(datos.Irms*1000),(int)(datos.p*1000),(int)(datos.q*1000),(int)(datos.s*1000),(int)(datos.fp*100));
	PRINT_STD("----------------------------------------\n");

  }  /* while (1) */


  PROCESS_END();
}

//Thread del control de carga
PROCESS_THREAD(control_carga, ev, data)
{

#define BEBUG_RELAY 0

  PROCESS_BEGIN();

  /*Selecciono como P6.4 como DO de salida. En este caso estoy usando el cuatro, pero poniendo P6.x puedo acceder
   * a cualquier otro pin del puerto P6. Si quisiera usar otro puerto con GPIO para no usar ADCs, tendría que
   * modificar un poco las funciones usadas.
   */

#if BEBUG_RELAY
  SENSORS_ACTIVATE(button_sensor);
#endif

  /*Habilito salidas para ambos relays*/
  P6SEL &= ~(1 << P6_DO_RELAY_0);
  P6SEL &= ~(1 << P6_DO_RELAY_1);
  P6DIR |= (1 << P6_DO_RELAY_0);
  P6DIR |= (1 << P6_DO_RELAY_1);

  /*Arranco con ambos relays apagados*/
  P6OUT &= ~(1 << P6_DO_RELAY_0);
  P6OUT &= ~(1 << P6_DO_RELAY_1);
  status_relays = 0x00;

  while(1) {

#if BEBUG_RELAY
	   PROCESS_WAIT_EVENT_UNTIL((ev==sensors_event) && (data == &button_sensor));
#endif
	   PROCESS_WAIT_UNTIL(ev == PROCESS_EVENT_POLL);
	   ev = PROCESS_EVENT_NONE;

	   /*COMANDO DE RELAY 0*/
	   if ((comando_relays & 0x0F) > 0){
		   P6OUT |= (1 << P6_DO_RELAY_0); //Enciendo relay 0
		   status_relays |= 0x0F; //Actualizo estado: relay 0 encendido
	   }
	   else{
		   P6OUT &= ~(1 << P6_DO_RELAY_0); //Apago relay 0
		   status_relays &= 0xF0; //Actualizo estado: relay 0 apagado
	   }

	   /*COMANDO DE RELAY 1*/
	   if ((comando_relays & 0xF0) > 0){
		   P6OUT |= (1 << P6_DO_RELAY_1); //Enciendo relay 1
		   status_relays |= 0xF0; //Actualizo estado: relay 1 encendido
	   }
	   else{
		   P6OUT &= ~(1 << P6_DO_RELAY_1); //Apago relay 1
		   status_relays &= 0x0F; //Actualizo estado: relay 1 apagado
	   }

	   /*Monitoreo estado y señalizo con LEDs*/
	   /*LED AZUL = ESTADO R0 | LED VERDE = ESTADO R1*/
	   if ((status_relays & 0x0F) > 0){
		   leds_on(LEDS_BLUE);
	   }
	   else{
		   leds_off(LEDS_BLUE);
	   }
	   if ((status_relays & 0xF0) > 0){
		   leds_on(LEDS_GREEN);
	   }
	   else{
		   leds_off(LEDS_GREEN);
	   }


/*SOLO PARA DEBUG*/
#if BEBUG_RELAY
	   comando_relays=~comando_relays;
	   printf("\nComando [%d]\n", comando_relays);
#endif

  } //End while
  PROCESS_END();
}

