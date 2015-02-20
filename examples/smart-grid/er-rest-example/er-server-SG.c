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

/*este es el nuevo*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"
#include "rest-engine.h"
#include "interface.h"

//---------------Metering--------------------
#include "core/cfs/cfs.h"
#include <math.h>
#include "lib/sensors.h"
#include "dev/leds.h"
#include "dev/z1-phidgets.h"

#define SENSOR_CORRIENTE_1 0
#define SENSOR_CORRIENTE_2 0
#define SENSOR_VOLTAJE 3
#define VREF 5
#define TAM_VENTANA 250 
#define PERIODO_MUESTREO 1/128 //128 es la máxima resolución (div de seg) que se alcanza con los timers estandar
#define TIEMPO_ENTRE_MEDIDAS 5

#define VDC 1.0 //Tensión de continua de referencia que manda la placa de preprocesamiento.

//-------------------------------------------

#if PLATFORM_HAS_BUTTON
#include "dev/button-sensor.h"
#endif

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

/*
 * Resources to be activated need to be imported through the extern keyword.
 * The build system automatically compiles the resources in the corresponding sub-directory.
 */

extern resource_t res_get_SG;
extern resource_t res_obs_SG;
extern resource_t res_event_SG;
extern resource_t res_put_SG;


PROCESS(er_server, "Smart Grid Server Node");
AUTOSTART_PROCESSES(&er_server);

// INICIALIZACION DE VARIABLES


static double Gi1=1; //Ganancia V/A Amplificador n°1 corriente
static double Gi2=1; //Ganancia V/A Amplificador n°2 corriente
static double Gv=1; //Ganancia V/V Voltaje transductor vs Voltaje de Red

static double CurrentThreshold=2.3; //¿Qué es?
static double B[251]; //¿Qué es?

static float Vrms=0;
static float Irms=0;
static float p=0;
static float q=0;
static float s=0;
static float fp=0;


static int flag_corriente; //1 para Corriente 1 ; 0 para Corriente 2.

// Coeficientes del Filtro Pasabajos
static double coef_1=1/4;
static double coef_2=1/4;
static double coef_3=1/4;
static double coef_4=1/4;



typedef struct Medida {
	float Vrms;
	float Irms;
	float q;
	float p;
	float s;
	float fp;
} Medida;

//------------------FIN Declaraciones metering--------------------------------------


// Funciones Metering


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

//------------Fin funciones metering-------------------------

PROCESS_THREAD(er_server, ev, data)
{

	int i;

	static uint16_t buff_C1[TAM_VENTANA];
	static uint16_t buff_C2[TAM_VENTANA];
	static uint16_t buff_Corr[TAM_VENTANA];
	static uint16_t buff_V[TAM_VENTANA];
	static uint16_t *buff_ptrC1;
	static uint16_t *buff_ptrC2;
	static uint16_t *buff_ptrV;

	static struct etimer tmuestreo;

	static struct etimer periodico;


  PROCESS_BEGIN();


	extern const struct sensors_sensor phidgets;

	buff_ptrC1 = buff_C1;
	buff_ptrC2 = buff_C2;
	buff_ptrV = buff_V;

printf("Realizando la lectura inicial...");

etimer_set(&periodico, TIEMPO_ENTRE_MEDIDAS*CLOCK_SECOND); //seteo el temporizador a 1 min.

//----------Funciones de diagnóstico, información y auxiliares------------------
//------------------------------------------------------------------------------
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

  rest_activate_resource(&res_get_SG, "reportes/prueba_get");
  rest_activate_resource(&res_obs_SG, "reportes/prueba_obs");
  rest_activate_resource(&res_event_SG, "reportes/prueba_ev");
  rest_activate_resource(&res_put_SG, "reportes/prueba_put");

  /* Define application-specific events here. */

//--------FIN Funciones de diagnóstico, información y auxiliares----------------
//------------------------------------------------------------------------------


  while(1) {

	  PROCESS_WAIT_UNTIL(etimer_expired(&periodico));

	  etimer_reset(&periodico);
	  static uint16_t i1=0;
	  static uint16_t i2=0;
	  //printf("ContBuffV: %d, ContPtrV: %d\n DirBuffV: %d, DirPtrV: %d\n Voltaje: %d\n",buff_V[i1],*buff_ptrV, &buff_V[i1],buff_ptrV,voltaje);
	  SENSORS_ACTIVATE(phidgets);
	  etimer_set(&tmuestreo, PERIODO_MUESTREO * CLOCK_SECOND);
	  for (i1=0; i1<TAM_VENTANA; i1++) {
			// Timer y período de muestreo:
			PROCESS_WAIT_UNTIL(etimer_expired(&tmuestreo));
			etimer_set(&tmuestreo, PERIODO_MUESTREO * CLOCK_SECOND);

			*buff_ptrC1 = phidgets.value(SENSOR_CORRIENTE_1);
			*buff_ptrC2 = phidgets.value(SENSOR_CORRIENTE_2);
			*buff_ptrV = phidgets.value(SENSOR_VOLTAJE);

			//printf("ContBuffV: %d, ContPtrV: %d\n DirBuffV: %d, DirPtrV: %d\n Voltaje: %d\n",buff_V[i1],*buff_ptrV, &buff_V[i1],buff_ptrV,voltaje);
			buff_ptrC1++;
			buff_ptrC2++;
			buff_ptrV++;
		}

		//Luego de obtenidas todas las muestras, se vuelve el puntero al inicio.
		buff_ptrC1 = buff_C1;
		buff_ptrC2 = buff_C2;
		buff_ptrV = buff_V;

		static float VrmsAux=0;
		static float IrmsAux=0;
		static float Paux=0;
		static float Qaux=0;

		VrmsAux=0;
		IrmsAux=0;
		Paux=0;
		Qaux=0;
		//Variables auxiliares para almacenar muestras en float y pasadas a voltaje.
		static float VSampConvert;
		static float CSampConvert;
		static float CSampDefConvert;


		for (i2=0; i2<TAM_VENTANA-50; i2++){
			VSampConvert = buff_V[i2]*VREF/4096-VDC;
			printf("V sin convertir: %d\n",(int)(buff_V[i2]));
			printf("V convertido con continua x100: %d\n",(int)(100*buff_V[i2]*VREF/4096));
			printf("V convertido sin continua x100: %d\n",(int)(VSampConvert*100));
			CSampConvert = buff_C1[i2]*VREF/4096-VDC;
			CSampDefConvert = buff_C1[i2+50]*VREF/4096-VDC;

			VrmsAux = VrmsAux+powf(VSampConvert,2);
			IrmsAux = IrmsAux+powf(CSampConvert,2);
			Paux = Paux + VSampConvert*CSampConvert;
			Qaux = Qaux + VSampConvert*CSampDefConvert;
			//printf("V: %d\n C1: %d\n",buff_V[i2],buff_C1[i2] );
			//printf("V en buffer: %d\n",(int)buff_V[i2]);
			//printf("Paux: %d\n",(int)Paux);


		}

		struct Medida r;

		r.Vrms=sqrtf(VrmsAux/(TAM_VENTANA-50));
		r.Irms=sqrtf(IrmsAux/(TAM_VENTANA-50));
		r.p=Paux/(TAM_VENTANA-50);
		r.q=Qaux/(TAM_VENTANA-50);
		r.s=r.Vrms*r.Irms;
		r.fp=r.p/r.s;

		printf("VRMS (x100):  %d \n IRMS (x100):  %d \n P (x100):  %d \n Q (x100):  %d \n S(x100):  %d \n FP (x100): %d \n",(int)(r.Vrms*100),(int)(r.Irms*100),(int)(r.p*100),(int)(r.q*100),(int)(r.s*100),(int)(r.fp*100));
		//printf("VRMS: %ld.%03d mV)\n", (long) Vrms,(unsigned) ((Vrms - floor(Vrms)) * 1000));


		dato_get=(int32_t)(r.Vrms);



	  }  /* while (1) */



  PROCESS_END();
}
