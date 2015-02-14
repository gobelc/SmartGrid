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
#define PERIODO_MUESTREO 1/10000

static int W=1000; //Muestras de ventana, típicamente 3 ciclos de la señal de entrada
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

/*#if PLATFORM_HAS_LEDS
extern resource_t res_leds, res_toggle;
#endif*/

/*#if PLATFORM_HAS_BATTERY
#include "dev/battery-sensor.h"
extern resource_t res_battery;
#endif*/

PROCESS(er_server, "Smart Grid Server Node");
AUTOSTART_PROCESSES(&er_server);

// INICIALIZACION DE VARIABLES

 /*

  	  static double Vref=3.4; // Voltaje de referencia CC
  	  static double Rshunt=1; //Resistencia shunt transductor corriente
  	  static double Theta1=23; //Desfasaje natural voltaje/corriente n°1
  	  static double Theta2=34; //Desfasaje natural voltaje/corriente n°1 Threshold
  	  static int Fmuestreo; //Frecuencia de muestreo ADC
  	  static int Fred; //Frecuencia de red eléctrica
  	  double Volt;
  	  double Volt90;
  	  double x;

 */
   	  static double Gi1=1; //Ganancia V/A Amplificador n°1 corriente
   	  static double Gi2=1; //Ganancia V/A Amplificador n°2 corriente
   	  static double Gv=1; //Ganancia V/V Voltaje transductor vs Voltaje de Red

  	  static double CurrentThreshold=2.3;
  	  double B[251];

  	 float Vrms=0;
  	  	 	float Irms=0;
  	  	 	float p=0;
  	  	 	float q=0;
  	  	 	float s=0;
  	  	 	float fp=0;


  	  int flag_corriente; //1 para Corriente 1 ; 0 para Corriente 2.
  	  int count=0;
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

 	 void leer_voltaje(void){
  	 	shift(B,W/4);
 		B[W/4] = Gv*phidgets.value(SENSOR_VOLTAJE)*VREF/(4096);
 	 	return;
 	 }

 	 double leer_corriente_1(void){
 	 	double value=Gi1*phidgets.value(SENSOR_CORRIENTE_1)*VREF/(4096);
 	 	return value;
 	 }

 	 double leer_corriente_2(void){
 	 	double value=Gi2*phidgets.value(SENSOR_CORRIENTE_2)*VREF/(4096);
 	 	return value;
 	 }

 	 int definir_escala_corriente(void){

 	 	double suma=0;
 	 	double promedio;
 	 	int i=0;
 	 	for (i=0;i<50;i++){
 	 		//suma=filtrado(leer_corriente_2(),coef_1,coef_2,coef_3,coef_4)+suma;
 	 	suma=leer_corriente_2();
 	 	}

 	 	promedio=suma/(i+1);

 	 	if (promedio<CurrentThreshold){
 	 		flag_corriente=1;
 	 		}
 	 		else{
 	 			flag_corriente=0;
 	 			}
 	 	return flag_corriente;
 	 	}

 	 void lectura_inicial(){
 	 	flag_corriente=definir_escala_corriente();
 	 	int i=0;
 	 	for (i=0;i<W/4;i++){
 	 			leer_voltaje();
 	 			printf("Lectura inicial %i : %i Volts %i\n",i,(int)B[W/4],(int)B[0]);
 	 		}


 	 return;
 	 }






 	 struct Medida medir_unidades(){

 		double A=0; // W muestras de corriente

 		int i;

 		double a=0;
 	 	double b=0;
 	 	double c=0;
 	 	double d=0;
 	 //	double e=0;

		static uint16_t buff_C1[TAM_VENTANA];
		static uint16_t buff_C2[TAM_VENTANA];
		static uint16_t buff_Corr[TAM_VENTANA];
		static uint16_t buff_V[TAM_VENTANA];
		static uint16_t *buff_ptrC1;
		static uint16_t *buff_ptrC2;
		static uint16_t *buff_ptrV;
		static struct etimer tmuestreo;

		buff_ptrC1 = buff_C1;
		buff_ptrC2 = buff_C2;
		buff_ptrV = buff_V;

		for (i=0; i<TAM_VENTANA; i++) {
			// Timer y período de muestreo:
    			etimer_set(&tmuestreo, PERIODO_MUESTREO * CLOCK_SECOND);
  			PROCESS_WAIT_UNTIL(etimer_expired(&tmuestreo));

    			SENSORS_ACTIVATE(phidgets);
			*buff_ptrC1 = phidgets.value(SENSOR_CORRIENTE_1);
			*buff_ptrC2 = phidgets.value(SENSOR_CORRIENTE_2);
			*buff_ptrV = phidgets.value(SENSOR_VOLTAJE);	
    			SENSORS_DEACTIVATE(phidgets);
	
			buff_ptrC1++;
			buff_ptrC2++;
			buff_ptrV++;
		}
		//Luego de obtenidas todas las muestras, se vuelve el puntero al inicio.
		buff_ptrC1 = buff_C1;
		buff_ptrC2 = buff_C2;
		buff_ptrV = buff_V;
		
		double VrmsAux=0;
		double IrmsAux=0;
		double Paux=0;
		double Qaux=0;

		if (definir_escala_corriente == 1){
			buff_Corr = buff_C1;
		}else{
			buff_Corr = buff_C2;
		}
		
		for (i=0; i<TAM_VENTANA-50; i++){
			VrmsAux = VrmsAux+buff_V[i]*buff_V[i];
 			IrmsAux = IrmsAux+buff_Corr[i]*buff_Corr[i];
			Paux = Paux + buff_V[i]*buff_Corr[i];
			Qaux = Qaux + buff_V[i]*buff_Corr[i+50];
 		}

 		struct Medida r;

 	 	r.Vrms=sqrtf(VrmsAux/(TAM_VENTANA-50));
 	 	r.Irms=sqrtf(IrmsAux/(TAM_VENTANA-50));
 	 	r.p=Paux/(TAM_VENTANA-50);
 	 	r.q=Qaux/(TAM_VENTANA-50);
 	 	r.s=sqrtf(r.p*r.p+r.q*r.q);
 	 	r.fp=r.p/r.s;


 	 	printf("VRMS:  %d \n IRMS:  %d \n P:  %d \n Q:  %d \n S:  %d \n FP: %d \n",(int)r.Vrms,(int)r.Irms,(int)r.p,(int)r.q,(int)r.s,(int)r.fp);
 	 	//printf("VRMS: %ld.%03d mV)\n", (long) Vrms,(unsigned) ((Vrms - floor(Vrms)) * 1000));




 	 	return r;

}

//------------Fin funciones metering-------------------------

PROCESS_THREAD(er_server, ev, data)
{

  static struct etimer temporizador; // declaro un temporizador como testigo
  static int32_t testigo = 0; // declaro una variable de prueba

//static struct etimer periodico;

  PROCESS_BEGIN();

//------------------Declaraciones metering--------------------------------------

//extern const struct sensors_sensor phidgets;
//SENSORS_ACTIVATE(phidgets);



printf("Realizando la lectura inicial...");
//lectura_inicial();
printf("Medicion en curso...");

//static int32_t testigo = 0; // declaro una variable de prueba
etimer_set(&temporizador, 5*CLOCK_SECOND); //seteo el temporizador a 1 min.
//etimer_set(&periodico, 5*CLOCK_SECOND); //seteo el temporizador a 1 min.

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


/*#if PLATFORM_HAS_BUTTON
	SENSORS_ACTIVATE(button_sensor);
#endif*/

//--------FIN Funciones de diagnóstico, información y auxiliares----------------
//------------------------------------------------------------------------------


  while(1) {

	  /*PROCESS_WAIT_UNTIL(etimer_expired(&periodico));
	  struct Medida m = medir_unidades();
	  dato_get=(int32_t)(m.Vrms);
	  etimer_reset(&periodico);*/

	  //PROCESS_WAIT_UNTIL(etimer_expired(&periodico));
	  //return 0;

	  PROCESS_WAIT_EVENT();



	if (ev == PROCESS_EVENT_TIMER && etimer_expired(&temporizador)) {
		if (testigo < 1000) {
			++testigo;
		}
		else {
			testigo = 0;
			}
			printf("testigo = %d \n",testigo);
			dato_get = testigo + 100;
			dato_obs = testigo + 200;
			dato_ev = testigo + 300;

			etimer_reset(&temporizador);
		}

#if PLATFORM_HAS_BUTTON
		else if ((ev == sensors_event) && (data == &button_sensor)) {
		printf("Alguien apreto el boton!\n");

		      /* Acá llamo al trigger del recurso de evento "res_event_SG
		       * El triger se encargará de hacer las tareas disparadas por
		       * dicho evento */
		res_event_SG.trigger();
		}
#endif /* PLATFORM_HAS_BUTTON */
	

  }                             /* while (1) */

  PROCESS_END();
}
