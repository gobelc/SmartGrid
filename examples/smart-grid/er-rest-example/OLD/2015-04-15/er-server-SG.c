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

#include <stdio.h>
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
//#include "dev/z1-phidgets.h" //Esto habría que borrarlo si paso a manipular los ADC directamente.
#include "z1-ADCs-SG.h"
//#include "dev/relay-phidget.h" //Creo que esto se puede borrar
#include "dev/leds.h"


/*-------------------------------CONFIGURACION DE MUESTREO------------------------------*/
/*
 * Es muy importante tener en cuenta algunos detalles a la hora de configurar los parámetros
 * de muestreo. El esquema de funcionamiento del muestreo y la medición es el siguiente:
 *
 * > Se adquieren MEDICIONES_PROM muestreos del largo de un ciclo y medio (50Hz)
 * cada TIEMPO_ENTRE_MEDIDAS_TICKS.
 * > Tras cada muestreo se calculan y almacenan los valores de las variables eléctricas
 * calculadas en base a dicho muestreo.
 * > Luego de obtener MEDICIONES_PROM mediciones, las mismas se promedian y almacenan
 * como resultado definitivo a reportar.
 * > El procedimiento anterior se realiza cada TIEMPO_ENTRE_REPORTES_SEG
 *
 * El tiempo de procesamiento de un muestreo tiene una duracion que llamaremos T_CALC.
 * Para que el muestreo y las mediciones sean coherentes, se debe mantener cierta
 * relación entre los siguientes parámetros:
 *
 * 1) {[TAM_VENTANA * (PERIODO_MUESTREO/RTIMER_SECOND)] + T_CALC} < (TIEMPO_ENTRE_MEDIDAS_TICKS/CLOCK_SECOND)
 *
 * 2) MEDICIONES_PROM * (TIEMPO_ENTRE_MEDIDAS_TICKS/CLOCK_SECOND) < TIEMPO_ENTRE_REPORTES_SEG
 *
 * ----------------------DEFAULT------------------------
 * TAM_VENTANA 246 //(164 + 82)
 * OVERLAP_VENTANA 82
 * PERIODO_MUESTREO 4
 * MEDICIONES_PROM 4
 * TIEMPO_ENTRE_MEDIDAS_TICKS 64
 * TIEMPO_ENTRE_REPORTES_SEG 4
 * -----------------------------------------------------
 *
 * Para estos valores se estimó T_CALC < 0.4 seg. Por tanto se cumplen las condiciones 1) y 2)
 *
 */
#define SENSOR_CORRIENTE_1 SENSOR_1
#define SENSOR_CORRIENTE_2 SENSOR_2
#define SENSOR_VOLTAJE SENSOR_3
#define ADC_VREF 2.5 //Tensión de referencia para sensores (entre AVss y 2.5V de ref interna)
#define TAM_VENTANA 246 //Cantidad de muestras tomadas en un muestreo
#define OVERLAP_VENTANA 82 //Muestras solapadas para cálculo de potencia reactiva
#define PERIODO_MUESTREO 4 //Es el período de muestreo medido en ticks. Cada tick vale 1/32768 [seg] (RTIMER_SECOND=32768 -> 1seg)
#define MILISEG_POR_TICK 0.030518 //Cada tick equivale a 0.030518 msegs
#define MEDICIONES_PROM 4 //Cantidad de mediciones a promediar
#define TIEMPO_ENTRE_MEDIDAS_TICKS 64//16 //Cada cuanto tiempo se realiza una adquisición y medición [Ticks] (128=CLOCK_SECOND=1 segundo)
#define TIEMPO_ENTRE_REPORTES_SEG 4 //Tiempo entre medidas reportadas en segundos

#define ADC_OFFSET_CALIBRACION -0.063 //Por experimento vimos que el mote mete un offset (V)
//#define OFFSET_MOTE_3V 0.0215 //Por experimento vimos que el mote mete un offset (V)
//#define OFFSET_MOTE_5V 0.054 //Por experimento vimos que el mote mete un offset (V)
#define SIGNAL_VDC (1.4 + ADC_OFFSET_CALIBRACION)//Tensión de continua de referencia que manda la placa de preprocesamiento.0.0215

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

#define DEBUG_RELAY 0
#if DEBUG_RELAY
  #include "dev/button-sensor.h"
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

static int flag_corriente; //1 para Corriente 1 ; 0 para Corriente 2.

//Buffers que contienen los valores muestreados
static uint16_t buff_C1[TAM_VENTANA];
static uint16_t buff_C2[TAM_VENTANA];
//static uint16_t buff_Corr[TAM_VENTANA];
static uint16_t buff_V[TAM_VENTANA];
#if DEBUG_MUESTRAS
static uint16_t buff_Time_Stamp[TAM_VENTANA];
#endif

static int muestra; //Contiene el número de muesta actual durante el muestreo
static uint8_t mediciones; //Memoriza el numero de mediciones realizadas para luego hacer un promedio entre estas

struct Medida datos_parcial; //Calculo de los valores parciales de las mediciones. Tras promediar varios ciclos se obtiene el dato final a reportar

static struct rtimer t_muestreo; //Timer de tiempo real que genera interrupciones para tomar las muestras (cuenta el período de muestreo)
static struct etimer t_medidas; //Timer utilizado para contar tiempos entre mediciones
static struct etimer t_reportes; //Timer utilizado para contar el tiempo entre medidas reportadas


/*---------------------FIN DECLARACIÓN DE VARIABLES----------------------------*/

/*------------------------------------------------------------------------------*/
/*----------------------------FUNCIONES MEDICIÓN--------------------------------*/
/*------------------------------------------------------------------------------*/


 /* FUNCIÓN muestreo: Al ser llamada esta función se encarga de tomar una tanda de muestras. Además, mientras no se
  * haya completado la totalidad de las muestras, la propia función agenda una próxima ejecución de ella misma
  * a través de un rtimer, un PERIODO_MUESTREO despues de la ejecución actual. De esta manera las muestras se toman
  * regularmente y a gran velocidad. Al llegar a la última muestra, la función le postea un evento poll al proceso
  * principal, y el mismo continúa.
  */

 static char muestreo(struct rtimer *rt, void* ptr){

	 if (muestra < TAM_VENTANA){

		 uint8_t ret;
		 ret = rtimer_set(&t_muestreo, RTIMER_NOW()+PERIODO_MUESTREO, 1, (void (*)(struct rtimer *, void *))muestreo, NULL);
		 if(ret){
			 PRINT_STD("Error Timer: %u\n", ret);
		 }
	 }
	 else {
		 process_poll(&medicion);
	 }

	 //Se realiza la lectura de los diferentes sensores. Esto es, se toman las muestras:
	 buff_C1[muestra] = sensores.value(SENSOR_CORRIENTE_1);
	 buff_C2[muestra] = sensores.value(SENSOR_CORRIENTE_2);
	 buff_V[muestra] = sensores.value(SENSOR_VOLTAJE);

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

	//PROCESS_PAUSE();

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

#if (!DEBUG_MUESTRAS && !PRINT_STANDARD && !DEBUG && !DEBUG_RELAY)
	rest_activate_resource(&res_mediciones_SG, "Reportes/Mediciones");
	rest_activate_resource(&res_comando_SG, "Comandos/Relays");
#endif

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

  mediciones = 0;

  extern const struct sensors_sensor sensores;

  PRINT_STD("\n");
  PRINT_STD("----------------------------------------\n");
  PRINT_STD("Iniciando funciones de medicion...\n");
  PRINT_STD("----------------------------------------\n");

  etimer_set(&t_reportes, TIEMPO_ENTRE_REPORTES_SEG*CLOCK_SECOND); //Expirado este tiempo se comenzará la primer medición

//PRINTF("por entrar por 1ra vez al while(1)\n"); //DEBUG

  while(1) {

	PROCESS_WAIT_UNTIL(etimer_expired(&t_reportes)); //Cuando este temporizador expira, se comienza una nueva medición
	etimer_reset(&t_reportes);
	PRINTF("Comienzo barrido en instante t = %u[s] = %lu[Ticks]\n", clock_seconds(), clock_time());
	etimer_set(&t_medidas, TIEMPO_ENTRE_MEDIDAS_TICKS); //Expirado este tiempo se realizará el primer muestreo

	while (mediciones <= MEDICIONES_PROM){

		static uint16_t i=0; //Variable auxiliar para for
		muestra = 1; //Inicializo el número de muestra actual

		SENSORS_ACTIVATE(sensores); //Activo sensores

		char status_timer = muestreo(&t_muestreo, NULL); //Disparo el primer muestreo, luego sigue por autoinvocación

		/*Mientras se muestrea, el proceso quedará detenido en esta línea. Cuando se tome la última muestra,
		*la función "muestreo" hará un poll de este proceso y lo destrancará...
		*/
		PROCESS_WAIT_UNTIL(ev == PROCESS_EVENT_POLL);
		/*SENSORS_DEACTIVATE(sensores); /*Desactivo sensores. Esta línea quedó comentada porque al desactivar los
		sensores me desactivaba también los relays.*/

		//PRINTF("termino muestreo en instante t = %u[s] = %u[Ticks]\n", clock_seconds(), clock_time()); //DEBUG

		//Variables auxiliares para cálculos
		static float VrmsAux;
		static float IrmsAux;
		static float Paux;
		static float Qaux;

		//Inicializo variables auxiliares
		VrmsAux=0;
		IrmsAux=0;
		Paux=0;
		Qaux=0;

		//Variables auxiliares para almacenar muestras en float y pasadas a voltaje.
		static float V_Samp_Volts; //Muestras de V pasadas a Volts
		static float C_Samp_Volts; //Muestras de C pasadas a volts
		static float C_Samp_Defasada_Volts; //Muestras de C desfasada pasadas a Volts
#if DEBUG_MUESTRAS
		static float Time_Stamp_mS; //Time stamp de cada muestra en usegundos.
		static int Time_Stamp_mS_Parte_Entera;
		static int Time_Stamp_mS_Parte_Decimal;
		static char Aux_Time_hay_Overflow;

		Aux_Time_hay_Overflow = 0; //Inicializo el flag de overflow
#endif
		PRINT_MUESTRAS("\n\n\n");
		PRINT_MUESTRAS("Tension [mV]; Corriente [mV]; Time stamp [mS]; Time stamp [Ticks];\n\n");

		//Para cada muestra calculo el valor medido por el sensor en VOLTS
		for (i=1; i<=TAM_VENTANA-OVERLAP_VENTANA; i++){

			V_Samp_Volts = ((buff_V[i]*ADC_VREF)/4096.0)-SIGNAL_VDC;
			C_Samp_Volts = ((buff_C1[i]*ADC_VREF)/4096.0)-SIGNAL_VDC;
			C_Samp_Defasada_Volts = ((buff_C1[i+OVERLAP_VENTANA]*ADC_VREF)/4096.0)-SIGNAL_VDC;

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
			Time_Stamp_mS_Parte_Decimal = (int)((Time_Stamp_mS-Time_Stamp_mS_Parte_Entera) * 10); //Me quedo con 1 decimales

			PRINT_MUESTRAS("%d; %d; %d.%u; %u;\n",(int)(V_Samp_Volts*1000),(int)(C_Samp_Volts*1000),Time_Stamp_mS_Parte_Entera,Time_Stamp_mS_Parte_Decimal,buff_Time_Stamp[i]);
	#endif

			/*IMPORTANTE: Acá falta incluir la elección de sensor de corriente y también falta
			hacer el cálculo de la corriente a partir del voltaje. Como está hecho hoy, la medición de
			corriente me da un valor de tensión, pero en ningún lado está la relación tensión-corriente
			que impone la placa preprocesadora*/
			VrmsAux += powf(V_Samp_Volts,2);
			IrmsAux += powf(C_Samp_Volts,2);
			Paux += V_Samp_Volts*C_Samp_Volts;
			Qaux += V_Samp_Volts*C_Samp_Defasada_Volts;

		} //fin de for (i=1; i<=TAM_VENTANA-OVERLAP_VENTANA; i++)

		if (mediciones == 0) {

			datos_parcial.Vrms=0;
			datos_parcial.Irms=0;
			datos_parcial.p=0;
			datos_parcial.q=0;
			datos_parcial.s=0;
			datos_parcial.fp=0;

			mediciones = 1;
		}
		if (mediciones <= MEDICIONES_PROM){

			PRINTF("Muestreo numero %d realizado en instante t = %u[s] = %lu[Ticks]\n", mediciones, clock_seconds(), clock_time()); //DEBUG

			datos_parcial.Vrms+=(sqrtf(VrmsAux/(TAM_VENTANA-OVERLAP_VENTANA)))/MEDICIONES_PROM;
			datos_parcial.Irms+=(sqrtf(IrmsAux/(TAM_VENTANA-OVERLAP_VENTANA)))/MEDICIONES_PROM;
			datos_parcial.p+=(Paux/(TAM_VENTANA-OVERLAP_VENTANA))/MEDICIONES_PROM;
			datos_parcial.q+=(Qaux/(TAM_VENTANA-OVERLAP_VENTANA))/MEDICIONES_PROM;
			//datos_parcial.s+=(datos.Vrms*datos.Irms)/MEDICIONES_PROM;
			//datos_parcial.fp+=(datos.p/datos.s)/MEDICIONES_PROM;
			mediciones++;
		}
		if (mediciones > MEDICIONES_PROM) {

			/*No paso los datos definitivos hasta terminar con el cálculo de todos lo ciclos a promediar*/

			datos.Vrms=datos_parcial.Vrms;
			datos.Irms=datos_parcial.Irms;
			datos.p=datos_parcial.p;
			datos.q=datos_parcial.q;
			datos.s=datos_parcial.Vrms*datos_parcial.Irms;
			datos.fp=datos_parcial.p/datos_parcial.s;

			//Se imprimen todos los valores medidos

			PRINT_STD("<><><><><><><><><><><><><><><><><><><><>\n");
			PRINT_STD("Medicion de Energia en instante t=%u[s]:\n", clock_seconds());
			PRINT_STD("----------------------------------------\n");
			PRINT_STD("VRMS: %d [mV]\nIRMS: %d [mA]\nP: %d [mW]\nQ: %d [mVAR]\nS: %d [mVA]\nFP (%): %d\n",(int)(datos.Vrms*1000),(int)(datos.Irms*1000),(int)(datos.p*1000),(int)(datos.q*1000),(int)(datos.s*1000),(int)(datos.fp*100));
			PRINT_STD("<><><><><><><><><><><><><><><><><><><><>\n");

		}

		PROCESS_WAIT_UNTIL(etimer_expired(&t_medidas)); //Cuando este temporizador expira, se comienza un nuevo muestreo

		/*Al comenzar la nueva medición, lo primero que hago es resetear el timer, de modo de garantizar que el tiempo
		*entre muestras se respete independientemente del tiempo insumido por las operaciones siguientes.
		*/
		etimer_reset(&t_medidas);

	} /*while (mediciones <= MEDICIONES_PROM)*/

	//Esta raya la imprimo por efecto visual, para que se note cuando llega una nueva medida en la consola
	PRINT_STD("\n");
	PRINT_STD("----------------------------------------\n");

	mediciones = 0;

  }  /* while (1) */


  PROCESS_END();
}

//Thread del control de carga
PROCESS_THREAD(control_carga, ev, data)
{

  PROCESS_BEGIN();

  /*Selecciono P6.2 y P6.4 como DO de salida. En este caso estoy usando el cuatro, pero poniendo P6.x puedo acceder
   * a cualquier otro pin del puerto P6. Si quisiera usar otro puerto con GPIO para no usar ADCs, tendría que
   * modificar un poco las funciones usadas.
   */

#if DEBUG_RELAY
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

#if DEBUG_RELAY
	   PROCESS_WAIT_EVENT_UNTIL((ev==sensors_event) && (data == &button_sensor));
#else
	   PROCESS_WAIT_UNTIL(ev == PROCESS_EVENT_POLL);
	   ev = PROCESS_EVENT_NONE;
#endif

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
	   printf("\nComando [%d]\n", comando_relays);
	   printf("\nStatus [%d]\n", status_relays);

/*SOLO PARA DEBUG*/
#if DEBUG_RELAY
	   comando_relays=~comando_relays;
	   printf("\nComando [%d]\n", comando_relays);
#endif

  } //End while
  PROCESS_END();
}

