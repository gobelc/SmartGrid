/*
 * metering.c
 *
 *  Created on: Dec 20, 2014
 *      Author: Gonzalo Belcredi, Pablo Modernell, Nicolás Sosa
 */

#include <stdio.h>
#include "core/cfs/cfs.h"
#include <math.h>
#include "contiki.h"
#include "lib/sensors.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "dev/z1-phidgets.h"

#define SENSOR_CORRIENTE_1 0
#define SENSOR_CORRIENTE_2 0
#define SENSOR_VOLTAJE 3
#define VREF 5

static int W=1000; //Muestras de ventana, típicamente 3 ciclos de la señal de entrada


/*---------------------------------------------------------------------------*/
PROCESS(metering_process, "Medicion y reporte de energia");
AUTOSTART_PROCESSES(&metering_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(metering_process, ev, data)
{
  //static struct etimer timer;
  PROCESS_BEGIN();
  extern const struct sensors_sensor phidgets;
  SENSORS_ACTIVATE(phidgets);

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

 	  double x_3;
 	  double x_2;
 	  double x_1;

 	 typedef struct Medida {
 		float Vrms;
 		float Irms;
 		float q;
 		float p;
 		float s;
 		float fp;
 	 } Medida;

 	 void shift(double a[], int n) {
 	 	     int i;
 	 	     for(i = 0; i!= n; i++){
 	 	        a[i] = a[i+1];
 	 	     }
 	 }



 	int desfasaje(int SENSOR1, int SENSOR2){

 		int deltaN=0;

 		float value1=phidgets.value(SENSOR1);
 		float value2=phidgets.value(SENSOR2);

 		int a=signo(value1);
 		 		int b=signo(value2);

 		if (a==1)
 			while (signo(value1)){
 				value1=phidgets.value(SENSOR1);
 				value2=phidgets.value(SENSOR2);
 				b=signo(value2);
 			}
 			if (b==1)
 				while (b==1){
 					value2=phidgets.value(SENSOR2);
 					b=signo(value2);
 					deltaN++;
 				}
 			if (b==0)
 				while (b==0){
 					value2=phidgets.value(SENSOR2);
 					b=signo(value2);
 				}
 				while (b==1){
 					value2=phidgets.value(SENSOR2);
 			 		b=signo(value2);
 			 		deltaN++;
 				}

 		if (a==0)
 				while (!signo(value1)){
 					value1=phidgets.value(SENSOR1);
 				 	value2=phidgets.value(SENSOR2);
 				 	b=signo(value2);
 				}
				if (b==0)
					while (b==0){
						value2=phidgets.value(SENSOR2);
						b=signo(value2);
						deltaN++;
					}
				if (b==1)
					while (b==1){
						value2=phidgets.value(SENSOR2);
						b=signo(value2);
					}
					while (b==0){
						value2=phidgets.value(SENSOR2);
						b=signo(value2);
						deltaN++;
					}
	return deltaN;
 	}

 	int signo(float data){
 		if (data>0)
 			return 1;
 		else {
 			return 0;
 		}
 	}



 	double filtrado(x,coef_1,coef_2,coef_3,coef_4){
 	 	 	// FIR de media movil

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






 	void medir_unidades(){

 		double A=0; // W muestras de corriente

 		int i;

 		double a=0;
 	 	double b=0;
 	 	double c=0;
 	 	double d=0;
 	 //	double e=0;


 		for (i=0;i<W;i++){
 			leer_voltaje();
 			//B[W/4]=filtrado(B[W/4],coef_1,coef_2,coef_3,coef_4);
 			A=leer_corriente_2();
 			if (flag_corriente){
 	 				// A=filtrado(leer_corriente_2(),coef_1,coef_2,coef_3,coef_4);
 	 				A=leer_corriente_2();
 	 				leds_on(LEDS_RED);
 				}
 				else {
 						// A=filtrado(leer_corriente_1(),coef_1,coef_2,coef_3,coef_4);
 					A=leer_corriente_1();
 					leds_on(LEDS_GREEN);
 				}

 			a=A*A+a;
 			b=B[W/4]*B[W/4]+b;
 			c=A*B[W/4]+c;
 			d=A*B[0]+d;

 		}

 		struct Medida r;

 	 	r.Vrms=sqrtf(a/W);
 	 	r.Irms=sqrtf(b/W);
 	 	r.p=d/W;
 	 	r.q=c/W;
 	 	r.s=sqrtf(p*p+q*q);
 	 	r.fp=p/s;


 	 	printf("VRMS:  %d \n IRMS:  %d \n P:  %d \n Q:  %d \n S:  %d \n FP: %d \n",(int)r.Vrms,(int)r.Irms,(int)r.p,(int)r.q,(int)r.s,(int)r.fp);
 	 	//printf("VRMS: %ld.%03d mV)\n", (long) Vrms,(unsigned) ((Vrms - floor(Vrms)) * 1000));
 	 	medir_unidades();




 	 	return;

}

 while(1) {
	printf("Realizando la lectura inicial...");
	lectura_inicial();
	printf("Medicion en curso...");
	int a=0;
	while (1){
			a++;
		}
 	medir_unidades();
	return 0;
 }

   PROCESS_END();
}
 /*---------------------------------------------------------------------------*/
