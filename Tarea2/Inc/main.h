/*
 * main.h
 *
 *  Created on: May 30, 2025
 *      Author: paula-fdez
 */

#ifndef MAIN_H_
#define MAIN_H_

//Definimos una enumeracion con los diferentes estados que puede tener la maquina
typedef enum{
	refrescar,
	cambiar_numero,
	aumentar_tasa_refresco,
	disminuir_tasa_refresco,
	resetear,
	IDLE,
}posiblesEstados;

typedef struct
{
	posiblesEstados estado;
}estadoActual;


//Enumeramos las posibles partes que tenemos en numeroDisplay
typedef enum{
	unidad1 =0,
	decena1,
	centena1,
	milUnidad1,
}parteNumero1;

typedef struct
{
	parteNumero1 parte;
}parteNumero;

#endif /* MAIN_H_ */
