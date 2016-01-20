/*
 * Autor: Saúl Ibáñez Cerro
 */
#include "usarPid.h"
#include <stdio.h>
using namespace std;
Pid::Pid(){
	errorAnt=0;
	contador=0;
	for (int i = 0; i < 10; i++)
	{
		arr[i]=0;
	}
	sumarCeldasArray=0;
	contador2=0;
	for (int i = 0; i < 10; i++)
	{
		arr2[i]=0;
	}
	SumarCeldas=0;
}

/*
* Con esta funcion vamos a poder calcular el PID
*/
float Pid::OperarMiPid(float error){
	p=0.87*error;

	d=0.08*(errorAnt-error);
	errorAnt=error;

	arr[contador]=error;
	if (contador==9)
		contador=(contador+1)%10;
	else
		contador++;

	for (int k = 0; k < 10; k++)
		sumarCeldasArray=sumarCeldasArray+arr[k];
	
	i=0.05*(sumarCeldasArray/10.0);
	w=p+i+d;
	return w;
}


/*
* Esta función me va a servir para calcular la media de pixeles en hsv
* He colocado aqui la funcion porque me he basado en el método PID calculado
* anteriormente.
*/
float Pid::OperarMiMedia(int media){
	//printf(" media: %i \n", media);
	arr2[contador2]=media;
	if (contador2==9){
    	contador2=(contador2+1)%10;
	}else
    	contador2++;

    for (int k = 0; k < 10; k++){
		SumarCeldas=SumarCeldas+arr2[k];
		//printf("SumarCeldas: %i \n", arr2[k]);
	}

	Total=(float)SumarCeldas/10.0;
	SumarCeldas=0;
	//printf(" clculamos el total: %f \n", Total);
	return Total; 
}

/*
PARA PROBAR QUE EL PROGRAMA PARA CALCULAR EL PID ESTA CORRECTAMENTE IMPLEMENTADO
SE HA CREADO UN MAIN QUE SE DEJA COMENTADO, SI EJECUTAMOS EL PROGRAMA COMO C++
DEBERIAMOS PONER EN UN TERMINAL LOS COMANDOS:
			g++ usarPid.cpp
			g++ -o usarPid usarPid.cpp
			./usarPid 

int main(int argc, char **argv){
	Pid pan;
	printf("%f \n", pan.OperarMiPid(1.0));
}
*/

