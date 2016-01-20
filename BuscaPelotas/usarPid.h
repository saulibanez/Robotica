/*
 * Autor: Saúl Ibáñez Cerro
 */
#ifndef UsarPID
#define UsarPID

class Pid{
	public:
		Pid();
		float OperarMiPid(float error);
		float OperarMiMedia(int media);
	private:
		float p,i,d;
		float w;
		float errorAnt;
		float sumarCeldasArray;
		int contador;
		int arr [10];
		int arr2 [10];

		int SumarCeldas;
		int TotalFiltrado;
		int contador2;
        float Total;
};
#endif