#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>

#define INTERVALO 100 // maior que 127 nao funciona pois os contadores estao em char
#define CONJ_T 1
#define CONJ_GBELLMF 2
#define ANDMETHOD_MIN 1
#define ANDMETHOD_PROD 2
#define ORMETHOD_MAX 1
#define MAMDANI 1
#define SUGENO 2

#define debug 0
#define fuzzy_min(a, b) ((a) < (b) ? (a) : (b))
#define fuzzy_max(a, b) ((a) > (b) ? (a) : (b))

class Conjunto {
public:
	char tipo;
	float a,b,c,d;
    
	float membership(float x);
};

class LinearOutput {
public:
	float *c;
    
	float evaluate(float *x);
};

typedef struct {
	char numConjuntos;
	float min, max;
	Conjunto *conj;
	LinearOutput *linear_conj;
} Variavel;

typedef struct {
	unsigned char *conjEntrada, *conjSaida;
} Regra;

class FuzzySystem {
public:
	unsigned char tipo, or_method, and_method;
	unsigned char NUMINPUTS, NUMOUTPUTS, NUMRULES;
	Variavel *var;
	Regra *regras;
	float *saidas;
	float *num;
	float *den;
    float **forca_conj_saida;
    float *var_saida;
	
	FuzzySystem(unsigned char t, unsigned char orm, unsigned char andm, unsigned char inputs, unsigned char outputs, unsigned char rules, Variavel *v, Regra *r);
    
    void alocarVariaveis(void);
    void limparVariaveis(void);
	float* rodarSistema(float *x);
    void agregarSaida(unsigned char numsaida);
    void centroid(unsigned char numsaida);
    
};

void rodarMeuFis(float* x, float** y);