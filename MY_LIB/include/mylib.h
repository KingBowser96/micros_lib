// libs padrão
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <byteswap.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

// libs que estão no include
#include <i2cutil.h>
#include <galileo2io.h>
#include <jhdlcd.h>

//-----------------------------------------------------------------

// constantes utilizadas pelo ADC continuo
#define DATA_POINTS 1000 // 1000 amostras por sensor
#define SAMPLING_PERIOD 1e-3 // periodo de amostragem = 1 ms

// struct utilizada pelo ADC continuo com 4 sensores diferentes
struct sensors
{
        uint16_t pot;		/* be:u12/16>>0 */
        uint16_t light;		/* be:u12/16>>0 */
        uint16_t sound;		/* be:u12/16>>0 */
        uint16_t temp;		/* be:u12/16>>0 */
        int64_t timestamp;	/* le:s64/64>>0 */
};

//-----------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////////////////////// FUNÇÕES


// usada para se comunicar com um dispositivo I2C (ou seja, o LCD)
// a função writeLCD utiliza essa função
int i2c_write_reg(int fd,unsigned char reg,unsigned char data);


// como usar:
// pputs( "path" , "1" );
// ou
// snprintf( str , sizeof str , "%d\n" , duty_cycle );
// pputs( "path" , str );
int pputs(const char path[],const char s[]);


// como usar:
// pgets( str , sizeof str , "path" );
char *pgets(char *s,int size,const char path[]);


// ajusta o servo motor para o angulo desejado
// -90º < angle < 90º
int servoAngle(double angle);


// função para ler o valor de um potenciometro utilizando ADC one-shot
// retorna o valor em Volts (double)
double readOneShot();


// função para ler o valor de 4 sensores utilizando ADC continuo
// não faz nada ainda, tem que arrumar na hora
int readContinuous();


// função para escrever nas duas linhas do LCD
void writeLCD(const char *line1, const char *line2);

