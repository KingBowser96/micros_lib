#include "../include/mylib.h"


// usada para se comunicar com um dispositivo I2C (ou seja, o LCD)
// a função writeLCD utiliza essa função
int i2c_write_reg(int fd,unsigned char reg,unsigned char data)
{
        unsigned char buf[]={reg,data};
        
        return write(fd,buf,sizeof buf);
}

// como usar:
// pputs( "path" , "1" );
// ou
// snprintf( str , sizeof str , "%d\n" , duty_cycle );
// pputs( "path" , str );
int pputs(const char path[],const char s[])
{
	int fd;
	int n;
	
	if((fd=open(path,O_WRONLY)) == -1)
		return -1;
	
	n=write(fd,s,strlen(s));
	close(fd);
	return n;
}

// como usar:
// pgets( str , sizeof str , "path" );
char *pgets(char *s,int size,const char path[])
{
	int fd;
	
	if((fd=open(path,O_RDONLY)) ==-1)
		return NULL;
	
	read(fd,s,size);
	close(fd);
	return s;
}

// ajusta o servo motor para o angulo desejado
// -90º < angle < 90º
int servoAngle(double angle)
{
	int duty_cycle;
	double angle_rad;
	char str[100];
	
	angle_rad = angle *  M_PI/180.0;
	
	if((angle > M_PI_2) || (angle < - M_PI_2))
	{
		puts("Out of range.");
		return -1;
	}
	
	// (angulo/90º * 850 us) +  1500us
	// angulo minimo -> 650us
	// angulo maximo -> 2350us
	duty_cycle = angle_rad/M_PI_2 * 850000 + 1500000;
			
	// periodo é 20 milhoes de ns = 20ms = 50 Hz
	pputs("/sys/class/pwm/pwmchip0/device/pwm_period","20000000");
	
	// passa o valor do duty_cycle (int) para o str (string)
	snprintf(str,sizeof str,"%d\n",duty_cycle);
	pputs("/sys/class/pwm/pwmchip0/pwm1/duty_cycle",str);
	
	pputs("/sys/class/pwm/pwmchip0/pwm1/enable","1");
}

// função para ler o valor de um potenciometro utilizando ADC one-shot
// retorna o valor em Volts (double)
double readOneShot()
{
	int fd;
	char data_str[80];
	double scale;
	double voltage;
	int raw;

	if((fd=open("/sys/bus/iio/devices/iio:device0/in_voltage0_raw",O_RDONLY)) < 0)
	{
		perror("Opening in_voltage0raw:");
		return -1;
	}
	
	// fator de escala para transformar o valor bruto em volts
	pgets(data_str,sizeof data_str,"/sys/bus/iio/devices/iio:device0/in_voltage0_scale");
	scale = atof(data_str)/1000.0;
	
	// lê o valor bruto
	lseek(fd,0,SEEK_SET);
	read(fd,data_str,sizeof data_str);
	raw = atoi(data_str);
	
	// mostra o valor bruto, o fator de escala e o valor final
	voltage = raw * scale;
	printf("Raw=%d\tScale=%f\tVoltage=%fV\n", raw , scale , voltage );
	
	close(fd);
	
	return voltage;
}


// função para ler o valor de 4 sensores utilizando ADC continuo
// não faz nada ainda, tem que arrumar na hora
int readContinuous()
{
	char data_str[80];
	double scale[4];
	int fd;
	static struct sensors data[DATA_POINTS];
	int i;
	int samples;
	FILE *file;
	char path_str[80];
	
	// desativa o buffer
	pputs("/sys/bus/iio/devices/iio:device0/buffer/enable","0");
	
	// lê os 4 fatores de escala e ativa os 4 canais de leitura
	for(i=0;i < 4;i++)
	{
		snprintf(path_str,sizeof path_str,"/sys/bus/iio/devices/iio:device0/in_voltage%d_scale",i);
		pgets(data_str,sizeof data_str,path_str);
		scale[i] = atof(data_str)/1000.0;
		
		snprintf(path_str,sizeof path_str,"/sys/bus/iio/devices/iio:device0/scan_elements/in_voltage%d_en",i);
		pputs(path_str,"1");
	}
	
	// ativa o timestamp
	pputs("/sys/bus/iio/devices/iio:device0/scan_elements/in_timestamp_en","1");

	// ajusta a quantidade de amostras
	snprintf(data_str,sizeof data_str,"%d",DATA_POINTS);
	pputs("/sys/bus/iio/devices/iio:device0/buffer/length",data_str);
	
#ifdef TRIG_SYSFS
	// escolhe o trigger por software
	pgets(data_str,sizeof data_str,"/sys/bus/iio/devices/trigger0/name");
	pputs("/sys/bus/iio/devices/iio:device0/trigger/current_trigger",data_str);
#else
	// escolhe o trigger por hardware e ajusta a frequencia de amostragem (1/periodo)
	pgets(data_str,sizeof data_str,"/sys/bus/iio/devices/trigger1/name");
	pputs("/sys/bus/iio/devices/iio:device0/trigger/current_trigger",data_str);
	snprintf(data_str,sizeof data_str,"%d",(int)round(1.0/SAMPLING_PERIOD));
	pputs("/sys/bus/iio/devices/trigger1/frequency",data_str);
#endif        

	// ativa o buffer (vai começar)
	pputs("/sys/bus/iio/devices/iio:device0/buffer/enable","1");

#ifdef TRIG_SYSFS
	// se é por software: dispara um trigger pra cada amostra (e tem um delay de segurança...)
	for(i=0; i < DATA_POINTS;i++)
	{
			pputs("/sys/bus/iio/devices/trigger0/trigger_now","1");
			usleep(ceil(SAMPLING_PERIOD*1e6));
	}
#else
	// se é por hardware: só esperar...
	// mas tem que esperar de acordo com o periodo de amostragem e a quantidade de amostras
	sleep(ceil(DATA_POINTS*SAMPLING_PERIOD));
#endif
	
	// desativa o buffer e o trigger (ja coletou todas as amostras)
	pputs("/sys/bus/iio/devices/iio:device0/buffer/enable","0");
	pputs("/sys/bus/iio/devices/iio:device0/trigger/current_trigger","\n");

	// as conversões ficam salvas nesse arquivo aqui...
	if((fd=open("/dev/iio:device0",O_RDONLY)) < 0)
	{
			perror("Opening /dev/iio:device0:");
			return -1;
	}
	
	// salva as amostras em data (data é um array de structs sensors)
	// samples é a quantidade de amostras bem sucedidas (acho que aqui tem aquele bug das potencias de 2)
	samples = read(fd,data,sizeof data)/sizeof(struct sensors);
	close(fd);
	
	// desativa algo???
	pputs("/sys/bus/iio/devices/iio:device0/buffer/length","2");
	
	// desativa os 4 canais de amostragem
	for(i=0;i < 4;i++)
	{
			snprintf(path_str,sizeof path_str,"/sys/bus/iio/devices/iio:device0/scan_elements/in_voltage%d_en",i);
			pputs(path_str,"0");
	}
	
	// desativa o timestamp
	pputs("/sys/bus/iio/devices/iio:device0/scan_elements/in_timestamp_en","0");
	
	// manipula os dados lidos e printa bonitinho na tela
	for(i=0;i < samples;i++)
	{
			data[i].pot=bswap_16(data[i].pot);
			data[i].light=bswap_16(data[i].light);
			data[i].sound=bswap_16(data[i].sound);
			data[i].temp=bswap_16(data[i].temp);                
			
			printf("%f\t%f\t%f\t%f\t%f\n",
					data[i].pot*scale[0],
					data[i].light*scale[1],
					data[i].sound*scale[2],
					data[i].temp*scale[3],
					(data[i].timestamp-data[0].timestamp)*1e-9);
	}
	fclose(file);
	
	return 0;
}


// função para escrever nas duas linhas do LCD
void writeLCD(const char *line1, const char *line2)
{
  int n1, n2;
  int fd;
  int i;

  if ((fd = open("/dev/i2c-0", O_WRONLY)) < 0)
  {
	printf("Erro ao abrir o arquivo do LCD");  
    return;
  }
  
  /* Wait for 30 ms after power on */
  usleep(30000); 

  // TUDO DAQUI PRA BAIXO É PRA FAZER O SETUP CORRETO DO LCD
  
  /* LCD initialization */
  if (ioctl(fd, I2C_SLAVE, LCD_ADDR) < 0)
  {
	printf("Erro ao inicializar o LCD");  
    return;
  }

  i2c_write_reg(fd, LCD_C0, LCD_FUNCTIONSET | LCD_2LINE);
  usleep(40); /* Wait for more than 39 us */

  i2c_write_reg(fd, LCD_C0, LCD_DISPLAYSWITCH | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
  usleep(40); /* Wait for more then 39 us */

  i2c_write_reg(fd, LCD_C0, LCD_SCREENCLEAR);
  usleep(1600); /* Wait for more then 1.53 ms */

  i2c_write_reg(fd, LCD_C0, LCD_INPUTSET | LCD_ENTRYLEFT | LCD_DECREMENT);

  /* Backlight initialization */
  if (ioctl(fd, I2C_SLAVE, BL_ADDR) < 0)
  {
	printf("Erro ao inicializar o backlight");  
    return;
  }
  
  i2c_write_reg(fd, BL_MODE1, 0);
  i2c_write_reg(fd, BL_LEDOUT, BL_RED_GRPPWM | BL_GREEN_GRPPWM | BL_BLUE_GRPPWM);
  i2c_write_reg(fd, BL_MODE2, BL_DMBLNK);

  // aqui é pra escolher a cor de fundo [0,255]
  i2c_write_reg(fd, BL_RED, 255);
  i2c_write_reg(fd, BL_GREEN, 0);
  i2c_write_reg(fd, BL_BLUE, 0);

  // AQUI ACABA O SETUP DO LCD 
  
  // pega o tamanho das duas strings (cada string vai ser colocada em uma linha do LCD se possivel)
  n1 = strlen(line1);
  n2 = strlen(line2);

  if (ioctl(fd, I2C_SLAVE, LCD_ADDR) < 0)
    return;

  // escreve a string na primeira linha
  for (i = 0; i < n1; i++)
    i2c_write_reg(fd, LCD_RS, line1[i]);

  // pula os espaços em branco (o LCD é um array enorme, e a segunda linha começa a partir da posição 40?)
  for (i = 0; i < 40 - n1; i++)
    i2c_write_reg(fd, LCD_RS, ' ');
  
  // escreve a string na segunda linha
  for (i = 0; i < n2; i++)
    i2c_write_reg(fd, LCD_RS, line2[i]);

  close(fd);
}