#include "flow.h"

#define MOVAVG_SIZE  5	   //±£´æ×î½üµÄ5®¸öÊý¾Ý½øÐÐÆ½¾ùÂË²¨
//#define QUEUE_SIZE   5     //¶ÓÁÐ´óÐ¡
static float Dist_Xspeed_buffer[MOVAVG_SIZE] = {0};
static float Dist_Yspeed_buffer[MOVAVG_SIZE] = {0};
//static float Dist_Xcompentation_buffer[QUEUE_SIZE] = {0,0,0,0,0};//´æ·Å²¹³¥ÖµµÄ¶ÓÁÐ
//static float Dist_Ycompentation_buffer[QUEUE_SIZE] = {0,0,0,0,0};
static uint8_t Dis_index = 0;
//static uint8_t Queue_front = 0,Queue_rear = 3;//¶ÓÁÐÍ·Ö¸ÕëºÍÎ²Ö¸Õë,Î²½øÍ·³ö
unsigned char flowdata[47];
short int X_Speed,Y_Speed;
float Xmove=0,Ymove=0;
float tempx,tempy;
FLOW_DATA flow;



void Flow_XYspeed_NewDis(float xspeed,float yspeed) 
{
  Dist_Xspeed_buffer[Dis_index] = xspeed;
	Dist_Yspeed_buffer[Dis_index] = yspeed;
  Dis_index = ((Dis_index + 1) % MOVAVG_SIZE);  
}

float Flow_XYspeed_getAvg(float * buff, int size)
{
  float sum = 0.0;
  int i;
  for(i=0; i<size; i++) {
    sum += buff[i];
  }
  return (sum / (float)size);
}


//¶ÁÖ¸¶¨¼Ä´æÆ÷Ö¸¶¨×Ö½ÚÊýÊý¾Ý
u8 flow_read_data(u8 addr,u8 reg,u8 len,u8 *buf)
{
    IIC_Start(); 
    IIC_Send_Byte((addr<<1)|0);//·¢ËÍÆ÷¼þµØÖ·+Ð´ÃüÁî 
    if(IIC_Wait_Ack())    //µÈ´ýÓ¦´ð
    {
        IIC_Stop();         
        return 1;        
    }
    IIC_Send_Byte(reg);    //Ð´¼Ä´æÆ÷µØÖ·
    IIC_Wait_Ack();        //µÈ´ýÓ¦´ð
    IIC_Start();
    IIC_Send_Byte((addr<<1)|1);//·¢ËÍÆ÷¼þµØÖ·+¶ÁÃüÁî    
    IIC_Wait_Ack();        //µÈ´ýÓ¦´ð 
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//¶ÁÊý¾Ý£¬·¢ËÍnACK
        else *buf=IIC_Read_Byte(1);        //¶ÁÊý¾Ý£¬·¢ËÍnACK
        len--;
        buf++; 
    }    
    IIC_Stop();    //²úÉúÒ»¸öÍ£Ö¹Ìõ¼þ
    return 0;    
}

u8 flow_get_data(u8 *buf)
{
	//Ã»ÓÃµ½µÄ×¢ÊÍ£¨±ðÉ¾£©£¬½âËã¿ìÒ»µãÊÇÒ»µã
	flow.frame_count_sum = (flowdata[1]<<8)|flowdata[0];
	flow.pixel_flow_x_sum = (flowdata[3]<<8)|flowdata[2];
	flow.pixel_flow_y_sum = (flowdata[5]<<8)|flowdata[4];
	flow.flow_comp_m_x = (flowdata[7]<<8)|flowdata[6];
	flow.flow_comp_m_y = (flowdata[9]<<8)|flowdata[8];
	flow.qual = (flowdata[11]<<8)|flowdata[9];
	flow.gyro_x_rate = (flowdata[13]<<8)|flowdata[12];
	flow.gyro_y_rate = (flowdata[15]<<8)|flowdata[14];
	flow.gyro_z_rate = (flowdata[17]<<8)|flowdata[16];
	flow.gyro_range = flowdata[18];
	flow.sonar_timestamp1 = flowdata[19];
	flow.ground_distance1 = (flowdata[21]<<8)|flowdata[20];
	
//	flow.frame_count_since_last_readout = (flowdata[23]<<8)|flowdata[22];
//	flow.pixel_flow_x_integral = (flowdata[25]<<8)|flowdata[24];
//	flow.pixel_flow_y_integral = (flowdata[27]<<8)|flowdata[26];
//	flow.gyro_x_rate_integral = (flowdata[29]<<8)|flowdata[28];
//	flow.gyro_y_rate_integral = (flowdata[31]<<8)|flowdata[30];
//	flow.gyro_z_rate_integral = (flowdata[33]<<8)|flowdata[32];
//	flow.integral_timespan = flowdata[37] << 24 | flowdata[36] << 16 | flowdata[35] << 8 | flowdata[34];
//	flow.sonar_timestamp2 = flowdata[41] << 24 | flowdata[40] << 16 | flowdata[39] << 8 | flowdata[38];
//	flow.ground_distance12 = (flowdata[43]<<8)|flowdata[42];
//	flow.gyro_temperature = (flowdata[45]<<8)|flowdata[44];
//	flow.quality = flowdata[46];
}
#if SerialDebug
void flow_send_data(void)
{
			 char string_to_send1[80]={0};
/*******************************·Ö¶Î·¢ËÍ************************************/
			 sprintf(string_to_send1, "FRAME_COUNT_SUM:%d\r\n",flow.frame_count_sum);
			 UART1_Put_String((unsigned char *)string_to_send1);
			 sprintf(string_to_send1, "PIXEL_FLOW_X_SUM:%d\r\n",flow.pixel_flow_x_sum);
			 UART1_Put_String((unsigned char *)string_to_send1);
			 sprintf(string_to_send1, "PIXEL_FLOW_Y_SUM:%d\r\n",flow.pixel_flow_y_sum);
			 UART1_Put_String((unsigned char *)string_to_send1);
			 sprintf(string_to_send1, "FLOW_COMP_M_X:%d\r\n",flow.flow_comp_m_x);
			 UART1_Put_String((unsigned char *)string_to_send1);
			 sprintf(string_to_send1, "FLOW_COMP_M_Y:%d\r\n",flow.flow_comp_m_y);
			 UART1_Put_String((unsigned char *)string_to_send1);
			 sprintf(string_to_send1, "QUAL:%d\r\n",flow.qual);
			 UART1_Put_String((unsigned char *)string_to_send1);
			 sprintf(string_to_send1, "GYRO_X_RATE:%d\r\n",flow.gyro_x_rate);
			 UART1_Put_String((unsigned char *)string_to_send1);
			 sprintf(string_to_send1, "GYRO_Y_RATE:%d\r\n",flow.gyro_y_rate);
			 UART1_Put_String((unsigned char *)string_to_send1);
			 sprintf(string_to_send1, "GYRO_Z_RATE:%d\r\n",flow.gyro_z_rate);
			 UART1_Put_String((unsigned char *)string_to_send1);
			 sprintf(string_to_send1, "GYRO_RANGE:%d\r\n",flow.gyro_range);
			 UART1_Put_String((unsigned char *)string_to_send1);
			 sprintf(string_to_send1, "SONAR_TIMESTAMP1:%d\r\n",flow.sonar_timestamp1);
			 UART1_Put_String((unsigned char *)string_to_send1);
			 sprintf(string_to_send1, "GROUND_DISTANCE1:%d\r\n",flow.ground_distance1);
			 UART1_Put_String((unsigned char *)string_to_send1);
/*******************************·Ö¶Î·¢ËÍ************************************/
//			 sprintf(string_to_send1, "FRAME_COUNT_SINCE_LAST_READOUT:%d\r\n",flow.frame_count_since_last_readout);
//			 UART1_Put_String((unsigned char *)string_to_send1);
//			 sprintf(string_to_send1, "PIXEL_FLOW_X_INTEGRAL:%d\r\n",flow.pixel_flow_x_integral);
//			 UART1_Put_String((unsigned char *)string_to_send1);
//			 sprintf(string_to_send1, "PIXEL_FLOW_Y_INTEGRAL:%d\r\n",flow.pixel_flow_y_integral);
//			 UART1_Put_String((unsigned char *)string_to_send1);
//			 sprintf(string_to_send1, "GYRO_X_RATE_INTEGRAL:%d\r\n",flow.gyro_x_rate_integral);
//			 UART1_Put_String((unsigned char *)string_to_send1);
//			 sprintf(string_to_send1, "GYRO_Y_RATE_INTEGRAL:%d\r\n",flow.gyro_y_rate_integral);
//			 UART1_Put_String((unsigned char *)string_to_send1);
//			 sprintf(string_to_send1, "GYRO_Z_RATE_INTEGRAL:%d\r\n",flow.gyro_z_rate_integral);
//			 UART1_Put_String((unsigned char *)string_to_send1);
//			 sprintf(string_to_send1, "INTEGRATION_TIMESPAN:%d\r\n",flow.integral_timespan);
//			 UART1_Put_String((unsigned char *)string_to_send1);
/*******************************·Ö¶Î·¢ËÍ************************************/
//			 sprintf(string_to_send1, "SONAR_TIMESTAMP2:%d\r\n",flow.sonar_timestamp2);
//			 UART1_Put_String((unsigned char *)string_to_send1);
//			 sprintf(string_to_send1, "GROUND_DISTANCE2:%d\r\n",flow.ground_distance12);
//			 UART1_Put_String((unsigned char *)string_to_send1);
//			 sprintf(string_to_send1, "GYRO_TEMPERATURE:%d\r\n",flow.gyro_temperature);
//			 UART1_Put_String((unsigned char *)string_to_send1);
//			 sprintf(string_to_send1, "QUALITY:%d\r\n",flow.quality);
//			 UART1_Put_String((unsigned char *)string_to_send1);
/*test*/
//			   sprintf(string_to_send1, "FLOW_COMP_M_X:%d\r\n",flow.flow_comp_m_x);
//			   UART1_Put_String((unsigned char *)string_to_send1);
//			   sprintf(string_to_send1, "FLOW_COMP_M_Y:%d\r\n",flow.flow_comp_m_y);
//			   UART1_Put_String((unsigned char *)string_to_send1);
//				 sprintf(string_to_send1, "PIXEL_FLOW_X_INTEGRAL:%d\r\n",flow.pixel_flow_x_integral);
//			   UART1_Put_String((unsigned char *)string_to_send1);
//				 sprintf(string_to_send1, "X_INTEGRAL:%d\r\n",flow.pixel_flow_x_integral);
//				 UART1_Put_String((unsigned char *)string_to_send1);
//			   sprintf(string_to_send1, "Y_INTEGRAL:%d\r\n",flow.pixel_flow_y_integral);
//			   UART1_Put_String((unsigned char *)string_to_send1);
}
#endif

//¶Á8Î»ÎÞ·ûºÅÊý¾Ý
uint8_t     readu8_date(u8 addr,u8 reg)
{
    u8 buff[1];
    uint8_t date;
    flow_read_data(addr,reg,1,buff);
    date = buff[0];
    return date;
}
//¶Á16Î»ÎÞ·ûºÅÊý¾Ý
uint16_t    readu16_date(u8 addr,u8 reg)
{
    u8 buff[2];
    uint16_t date;
    flow_read_data(addr,reg,2,buff);
    date = buff[1] << 8 | buff[0];
    return date;

}
//¶Á16Î»ÓÐ·ûºÅÊý¾Ý
int16_t     reads16_date(u8 addr,u8 reg)
{
    u8 buff[2];
    int16_t date;
    flow_read_data(addr,reg,2,buff);
    date = buff[1] << 8 | buff[0];
    return date;
}
//¶Á32Î»ÎÞ·ûºÅÊý¾Ý
uint32_t    readu32_date(u8 addr,u8 reg)
{
    u8 buff[4];
    int16_t date;
    flow_read_data(addr,reg,2,buff);
    date = buff[3] << 24 | buff[2] << 16 | buff[1] << 8 | buff[0];
    return date;
}

//¸üÐÂ¹âÁ÷Êý¾Ý
void Flow_Routing()
{
	uint32_t now_time;
	static uint32_t last_time = 0;
	float dt;
	float altitude;
	float compensation_x,compensation_y,diff_roll,diff_pitch,COEFFICIENT_X = 1.0,COEFFICIENT_Y = 1.0;
	static short int Last_X_Speed = 0,Last_Y_Speed = 0;
	
	char string_to_send1[100]={0};

	if(last_time == 0){
		last_time = micros();
		return ;
	}
	now_time = micros(); 
	if(now_time < last_time){ 
		last_time = now_time;
		return ;
	}
	dt = (float)(now_time - last_time);
	dt /= 1000000.0f;  
	last_time=now_time;
	
	altitude = (MS5611_Altitude + MS5611_Altitude_Last)*0.5;
	diff_roll = IMU_Roll - IMU_Roll_Last;
	diff_pitch = IMU_Pitch - IMU_Pitch_Last;
	compensation_x = (altitude*diff_roll*COEFFICIENT_X)/dt*3.1415926/180.0*10;
	compensation_y = (altitude*diff_pitch*COEFFICIENT_Y)/dt*3.1415926/180.0*10;
//	compensation_x = Math_fConstrain(compensation_x,-250.0f,+250.0f);;
//	compensation_y = Math_fConstrain(compensation_y,-250.0f,+250.0f);;
	
	flow_read_data(FLOW_ADDR,0,50,flowdata);
	flow_get_data(flowdata);
	
	if(compensation_x * flow.flow_comp_m_x < 0)
		compensation_x = -compensation_x;
	if(compensation_y * flow.flow_comp_m_y < 0)
		compensation_y = -compensation_y;
	if(abs((int)compensation_x)<(abs(flow.flow_comp_m_x)*1.5))
	{
  	X_Speed = flow.flow_comp_m_x - compensation_x;					
	}
	else
	{
		X_Speed = flow.flow_comp_m_x - compensation_x*0.5;	
	}
	if(abs((int)compensation_y)<(abs(flow.flow_comp_m_y)*1.5))
	{				
  	Y_Speed = flow.flow_comp_m_y - compensation_y;
	}
	else
	{
		Y_Speed = flow.flow_comp_m_y - compensation_y*0.5;
	}
	X_Speed = Last_X_Speed + 
	(dt / (5.3052e-3 + dt) * X_Speed - Last_X_Speed);//100hz
	Y_Speed = Last_Y_Speed + 
	(dt / (5.3052e-3 + dt) * Y_Speed - Last_Y_Speed);
	Last_X_Speed = X_Speed;
	Last_Y_Speed = Y_Speed;
	
	Flow_XYspeed_NewDis(X_Speed,Y_Speed);  //·ÅÈë»¬¶¯¶ÓÁÐ£¬È¡¾ùÖµ
	X_Speed = Flow_XYspeed_getAvg( Dist_Xspeed_buffer, MOVAVG_SIZE);
	Y_Speed = Flow_XYspeed_getAvg( Dist_Yspeed_buffer, MOVAVG_SIZE);

if((((int16_t)(PWM_Input_CH1 - PWM_Input_Offset) > (int16_t)-35)&&((int16_t)(PWM_Input_CH1 - PWM_Input_Offset) < (int16_t)35))&&(((int16_t)(PWM_Input_CH2 - PWM_Input_Offset) > (int16_t)-35)&&((int16_t)(PWM_Input_CH2 - PWM_Input_Offset) < (int16_t)35)))
	{
		Xmove = Xmove + X_Speed*dt;
		Ymove = Ymove + Y_Speed*dt;
	}
	else
	{
		Xmove = Position_X_Hold.target;
		Ymove = Position_Y_Hold.target;
	}
}

