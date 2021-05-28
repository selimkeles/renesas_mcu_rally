#include "renesas_api.h"

#define UPRAMP 80.0   //50.0//70//80//90
#define FAST 45.0     //30.0
#define FAST_1 59.0
#define FAST_2 58.0
#define FAST_3 57.0
#define FAST_4 56.0
#define FAST_5 55.0
#define FAST_6 40.0    //30.0
#define MEDIUM 20.0   //20.0
#define SLOW 20.0     //10.0
#define BRAKE -40.0   //-40.0
#define CORNER 10

#define STRICT 1000.0 //1000.0
#define LOOSE 10.0    //10.0

enum states_t
{
  CURVE,
  FOLLOW,
  CORNER_IN,
  CORNER_OUT,
  RIGHT_CHANGE_DETECTED,
  RIGHT_TURN,
  LEFT_CHANGE_DETECTED,
  LEFT_TURN,
  RAMP_UP,
  RAMP_DOWN
} state = FOLLOW;

double last_time = 0.0;
int left_change_pending = 0;
int right_change_pending = 0;
int detected_state = FOLLOW;

int main(int argc, char **argv)
{
  wb_robot_init(); // this call is required for WeBots initialisation
  init();          // initialises the renesas MCU controller

  while (wb_robot_step(TIME_STEP) != -1)
  {
    update();
    unsigned short *sensor = line_sensor();
    double *angles = imu();
    float line = 0, sum = 0, weighted_sum = 0;
    bool double_line = 0;
    bool left_change = 0;
    bool right_change = 0;
    int i=10;

    //int hata = 0;
//   unsigned int sensor_value_1=0;
//   unsigned int sensor_value_2=0;

    
    for (int i = 0; i < 8; i++)
    {
    //printf("%d\t",sensor[i]);
/*    sensor_value_1 += sensor[i] * i * 1000;
      sensor_value_2 += sensor[i]; */
      weighted_sum += sensor[i] * i;
      sum += sensor[i];
      if (sensor[i] < 500)
      {
        double_line++;
        if (i < 4)
        {
          left_change++;
        }
        else
        {
          right_change++;
        }
      }
    }
   // printf("\n");
    line = weighted_sum / sum - 3.5;
     
  /*unsigned int position = sensor_value_1 / sensor_value_2;
  sensor_value_1=0;
  sensor_value_2=0;
  //printf("%d", position);
  //printf("\n");
  hata = position + (-3500);
      //printf("%d",  hata);
     // printf("\n");

*/
    switch (state)
    {
    case CURVE:
    
    
    
    
    
    case FOLLOW:
     motor(FAST, FAST, FAST, FAST);

     if (angles[0]==0){
       if(time() - last_time > 0.2)
       {
         i=i+5;
         motor(FAST+i, FAST+i, FAST+i, FAST+i);
       }
     }
     else {
      if(time() - last_time > 0.2){
         i=i-5;
         motor(FAST-i, FAST-i, FAST-i, FAST-i); 
       }
     }
 
      
      handle(STRICT * line);
      if (angles[1] > 0.1)
      {
        last_time = time();
        state = RAMP_UP;
        printf("RAMP UP\n");
      }
      else if (angles[1] < -0.1)
      {
        last_time = time();
        state = RAMP_DOWN;
        printf("RAMP DOWN\n");
      }
      else if (double_line > 6)
      {
        if (detected_state == CORNER_IN)
        {
          last_time = time();
          state = CORNER_IN;
          printf("CORNER IN\n");
        }
        detected_state = CORNER_IN;
      }
      else if (time() - last_time > 0.2 && right_change > 3)
      {
        if (detected_state == RIGHT_CHANGE_DETECTED)
        {
          last_time = time();
          state = RIGHT_CHANGE_DETECTED;
          printf("RIGHT_CHANGE_DETECTED\n");
        }
        detected_state = RIGHT_CHANGE_DETECTED;
      }
      else if (time() - last_time > 0.2 && left_change > 3)
      {
        if (detected_state == LEFT_CHANGE_DETECTED)
        {
          last_time = time();
          state = LEFT_CHANGE_DETECTED;
          printf("LEFT_CHANGE_DETECTED\n");
        }
        detected_state = LEFT_CHANGE_DETECTED;
      }
      break;
    case CORNER_IN:
      if (time() - last_time < 0.2)//0.1
      {
        motor(-50, -50, -50, -50);
      }
      else
      {
        motor(CORNER, CORNER, CORNER, CORNER);
      }

      handle(LOOSE * line);
      if (time() - last_time > 0.2 && (left_change > 3 || right_change > 3))
      {
        last_time = time();
        state = CORNER_OUT;
        printf("CORNER OUT\n");
      }
      break;
    case CORNER_OUT:
      motor(CORNER, CORNER, CORNER, CORNER);
      handle(STRICT * line);
      
      if (time() - last_time > 0.70)
      {
        last_time = time();
        state = FOLLOW;
        printf("FOLLOW\n");
      }
      break;
    case RIGHT_CHANGE_DETECTED:
      motor(MEDIUM, MEDIUM, MEDIUM, MEDIUM);
      handle(LOOSE * line);
      if (double_line == 0)
      {
        last_time = time();
        state = RIGHT_TURN;
        printf("RIGHT_TURN\n");
      }
      if (time() - last_time > 2.5)
      {
        last_time = time();
        state = FOLLOW;
        printf("FOLLOW\n");
      }
      break;
    case RIGHT_TURN:
      motor(MEDIUM, MEDIUM, MEDIUM, MEDIUM);
      if (time() - last_time < 0.35)//0.45
        handle(-40);
      else if (time() - last_time < 0.8)//0.6
        handle(40);//20
      else if (double_line > 1)
      {
        last_time = time();
        state = FOLLOW;
        printf("FOLLOW\n");
      }
      break;
    case LEFT_CHANGE_DETECTED:
      motor(MEDIUM, MEDIUM, MEDIUM, MEDIUM);
      handle(LOOSE * line);
      if (double_line == 0)
      {
        last_time = time();
        state = LEFT_TURN;
        printf("LEFT TURN\n");
      }
      if (time() - last_time > 2.5)
      {
        last_time = time();
        state = FOLLOW;
        printf("FOLLOW\n");
      }
      break;
    case LEFT_TURN:
      motor(MEDIUM, MEDIUM, MEDIUM, MEDIUM);
      if (time() - last_time < 0.35)//0.45
        handle(40);
      else if (time() - last_time < 0.8)//0.6
        handle(-40);//-20
      else if (double_line > 1)
      {
        last_time = time();
        state = FOLLOW;
        printf("FOLLOW\n");
      }
      break;
    case RAMP_UP:
      motor(UPRAMP, UPRAMP, UPRAMP, UPRAMP);
      handle(STRICT * line);
      if (angles[1] < 0.05)
      {
        last_time = time();
        state = FOLLOW;
        printf("FOLLOW\n");
      }
      break;
    case RAMP_DOWN:
      motor(SLOW, SLOW, SLOW, SLOW);
      handle(STRICT * line);
      if (angles[1] > -0.05)
      {
        last_time = time();
        state = FOLLOW;
        printf("FOLLOW\n");
      }
      break;
    }
  };

  wb_robot_cleanup(); // this call is required for WeBots cleanup

  return 0;
}