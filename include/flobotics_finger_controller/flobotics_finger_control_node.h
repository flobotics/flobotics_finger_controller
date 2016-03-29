float adc0=1;
float adc1=1;
float adc2=1;
float adc3=1;
float adc4=1;
float adc5=1;
float adc6=1;
float adc7=1;
float proximal_limit_min_1=10;
float proximal_limit_max_1=50;
float proximal_limit_hold_1=25;
float proximal_limit_min_2=10;
float proximal_limit_max_2=50;
float proximal_limit_hold_2=25;
float proximal_1_limit_min_1=10;
float proximal_1_limit_max_1=50;
float proximal_1_limit_hold_1=25;
float proximal_1_limit_min_2=10;
float proximal_1_limit_max_2=50;
float proximal_1_limit_hold_2=25;
float intermediate_limit_min_1=10;
float intermediate_limit_max_1=50;
float intermediate_limit_hold_1=25;
float intermediate_limit_min_2=10;
float intermediate_limit_max_2=50;
float intermediate_limit_hold_2=25;
float distal_limit_min_1=10;
float distal_limit_max_1=50;
float distal_limit_hold_1=25;
float distal_limit_min_2=10;
float distal_limit_max_2=50;
float distal_limit_hold_2=25;
uint8_t mode=0;

uint16_t cmp0=0, cmp1=0, cmp2=0, cmp3=0, cmp4=0, cmp5=0, cmp6=0, cmp7=0;
float limit_1 = 10, limit_2=50;
int count = 0;
enum mode {
  STOP = 0,
  IDLE,
  START_DRAWBACK,
  PROXIMAL_1_DRAWBACK,
  PROXIMAL_2_DRAWBACK,
  INTERMEDIATE_DRAWBACK,
  DISTAL_DRAWBACK,
};
flobotics_finger_messages::flobotics_finger_servo_control_values msg;
ros::Publisher servo_pub;

void start_drawback();
void stop_drawback();
void proximal_1_drawback();
void proximal_2_drawback();
void intermediate_drawback();
void distal_drawback();
