#include "agv_nav/mecumnamu.h"

mecumnamu::mecumnamu(float &a,float &b):sum(a+b){}
mecumnamu::~mecumnamu(){}

void mecumnamu::wheel2center(const float wheel_speed[4],float *center_speed){
    center_speed[0] = 0.25*(-1*wheel_speed[0]+wheel_speed[1]-wheel_speed[2]+wheel_speed[3]);
    center_speed[1] = 0.25*(wheel_speed[0]+wheel_speed[1]+wheel_speed[2]+wheel_speed[3]);
    center_speed[3] = 0.25*(wheel_speed[0]-wheel_speed[1]-wheel_speed[2]+wheel_speed[3]) / this->sum;
}
void mecumnamu::center2wheel(float *wheel_speed,const float center_speed[4]){
    wheel_speed[0] = center_speed[1] - center_speed[0] + center_speed[2]*this->sum;
    wheel_speed[1] = center_speed[1] + center_speed[0] - center_speed[2]*this->sum;
    wheel_speed[2] = center_speed[1] - center_speed[0] - center_speed[2]*this->sum;
    wheel_speed[3] = center_speed[1] + center_speed[0] + center_speed[2]*this->sum;
}