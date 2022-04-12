class mecumnamu
{
public:
    mecumnamu(float &a,float &b);
    ~mecumnamu();
    void wheel2center(const float wheel_speed[4],float *center_speed);
    void center2wheel(float *wheel_speed,const float center_speed[4]);
private:
    float sum;
};