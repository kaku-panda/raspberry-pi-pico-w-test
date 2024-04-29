#include<stdlib.h>

#define LINE_MIDDLE 4.5
#define PROPORTION  0.1
#define SPEED       0.4
#define MAX_SPEED   255

float get_average_line_position(uint8_t sensor_value){
    int length = 0;
    int sum    = 0;
    for(int n = 0; n < 8; n++){
        if(sensor_value >> n & 1){
            sum += n;
            length++;
        }
    }
    if(length == 0){
        return 0;
    }else{
       return sum / length;
    }
}

uint32_t control_motors(uint8_t sensor_value){
    float position = get_average_line_position(sensor_value);
    float error = LINE_MIDDLE - position;

    float correction = error * PROPORTION;
    
    return(int)((MAX_SPEED * (SPEED + correction)) << 24) | (int)((MAX_SPEED * (SPEED - correction)) << 8);

}