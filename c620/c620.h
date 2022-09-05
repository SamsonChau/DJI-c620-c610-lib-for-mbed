#include "mbed.h"
#include <cstdint>
#include <stdint.h>

#ifndef C620_H 
#define C620_H

#define constrain(amt, low, high)                                              \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

typedef enum { 
    m3508_vel_loop = 0, 
    m3508_pos_loop 
} m3508_mode;

#define CMD_ID_0      0x1FF
#define CMD_ID_1      0x200
#define Motor_1_RevID 0x201
#define Motor_2_RevID 0x202
#define Motor_3_RevID 0x203
#define Motor_4_RevID 0x204
#define Motor_5_RevID 0x205
#define Motor_6_RevID 0x206
#define Motor_7_RevID 0x207
#define Motor_8_RevID 0x208

struct pid{
    float err               = 0;
    float P                 = 0;
    float I                 = 0;
    float D                 = 0;
    float kP                = 0;
    float kI                = 0;
    float kD                = 0;
    float prev_err          = 0;
    //float prev_prev_err     = 0;
    float deadband          = 0;
    float max_err           = 20000;
    float IntegraLimit      = 100000;    
};

class c620
{
    public:
        CAN*        can1;
        CANMessage  rxmsg;
        Timer       t_pid;
        Timer       t_canbus;
        //Change to m3508_init to replace constructor      
        void c620_init(CAN* _CAN);    
	
        //C620 function
        void c620_read();                       //read the can msg feeded back from c620
        void c620_calc();                       //cal the PID output and send the command
	    void c620_run();                        //sum up function of the previous one

        void CAN_Send(int16_t current1, int16_t current2, int16_t current3, int16_t current4,int16_t current5, int16_t current6, int16_t current7, int16_t current8);
        void set_velocity(int id, int speed);   // set the velocity in rpm of the motor (output shaft) 
        void set_position(int id, double pos);   // set the position in deg (output shaft)
        void set_mode(int mode);                // set it to position mode or velocity mode
        void set_device_num(int8_t num);           // set the number of device u use in the same can bus network
        int get_velocity(int id);               // get the velocity raw output from the encoder
        int get_position(int id);               // get the position raw output from the encoder
        int get_current(int id);                // get teh currebt raw output fron the encoder
        int get_temp(int id);                   // get temp

        void set_v_pid_param(int ID, float kp, float ki, float kd);             //set the velocity of PID constent
        void set_p_pid_param(int ID, float kp, float ki, float kd);             //set the velocity of PID constent
        void set_i_pid_param(int ID, float kp, float ki, float kd);             //set the velocity of PID constent

        //command buffer
        int16_t       required_current[8]     = {0};
        double       required_position[8]    = {0};
        double       required_velocity[8]    = {0};
        int16_t     motor_set_current[8]    = {0}; 
        
        //Read Value
        int16_t     read_current[8]         = {0};
        double      read_position[8]        = {0};      //value 0-8192 for c620, 0-8191 for c610
        int16_t     read_velocity[8]        = {0};      //value: rpm
        int8_t      temp[8]                 = {0};      // value of the temperature
        //Position Value
        int32_t     round_cnt[8]            = {0};      // round count of the motor
        uint16_t    last_pos[8]             = {0};      // storage of the last readed position
        uint16_t    start_pos [8]           = {0};      // The position of the first CAN message received, 
        long long    global_pos[8]           = {0};      // position of relative to the starting point
        bool        pos_init[8]             = {true};   // Set position to 0 flag
        //mode
        int8_t      loop_mode[8];           
        int8_t      lock[8]                 = {0};      //lock of the memory assess
        //pid parameter

        pid v_pid[8];
        pid p_pid[8];
        pid i_pid[8];   

        //motor and driver parameters
        float       dt                      = 0.001;    // Time period
        float       watch_dog               = 0;        // Time of not receiving can msg
        int         profile_velocity_CW[8]  = {0};      // 0 ~ +7200 max for c610
        int         profile_velocity_CCW[8] = {0};      // 0 ~ -7200 max for c610
        int         profile_torque_CW[8]    = {0};      // recomanded not over +50000
        int         profile_torque_CCW[8]   = {0};      // recomanded not less than -50000
        int         motor_max_current       = 7000;     // default
        float       gearatio                = 19.2;     // gear ratio of the motor, m3508 = 1:19, m2006 = 1:36   
        int         No_of_device            = 1;        // number of c620 you are using in the same network default 4, max. 8
        int         handle                  = 0;
        int8_t      can_detect[8]           = {0};      // enable state
    };
#endif 
