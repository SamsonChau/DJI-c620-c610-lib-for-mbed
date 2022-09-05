#include "c620.h"
#include "mbed.h"

void c620::c620_init(CAN *_CAN) {
  can1 = _CAN;
  can1->frequency(1000000);
  handle = can1->filter(0x200, 0x700, CANStandard);
  can1->reset();
}
/////CAN send all motors/////
void c620::CAN_Send(int16_t current1, int16_t current2, int16_t current3,
                    int16_t current4, int16_t current5, int16_t current6,
                    int16_t current7, int16_t current8) {
  CANMessage TxMessage;
  TxMessage.id = CMD_ID_1;        // Common ID of c620
  TxMessage.format = CANStandard; // Standard Frame
  TxMessage.type = CANData;       // Data
  TxMessage.len = 8;              // Size = 8 byte

  // Motor Current Value Range from -16384 to 16384
  // Motor 1 Current Control
  TxMessage.data[0] = current1 >> 8; // data1;
  TxMessage.data[1] = current1;      // data2;
  // Motor 2 Current Control
  TxMessage.data[2] = current2 >> 8; // data3;
  TxMessage.data[3] = current2;      // data4;
  // Motor 3 Current Control
  TxMessage.data[4] = current3 >> 8; // data5;
  TxMessage.data[5] = current3;      // data6;
  // Motor 4 Current Control
  TxMessage.data[6] = current4 >> 8; // data7;
  TxMessage.data[7] = current4;      // data8;

  can1->write(TxMessage);

  if (No_of_device > 4) {
    CANMessage TxMessage2;
    TxMessage2.id = CMD_ID_0;        // Common ID of c620
    TxMessage2.format = CANStandard; // Standard Frame
    TxMessage2.type = CANData;       // Data
    TxMessage2.len = 8;              // Size = 8 byte

    // Motor Current Value Range from -16384 to 16384
    // Motor 5 Current Control
    TxMessage2.data[0] = current5 >> 8; // data1;
    TxMessage2.data[1] = current5;      // data2;
    // Motor 6 Current Control
    TxMessage2.data[2] = current6 >> 8; // data3;
    TxMessage2.data[3] = current6;      // data4;
    // Motor 7 Current Control
    TxMessage2.data[4] = current7 >> 8; // data5;
    TxMessage2.data[5] = current7;      // data6;
    // Motor 8 Current Control
    TxMessage2.data[6] = current8 >> 8; // data7;
    TxMessage2.data[7] = current8;      // data8;

    can1->write(TxMessage2);
  }
}

/////CAN Send is Triggered when Receiving CAN Message/////
void c620::c620_read() {
  int16_t motor_index = 0x00;
  watch_dog = (float)std::chrono::duration_cast<std::chrono::seconds>(
                  t_canbus.elapsed_time())
                  .count();
  if (can1->read(rxmsg, handle)) {
    t_canbus.stop();
    uint16_t actual_local_position =
        (uint16_t)(rxmsg.data[0] << 8) | rxmsg.data[1];
    int16_t actual_velocity = (int16_t)(rxmsg.data[2] << 8) | rxmsg.data[3];
    int16_t actual_current = (int16_t)(rxmsg.data[4] << 8) | rxmsg.data[5];
    int8_t actual_tempture = (int8_t)(rxmsg.data[6]);
    motor_index = rxmsg.id - 0x201;
    /*
    switch (rxmsg.id) {
    case Motor_1_RevID: {
      motor_index = 0;
      break;
    }
    case Motor_2_RevID: {
      motor_index = 1;
      break;
    }
    case Motor_3_RevID: {
      motor_index = 2;
      break;
    }
    case Motor_4_RevID: {
      motor_index = 3;
      break;
    }
    case Motor_5_RevID: {
      motor_index = 4;
      break;
    }
    case Motor_6_RevID: {
      motor_index = 5;
      break;
    }
    case Motor_7_RevID: {
      motor_index = 6;
      break;
    }
    case Motor_8_RevID: {
      motor_index = 7;
      break;
    }
    default:
      return;
    }*/

    // lock the access of the array to prevent crash
    lock[motor_index] = true;
    read_velocity[motor_index] = actual_velocity;
    read_position[motor_index] =
        (double)((double)(actual_local_position)*360 / 8192);
    read_current[motor_index] = actual_current;
    temp[motor_index] = actual_tempture;
    // To record the starting position
    if (pos_init[motor_index]) {
      start_pos[motor_index] = read_position[motor_index];
      round_cnt[motor_index] = 0;
      pos_init[motor_index] = false;
    }
    // Accumulate the round count
    if (read_position[motor_index] - last_pos[motor_index] > 180) {
      round_cnt[motor_index]--;
    } else if (read_position[motor_index] - last_pos[motor_index] < -180) {
      round_cnt[motor_index]++;
    } else {
      // printf("%d\n",round_cnt[motor_index]);
    }

    // Multi-turn global position
    global_pos[motor_index] =
        (round_cnt[motor_index] * 360 + (read_position[motor_index])) -
        (start_pos[motor_index]);
    // After using the value read_position, assign its value to last_pos
    last_pos[motor_index] = read_position[motor_index]; // used for round count
    // Release the lock
    lock[motor_index] = false;
    t_canbus.reset();
    t_canbus.start();
  }
  if (watch_dog < 1) {
    can_detect[motor_index] = 1;
  } else {
    for (int i = 0; i < 8; i++) {
      can_detect[i] = 0;
    }
  }
}

void c620::c620_calc() // Execution frequency 100Hz
{
  t_pid.stop();
  // dt =
  // (float)std::chrono::duration_cast<std::chrono::microseconds>(t_pid.elapsed_time()).count()
  // / 1000000;
  dt = 0.001;
  for (int motor_index = 0; motor_index < 8; motor_index++) {

    if (loop_mode[motor_index] == m3508_pos_loop) {
      p_pid[motor_index].err =
          required_position[motor_index] - global_pos[motor_index];

      // contrain the error maximum and lower value

      p_pid[motor_index].err =
          constrain(p_pid[motor_index].err, -p_pid[motor_index].max_err,
                    p_pid[motor_index].max_err);
      if (p_pid[motor_index].err != 0 &&
          abs(p_pid[motor_index].err) < p_pid[motor_index].deadband) {
        p_pid[motor_index].err = 0;
      }
      // position loop
      p_pid[motor_index].P = p_pid[motor_index].kP * p_pid[motor_index].err;
      p_pid[motor_index].I +=
          p_pid[motor_index].kI * p_pid[motor_index].err * dt;
      p_pid[motor_index].D =
          p_pid[motor_index].kD *
          (p_pid[motor_index].err - p_pid[motor_index].prev_err) / dt; // dt;

      // contrain the I term;
      // p_pid[motor_index].I =
      // constrain(p_pid[motor_index].I,-p_pid[motor_index].IntegraLimit,p_pid[motor_index].IntegraLimit);

      // Store previous error
      p_pid[motor_index].prev_err = p_pid[motor_index].err;

      // Sum up the terms -> require velocity
      required_velocity[motor_index] =
          p_pid[motor_index].P + p_pid[motor_index].I + p_pid[motor_index].D;
      // required_velocity[motor_index] =
      // constrain(required_velocity[motor_index],
      // profile_velocity_CW[motor_index], profile_velocity_CCW[motor_index]);
      // //max 7200 for 2006
    }

    v_pid[motor_index].P =
        v_pid[motor_index].kP *
        (required_velocity[motor_index] - (double)read_velocity[motor_index]);
    v_pid[motor_index].I += v_pid[motor_index].kI * v_pid[motor_index].P * dt;
    v_pid[motor_index].D =
        v_pid[motor_index].kD *
        (v_pid[motor_index].P - v_pid[motor_index].prev_err) / dt;
    // contrain the I term;
    // v_pid[motor_index].err =
    // constrain(v_pid[motor_index].I,-v_pid[motor_index].IntegraLimit,v_pid[motor_index].IntegraLimit);

    // Store old velocity error
    v_pid[motor_index].prev_err = v_pid[motor_index].P;

    // Calculate output current
    required_current[motor_index] =
        floor((v_pid[motor_index].P * v_pid[motor_index].kP +
               v_pid[motor_index].I * v_pid[motor_index].kI +
               v_pid[motor_index].D * v_pid[motor_index].kD));
    required_current[motor_index] =
        constrain(required_current[motor_index], profile_torque_CW[motor_index],
                  profile_torque_CCW[motor_index]);

    // current loop
    i_pid[motor_index].P =
        required_current[motor_index] - read_current[motor_index];
    i_pid[motor_index].I += i_pid[motor_index].P * dt;
    i_pid[motor_index].D =
        (i_pid[motor_index].P - i_pid[motor_index].prev_err) / dt;

    // Store the error
    i_pid[motor_index].prev_err = i_pid[motor_index].P;

    // Calculate output current
    motor_set_current[motor_index] =
        (int16_t)floor((i_pid[motor_index].P * i_pid[motor_index].kP +
                        i_pid[motor_index].I * i_pid[motor_index].kI +
                        i_pid[motor_index].D * i_pid[motor_index].kD));
    // motor_set_current[motor_index] = required_velocity[motor_index];
  }

  // Set current constaint according to m3508 & c620 official manual (m3508
  // rated 10A, c620 rated 0 - 20A) Max 16384 -> 16384/2 = 8192 = 10 A, limit to
  // 7000 = 8.5A for testing
  for (int i = 0; i < 8; i++) {
    motor_set_current[i] =
        constrain(motor_set_current[i], -motor_max_current, motor_max_current);
  }
  CAN_Send(motor_set_current[0], motor_set_current[1], motor_set_current[2],
           motor_set_current[3], motor_set_current[4], motor_set_current[5],
           motor_set_current[6], motor_set_current[7]);
  // CAN_Send(motor_out[0], motor_out[1], motor_out[2], motor_out[3]);
  t_pid.reset();
  t_pid.start();
}

void c620::c620_run() {
  for (int i = 0; i < No_of_device; i++) {
    c620_read();
  }
  c620_calc();
}
void c620::set_device_num(int8_t num) {
  if (num > 0) {
    No_of_device = num;
  }
}
void c620::set_velocity(int motor_index, int speed) {
  loop_mode[motor_index] = m3508_vel_loop;
  required_velocity[motor_index] = speed * gearatio;
}
// pos loop input degree value (float)
void c620::set_position(int motor_index, double pos) {
  loop_mode[motor_index] = m3508_pos_loop;
  required_position[motor_index] = pos * gearatio;
}
void c620::set_v_pid_param(int ID, float kp, float ki, float kd) {
  v_pid[ID].kP = kp;
  v_pid[ID].kI = ki;
  v_pid[ID].kD = kd;
}
void c620::set_p_pid_param(int ID, float kp, float ki, float kd) {
  p_pid[ID].kP = kp;
  p_pid[ID].kI = ki;
  p_pid[ID].kD = kd;
}
void c620::set_i_pid_param(int ID, float kp, float ki, float kd) {
  i_pid[ID].kP = kp;
  i_pid[ID].kI = ki;
  i_pid[ID].kD = kd;
}
int c620::get_velocity(int motor_index) {
  if (lock[motor_index]) {
    return 0;
  } else {
    return (int)read_velocity[motor_index];
  }
}
int c620::get_current(int motor_index) {
  if (lock[motor_index]) {
    return 0;
  } else {
    return read_current[motor_index];
  }
}
int c620::get_temp(int motor_index) {
  if (lock[motor_index]) {
    return 0;
  } else {
    return (int)temp[motor_index];
  }
}
int c620::get_position(int motor_index) {
  if (lock[motor_index]) {
    return 0;
  } else {
    return (int)read_position[motor_index];
  }
}
