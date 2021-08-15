#include <webots/robot.h>

// Added a new include file
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <webots/connector.h>

#define TIME_STEP 2
int counter = 0;
int main(int argc, char **argv) {
 wb_robot_init();

 // get the motor devices
 WbDeviceTag left_motor = wb_robot_get_device("wheel_left");
 WbDeviceTag right_motor = wb_robot_get_device("wheel_right");
 WbDeviceTag lift_c = wb_robot_get_device("lift_center");
 WbDeviceTag lift_r = wb_robot_get_device("lift_right");
 WbDeviceTag lift_l = wb_robot_get_device("lift_left");
 // set the target position of the motors
 
 WbDeviceTag ds_l = wb_robot_get_device("distance_sens_left");
 WbDeviceTag ds_r= wb_robot_get_device("distance_sens_right");

  // WbDeviceTag puck_conn = wb_robot_get_device("connector_pp");
  WbDeviceTag vac_conn_c = wb_robot_get_device("vacuum_connector_center");
  WbDeviceTag vac_conn_r = wb_robot_get_device("vacuum_connector_right");
  WbDeviceTag vac_conn_l = wb_robot_get_device("vacuum_connector_left");
  wb_connector_enable_presence(vac_conn_c, 1);
  wb_connector_enable_presence(vac_conn_r, 1);
  wb_connector_enable_presence(vac_conn_l, 1);
  
  // printf("pres = %d\n", wb_connector_get_presence(vac_conn));
  // printf("presssss");
  
   
  wb_distance_sensor_enable(ds_l, TIME_STEP);
  wb_distance_sensor_enable(ds_r, TIME_STEP);
 
 while (wb_robot_step(TIME_STEP) != -1){
    double ds_l_val = wb_distance_sensor_get_value(ds_l);
   double ds_r_val = wb_distance_sensor_get_value(ds_r);
   
   // printf("ds l = %f\n", ds_l_val);
   // printf("ds l = %f\n", ds_r_val);
   // printf("pres = %d\n", wb_connector_get_presence(vac_conn));
   if (!wb_connector_is_locked(vac_conn_c)){
     if (wb_connector_get_presence(vac_conn_c) == 1)
     {
       wb_connector_lock(vac_conn_c);
       printf("LOCKED\n");
     }
   }
   
      if (!wb_connector_is_locked(vac_conn_r)){
     if (wb_connector_get_presence(vac_conn_r) == 1)
     {
       wb_connector_lock(vac_conn_r);
       printf("LOCKED\n");
     }
   }
   
      if (!wb_connector_is_locked(vac_conn_l)){
     if (wb_connector_get_presence(vac_conn_l) == 1)
     {
       wb_connector_lock(vac_conn_l);
       printf("LOCKED\n");
     }
   }
   if (counter == 1)
   {
     wb_motor_set_position(lift_c, 0);
     wb_motor_set_position(lift_l, 0);
     wb_motor_set_position(lift_r, 0);
   }
   if (counter == 200)
   {
   

 wb_motor_set_position(left_motor, 5.05);
 wb_motor_set_position(right_motor, 5.05);
      // wb_motor_set_position(left_motor, 30.0);
      
   }
   
   else if (counter == 550)
   {
   
   // if (ds_l_val < 1000.0)
   // {
     wb_motor_set_position(left_motor, -5.0);
     wb_motor_set_position(right_motor, -5.0);
   // }
   // else{
   // wb_motor_set_position(left_motor, 5.0);
   // }
   // wb_motor_set_position(right_motor, 2.0);
 // wb_motor_set_position(left_motor, 30.0);
 // wb_motor_set_position(right_motor, 30.0);
   }
      else if (counter == 950)
   {
     wb_motor_set_position(lift_c, 0.1);
     wb_motor_set_position(lift_l, 0.1);
     wb_motor_set_position(lift_r, 0.1);
   }
   
   
   
      // else if (counter == 1100)
   // {
 // wb_motor_set_position(left_motor, 40.0);
 // wb_motor_set_position(right_motor, 40.0);
   // }
 
 
 counter++;
 // printf("count = %d\n", counter);
 }

 wb_robot_cleanup();

 return 0;
}