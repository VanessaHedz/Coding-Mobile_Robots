/********************************************************
 *     v1: reactive_Students :  Follows a wall           *                                       *
 *                                                      *
 *      state_machine_student.h          		*
 *                                                      *
 *		Student:				*
 *		FI-UNAM					*
 *		2-15-2024                               *
 *                                                      *
 ********************************************************/

/*
Hacer que ignore la luz, pq no tiene que seguir la luz.
Hacer que vaya hacia adelante y, si encuentra una pared, que la ignore
*/
// Student State Machine 
AdvanceAngle reactive_students(Raw observations, int dest, int intensity, float Mag_Advance, float max_angle, int num_sensors){

 AdvanceAngle gen_vector;
 int obs;
 int j;
 float left_side=0;
 float right_side=0;
 int value = 0;
 static int step=0;

 step++;
 printf("\n\n **************** Student Reactive Behavior %d *********************\n",step);

 for(j=0;j<num_sensors/2;j++){
        right_side = observations.sensors[j] + right_side;
        printf("right side sensor[%d] %f\n",j,observations.sensors[j]);
 }

 for(j=num_sensors/2;j<num_sensors;j++){
        left_side = observations.sensors[j] + left_side;
        printf("left side sensor[%d] %f\n",j,observations.sensors[j]);
 }

 right_side = right_side/(num_sensors/2);
 left_side = left_side/(num_sensors/2);
 printf("Average right side %f\n",right_side);
 printf("Average left side %f\n",left_side);

 if( left_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 if( right_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 obs = value;
 printf("intensity %d obstacles %d dest %d\n",intensity,obs,dest);

 //No obstacle
 if(obs==0){
	 gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
	 printf("FORWARD");
 }

 //Obstacle to the RIGHT

 else if(obs==1){
	 gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
         printf("FORWARD");

 }

 //Obstacle to the LEFT

 else if(obs==2){
	 gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
         printf("FORWARD");

 }

 //Obstacle in FRONT 
 else if(obs==3){
	 gen_vector=generate_output(LEFTADVANCETWICE,Mag_Advance,max_angle);
         printf("LEFT");
 }
 
 return gen_vector;

}


// Student State Machine 
AdvanceAngle state_machine_students(Raw observations, int dest, int intensity, int state, int *next_state, float Mag_Advance, float max_angle, int num_sensors){

 AdvanceAngle gen_vector;
 int obs;
 int j;
 float left_side=0;
 float right_side=0;
 int value = 0;

 printf("\n\n **************** Student State Machine *********************\n");

 for(j=0;j<num_sensors/2;j++){
        right_side = observations.sensors[j] + right_side;
        printf("right side sensor[%d] %f\n",j,observations.sensors[j]);
 }

 for(j=num_sensors/2;j<num_sensors;j++){
        left_side = observations.sensors[j] + left_side;
        printf("left side sensor[%d] %f\n",j,observations.sensors[j]);
 }

 right_side = right_side/(num_sensors/2);
 left_side = left_side/(num_sensors/2);
 printf("Average right side %f\n",right_side);
 printf("Average left side %f\n",left_side);

 if( left_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 if( right_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 obs = value;
 printf("intensity %d obstacles %d dest %d\n",intensity,obs,dest);

/*      PROGRAM       */

switch ( state ){
        case 0:
                if(obs==0){
                        gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state=0; //Go back and check the sensors?
                        printf("Present state: %d STOP\n",state);
                }
                else{
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                        printf("Present State: %d STOP\n", state);

                        if (obs == 1){
                                // obtacle in the  right
                                *next_state = 1;
                        }
                        else if (obs == 2){
                                // obtacle in the left
                                *next_state = 3;
                        }
                        else if (obs == 3){
                                // obstacle in the front
                                *next_state = 5;
                        }
                }
                break;
        case 1:
                //BACKWARD obstacle to the RIGHT
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state=2;
                break;
        case 2:
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state=0; //Go check if there is another obstacle
                break;
        case 3:
                //BACKWARD obstacle to the LEFT
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state=4;
                break;
        case 4:
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state=0; //Go check if there is another obstacle
                break;
        case 5:
                //BACKWARD obstacle in FRONT
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                *next_state=6;
                break;
        case 6:
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_state=7;
                break;
        case 7:
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_state=8;
                break;
        case 8:
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state=9;
                break;
        case 9:
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state=0;
                break;
}
 return gen_vector;

}


//                                 (                                                                                                                                                    Actual position   light source position)
//type -> coord: xc, yx, angle;
AdvanceAngle Bug1_students(Raw observations, int dest, int intensity, int state, int *next_state, float Mag_Advance, float max_angle, int num_sensors, float angle_light, coord coord_robot, coord coord_dest){

 AdvanceAngle gen_vector;
 int obs;
 int j;
 float left_side=0;
 float right_side=0;
 int value = 0;

 //Variables
 float initial_position;
 float dif_x;
 float dif_y;
 float slope;
 double angle;

 printf("\n\n **************** Student Bug 1 *********************\n");

 for(j=0;j<num_sensors/2;j++){
        right_side = observations.sensors[j] + right_side;
        printf("right side sensor[%d] %f\n",j,observations.sensors[j]);
 }

 for(j=num_sensors/2;j<num_sensors;j++){
        left_side = observations.sensors[j] + left_side;
        printf("left side sensor[%d] %f\n",j,observations.sensors[j]);
 }

 right_side = right_side/(num_sensors/2);
 left_side = left_side/(num_sensors/2);
 printf("Average right side %f\n",right_side);
 printf("Average left side %f\n",left_side);

 if( left_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 if( right_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 obs = value;
 printf("intensity %d obstacles %d dest %d\n",intensity,obs,dest);

/*      PROGRAM       */

switch ( state ){
        case 0:
                if (intensity == 1)
                {
                       gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                       printf("STOP"); 
                       *next_state=0;
                }
                else
                {       
                        //Orientate the robot to the light source  
                        gen_vector.angle = angle_light;
                        printf("The robot is looking at the light source");
                        *next_state = 1;                      
                }

                break;
        case 1:
                if(obs == 0){
                        gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state=0; //Go back and check the sensors?
                        printf("Present state: %d FORWARD\n",state);
                }
                else
                {
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                        printf("Present State: %d STOP\n", state);

                        if (obs == 1){
                                // obtacle in the  right
                                *next_state = 2;
                        }
                        else if (obs == 2){
                                // obtacle in the left
                                *next_state = 2;
                        }
                        else if (obs == 3){
                                // obstacle in the front
                                *next_state = 3;
                        }
                }
                break;
        case 2:
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state=1;
                break;
        
        case 3:
                gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                *next_state=4;
                break;
        
        case 4:               
                //Guarda las coordenadas del punto en el que chocó 
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);

                if( obs==3 && obs==1 )
                {
                        *next_state=4;
                }
                else
                {
                        *next_state=5;
                }
                break;
        
        case 5:
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state=1;
                break;
}

 return gen_vector;

}
