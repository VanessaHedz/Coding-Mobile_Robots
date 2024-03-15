/********************************************************
 *                                                      *
 *      state_machine_potentialFields.h            		*
 *                                                      *
 *		Student:	Vanessa Hernández                   *
 *		FI-UNAM				                        	*
 *		3-14-2024                                       *
 *                                                      *
 ********************************************************/

 // Student State Machine 
AdvanceAngle potential_fields(Raw observations, int dest, int intensity, float Mag_Advance, float max_angle, int num_sensors){

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

