/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_avoidance.h			*
 *                                                      *
 *		Jesus Savage				*
 *		FI-UNAM					*
 *		6-1-2015                                *
 *                                                      *
 ********************************************************/


// Function to get next state
AdvanceAngle state_machine_avoidance(Raw observations, int dest, float distance, float distance_G, float *next_distance, int num_sensors, coord coord_robot, coord coord_per, coord *coord_perd,coord coord_cer, coord *next_coord_cer, int obs_enc, int *next_obs_enc, int a, int *next_a, int state, int *next_state,float Mag_Advance,float max_angle){

 AdvanceAngle gen_vector;
 int obs;
 int j;
 float left_side=0;
 float right_side=0;
 int value = 0;
 //int obs_enc=0;

 for(j=0;j<num_sensors/2;j++){
	right_side = observations.sensors[j] + right_side; 
#ifdef DEBUG
	printf("right side sensor[%d] %f\n",j,observations.sensors[j]);
#endif
 }

 for(j=num_sensors/2;j<num_sensors;j++){
	left_side = observations.sensors[j] + left_side; 
#ifdef DEBUG
	printf("left side sensor[%d] %f\n",j,observations.sensors[j]);
#endif
 }

 right_side = right_side/(num_sensors/2);
 left_side = left_side/(num_sensors/2);
#ifdef DEBUG
 printf("Average right side %f\n",right_side);
 printf("Average left side %f\n",left_side);
#endif

 if( left_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 if( right_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 obs = value;
#ifdef DEBUG
 printf("obs %d\n",obs);
#endif
 //coord_per=coord_robot;

 switch ( state ) {

        case 0:
                if (obs == 0 && obs_enc!=1){
			// there is not obstacle
			*next_a=0;
			*next_coord_cer=coord_robot;
			*next_distance=distance;
                        if (dest == 0){
                                // go right
                                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                                *next_state = 17;

                                printf("Present State: %d RIGHT\n", state);
		         }
		         else if (dest == 1){
                                // go left
                                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                                *next_state = 18;

                                printf("Present State: %d LEFT\n", state);
		         }
		         else if (dest == 2){
                                // go right single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                *next_state = 17;

                                printf("Present State: %d FORWARD\n", state);
		         }
		         else if (dest == 3){
                                // go left single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                *next_state = 18;

                                printf("Present State: %d FORWARD\n", state);
                 	 }
                
                }
                else{
			
			*next_obs_enc=1;
                        
			if(a!=1){
				*coord_perd=coord_robot;				
				*next_a=1;
			}
			
                        else if (obs == 1){
                                // obtacle in the right
				gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        	
                                *next_state = 2;
                        }                       
			else if (obs == 2){
                                // obstacle in the left
				gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                        	
                                *next_state = 0;
                        }
                        else if (obs == 3){
                                // obstacle in the front
                                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                        	
                                *next_state = 16;
                        }
			else if (obs == 0){
                                // obstacle lost
				gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                        	
                                *next_state = 14;
                        }

			
			
                }

                break;

        case 1: // Segunda vuelta
		if (obs == 0 && obs_enc!=1){
                        gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                        *next_state = 0;
                }
                else{
			
			*next_obs_enc=1;
                        
			if (obs == 1){
                                // obtacle in the right
				gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        	
                                *next_state = 5;
                        }
			else if (obs == 2){
                                // obstacle in the left
				gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                        	
                                *next_state = 1;
                        }
                        else if (obs == 3){
                                // obstacle in the front
                                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                        	
                                *next_state = 10;
                        }
			else if (obs == 0){
                                // obstacle lost
				gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                        	
                                *next_state = 7;
                        }
			
                }                
		
                break;

        case 2: // caso especial avanzó
		
		if(distance < distance_G) {
			*next_distance=distance;
			*next_coord_cer=coord_robot;
		}
	
            	if ((coord_per.xc/coord_robot.xc)>=0.95 && (coord_per.xc/coord_robot.xc)<=1.05 && (coord_per.yc/coord_robot.yc)>=0.95 && (coord_per.yc/coord_robot.yc)<=1.05) {
	                
			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
	        	
	                *next_state = 1;
        	}
		else if (obs == 1){
                        // obtacle in the right
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                        *next_state = 3;
                }
		else if (obs == 0){
                        // obtacle LOST
			gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                        *next_state = 4;
                }   
		else if (obs == 3){
                        // obstacle in the front
                        gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 16;
                }		
		else {
                        // obtacle in the right
			gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                        *next_state = 4;
                }

                break;

        case 3: // caso especial 3
				
            	if (obs == 1){
                        // obtacle in the right
			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 2;
                }
		else if (obs == 0){
                        // obtacle in the right
			gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                        *next_state = 4;
                }   
		
                break;

        case 4: // caso especial 3
				
            	if (obs == 1){
                        // obtacle in the right
			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 2;
                }
		else if (obs == 0){
                        // obtacle in the right
			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 2;
                }   
		
                break;


        case 5: // caso especial SEGUNDA VUELTA  avanzó
				
            	if ((coord_cer.xc/coord_robot.xc)>=0.8 && (coord_cer.xc/coord_robot.xc)<=1.2 && (coord_cer.yc/coord_robot.yc)>=0.8 && (coord_cer.yc/coord_robot.yc)<=1.2) {
		                // obtacle in the right
				gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
		        	
		                *next_state = 11;
                	}
		
                else if (obs == 1){
                        // obtacle in the right
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                        *next_state = 6;
                }
		else if (obs == 0){
                        // obtacle LOST
			gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                        *next_state = 7;
                }
		else if (obs == 3){
                        // obstacle in the front
                        gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 10;
                }  
		else {
                        // obtacle in the right
			gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                        *next_state = 7;
                }
		
                break;

        case 6: // caso especial 6
				
            	if (obs == 1){
                        // obtacle in the right
			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 5;
                }
		else if (obs == 0){
                        // obtacle in the right
			gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                        *next_state = 7;
                }   
		
                break;

        case 7: // caso especial 7
				
            	if (obs == 1){
                        // obtacle in the right
			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 5;
                }
		else if (obs == 0){
                        // obtacle in the right
			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 5;
                }   
		
                break;

	case 8: 
		
		if (obs == 1){
                        
			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                	
                        *next_state = 5;
                }
		else if (obs == 0){
                        
			gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                	
                        *next_state = 1;
                } 
		else {
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 1;
                }                
                break;
        case 9: // Backward, obstacle in LEFT

		if (obs == 3){
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 9;
                }
		else if (obs == 2){
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 9;
                }
		else if (obs == 1){
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 8;
                }
		else {
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 1;
                }                
                break;

        case 10: 
		if (obs == 3){
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 9;
                }
		else if (obs == 2){
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 9;
                }
		else if (obs == 1){
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 8;
                } 
		else {
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 0;
                }                
                break;
	

        case 11: // // check destination
		 if (dest == 0){
                                
                                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                                *next_state = 12;

                                
                 }
                 else if (dest == 1){
                                
                                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                                *next_state = 13;

                                
                 }
                 else if (dest == 2){
                                
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                *next_state = 19;

                                
                 }
                 else if (dest == 3){
                                
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                *next_state = 20;
				

                                
                 }
		 else{
				gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                *next_state = 19;	
		 }

                break;
        case 12: 
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                *next_state = 11;
                break;
        case 13: 
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                printf("Present State: %d TURN 1 LEFT\n", state);
                *next_state = 11;
                break;


        case 14: 
		
		if (obs == 1){
                        
			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                	
                        *next_state = 2;
                }
		else if (obs == 0){
                        
			gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                	
                        *next_state = 0;
                } 
		else {
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 0;
                }                
                break;

        case 15: // Backward, obstacle in LEFT

		if (obs == 3){
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 15;
                }
		else if (obs == 2){
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 15;
                }
		else if (obs == 1){
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 14;
                }
		else {
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 0;
                }                
                break;

        case 16: 
		if (obs == 3){
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 15;
                }
		else if (obs == 2){
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 15;
                }
		else if (obs == 1){
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 14;
                } 
		else {
                        
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	
                        *next_state = 0;
                }                
                break;

        case 17: // Backward, obstacle in front
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                *next_state = 0;
                break;
        case 18: /// Left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                printf("Present State: %d TURN 1 LEFT\n", state);
                *next_state = 0;
                break;

	case 19: 
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_obs_enc=0;		                
		*next_state = 0;
                break;
        case 20: 
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_obs_enc=0;		                
		*next_state = 0;
                break;


 }

 //printf("Next State: %d\n", *next_state);
 return gen_vector;

}
                 

