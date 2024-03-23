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

/******************************************************** 
                         FUNCTIONS
*********************************************************/

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
AdvanceAngle Bug1_students(Raw observations, int dest, int intensity, int state, int *next_state, float Mag_Advance, float max_angle, int num_sensors, float angle_light, coord coord_robot, coord coord_dest,float distance1){

 AdvanceAngle gen_vector;
 int obs;
 int j;
 float left_side=0;
 float right_side=0;
 int value = 0;

 //Variables distance
 static float d1 = 0.0;
 static float d2 = 0.0;
 static float min_dist = 0.0;

 static bool reachedMinDistance = false; 

 //Variables
 static coord hit_coord; //Coordenates when the robot detect an obstacle for the first time
 static coord minDistance_coord; //Coordenate when the robot is in the min distance to the light source
 static coord coord1;
 static coord coord2;

 printf("\n\n *************************** Student Bug 1 ****************************************************\n");

 for(j=0;j<num_sensors/2;j++){
        right_side = observations.sensors[j] + right_side;
        //printf("right side sensor[%d] %f\n",j,observations.sensors[j]);
 }

 for(j=num_sensors/2;j<num_sensors;j++){
        left_side = observations.sensors[j] + left_side;
        //printf("left side sensor[%d] %f\n",j,observations.sensors[j]);
 }

 right_side = right_side/(num_sensors/2);
 left_side = left_side/(num_sensors/2);
 //printf("Average right side %f\n",right_side);
 //printf("Average left side %f\n",left_side);

 if( left_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 if( right_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 obs = value;

 //printf("intensity %d obstacles %d dest %d\n",intensity,obs,dest);

/*      PROGRAM       */

switch ( state ){

        case 0:
                if(intensity == 1)
                {
                        //The robot has reached the light source
                        gen_vector = generate_output(STOP,Mag_Advance,max_angle);
                        //printf("STOP");
                        *next_state = 0;
                }
                else
                {
                        gen_vector.angle = angle_light;
                        printf("\nThe robot is looking at the light source");
                        *next_state = 1;
                }
                break;
        
        case 1:
                if(obs == 0) //No obstacle
                {
                        gen_vector = generate_output(FORWARD,Mag_Advance,max_angle);
                        printf("\nNo obstacle");
                        *next_state = 0;       
                }
                else if(observations.sensors[7] <= 0.06 && observations.sensors[8] <= 0.06 ||
                        observations.sensors[10] <= 0.05 && observations.sensors[11] <= 0.05 ||
                        observations.sensors[5] <= 0.06 && observations.sensors[6] <= 0.06 )
                {
                        //Verifica si se encontró alguno de los choques que esperamos:
                                // Choque frontal (sensores [6,7,8,9])
                                // Choque frontal + derecha (sensores [8,9,10,11,12])
                                // Choque frontal + izquierda (sensores [4,5,6,7])
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);

                                        /* GUARDA LAS COORDENADAS */

                        //hit_coord = coord_robot; //Keep the coordenates when the robot detect an obsacle --------------------------------------------------------- !
                        hit_coord.xc = coord_robot.xc;
                        hit_coord.yc = coord_robot.yc;
                        hit_coord.anglec = 0.0;
                        
                        printf("\n*********** THE ROBOT HIT THE OBSTACLE ***********"); //test_students.dat 
                        printf("Coord X: %f ; Coord Y: %f",hit_coord.xc,hit_coord.yc);

                        *next_state = 2;
                }

                //If there is an obstacle to the left or right but the robot does not hit it, it ignores it.
                else
                {
                        gen_vector = generate_output(FORWARD,Mag_Advance,max_angle);
                        printf("\nNo obstacle");
                        *next_state = 0;
                }
                break;
        

        /* ------------------------- STATE MACHINES OBS IN FRONT -----------------------------------------------*/
        case 2:
                //Keep the obstacle to the right        (front sensors)                 (right sensors)
                if(observations.sensors[7] >= 0.09 && observations.sensors[8] >= 0.09 && observations.sensors[3] <= 0.05 && observations.sensors[2] <= 0.05)
                {
                        gen_vector = generate_output(STOP,Mag_Advance,max_angle);
                        *next_state = 3; //Go FORWARD
                }
                else
                {
                        gen_vector.angle = gen_vector.angle + 0.05;
                        printf("\n\n ************************************ ROTATING ************************************");
                        *next_state = 2;
                }
                break;
        
        case 3:
                printf("\nTHE ROBOT HAS LEFT THE HIT POINT");
                printf("\nRobot coords. XC = %f, YC = %f",coord_robot.xc,coord_robot.yc);
                printf("\nHIT POINTS. XC = %f, YC = %f",hit_coord.xc,hit_coord.yc);
                gen_vector = generate_output(FORWARD,Mag_Advance,max_angle);
                
                //distancia mínima entre el robot y la luz (primera distancia)
                d1 = distance1;
                coord1.xc = coord_robot.xc;
                coord1.yc = coord_robot.yc;

                *next_state = 4;
                break;


        
        /* ---------------------- STATE MACHINES FOR RODEATE AN OBSTACLE --------------------------------------- */
        case 4:
                if(( coord_robot.xc>=(hit_coord.xc - 0.03) && coord_robot.xc<=(hit_coord.xc + 0.03) ) && 
                   ( coord_robot.yc>=(hit_coord.yc - 0.03) && coord_robot.yc<=(hit_coord.yc + 0.03) ))
                {
                                //The robot is in the peripherials of the hitpoint :D
                        gen_vector = generate_output(STOP,Mag_Advance,max_angle);
                        printf("\n ************************ THE ROBOT HAS RETURNED **************************");
                        *next_state = 8;
                }
                else if(observations.sensors[3]>0.09)
                {
                        //TURN TO THE RIGHT
                        gen_vector = generate_output(FORWARD,Mag_Advance,max_angle); //Make 1 step
                        printf("\n[1]Robot coords. XC = %f, YC = %f",coord_robot.xc,coord_robot.yc);
                        printf("\nHIT POINTS. XC = %f, YC = %f",hit_coord.xc,hit_coord.yc);
                        *next_state = 5;
                }
                else
                {
                        gen_vector = generate_output(FORWARD,Mag_Advance,max_angle);
                        printf("\n[2]Robot coords. XC = %f, YC = %f",coord_robot.xc,coord_robot.yc);
                        printf("\nHIT POINTS. XC = %f, YC = %f",hit_coord.xc,hit_coord.yc);

                        d2 = distance1;
                        coord2.xc = coord_robot.xc;
                        coord2.yc = coord_robot.yc;

                        if(d1 < d2)
                        {
                                min_dist = d1;
                                minDistance_coord.xc = coord1.xc;
                                minDistance_coord.yc = coord1.yc;

                                printf("\nMIN DIST IN ( %f , %f )",minDistance_coord.xc,minDistance_coord.yc);
                                printf("\nd1 = %f, COORD 1 ( %f , %f )",d1,coord1.xc,coord1.yc);
                                printf("\nd2 = %f, COORD 2 ( %f , %f )",d2,coord2.xc,coord2.yc);
                        }
                        else
                        {
                                d1 = d2;
                                coord1.xc = coord2.xc;
                                coord1.yc = coord2.yc;

                                /*d2 = 0.0;
                                coord2.xc = 0.0;
                                coord2.yc = 0.0;*/

                                printf("\nd1 = %f, COORD 1 ( %f , %f )",d1,coord1.xc,coord1.yc);
                                printf("\nd2 = %f, COORD 2 ( %f , %f )",d2,coord2.xc,coord2.yc);
                        }

                        *next_state = 4;
                        /*
                        d2 = distance1;
                        //compara distancia 1 y 2:
                        if(!reachedMinDistance) //    ------------------------------------------------------------------------ !
                        {
                                if( d2 > d1 )
                                {
                                        //The robot has reached the minimum distance
                                        minDistance_coord.xc = coord_robot.xc;
                                        minDistance_coord.yc = coord_robot.yc;
                                        reachedMinDistance = true;
                                        printf("\nTHE ROBOT HAS REACHED THE MINIMUM DISTANCE");
                                        printf("\n[3]Robot coords. XC = %f, YC = %f",coord_robot.xc,coord_robot.yc);
                                        printf("\nMIN DISTANCE COORD. XC = %f, YC = %f",minDistance_coord.xc,minDistance_coord.yc);

                                        //*next_state = ;
                                }
                                else
                                {
                                        //Se re-inicializan las variables 
                                        d1 = d2;
                                        d2 = 0.0;

                                        *next_state = 4;
                                }
                        }
                        */
                }                
                break;
        
        case 5:
                gen_vector = generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 6;
                break;
        
        case 6:
                gen_vector = generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 7;
                break;
        
        case 7:
                gen_vector = generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 4;
                break;

        case 8:
                gen_vector = generate_output(STOP,Mag_Advance,max_angle);
                break;
}

 return gen_vector;

}
