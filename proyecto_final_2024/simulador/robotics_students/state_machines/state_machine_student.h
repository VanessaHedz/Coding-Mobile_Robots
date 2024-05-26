/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_student.h          		*
 *                                                      *
 *		Student:				*
 *		FI-UNAM					*
 *		2-15-2024                               *
 *                                                      *
 ********************************************************/
#include <vector>

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

 if (intensity == 1){
	// Constants STOP, TURN RIGHT, ETC, are defined in ../utilities/constants.h
	// generate_output function in ../utilities/utilities.h
	gen_vector=generate_output(STOP,Mag_Advance,max_angle);
        printf("STOP\n");
	printf("\n **************** Reached light source ******************************\n");
 }
 else if (obs == 0){
	// There is not obstacle
        //gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
	//printf("FORWARD\n");

	if (dest == 0){
                // go right twice
		gen_vector=generate_output(RIGHTADVANCETWICE,Mag_Advance,max_angle);
                printf("TURN RIGHT TWICE\n");
        }
        else if (dest == 1){
                // go left twice
                gen_vector=generate_output(LEFTADVANCETWICE,Mag_Advance,max_angle);
                printf("TURN LEFT TWICE\n");
        }
        else if (dest == 2){
                 // go right
                 gen_vector=generate_output(RIGHTADVANCE,Mag_Advance,max_angle);
                 printf("TURN RIGHT\n");
        }
        else if (dest == 3){
                // go left
                gen_vector=generate_output(LEFTADVANCE,Mag_Advance,max_angle);
                printf("TURN LEFT\n");
        }
 }
 else if (obs == 1){
        // Obtacle in the right
        gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
	printf("TURN LEFT\n");
 }
 else if (obs == 2){
        // obtacle in the left
	gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
	printf("TURN RIGHT\n");
 }
 else if (obs == 3){
	// obstacle in the front
        gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
	printf("TURN LEFT\n");
 }


 return gen_vector;

}











AdvanceAngle minimosCuadrados(Raw observations, int dest, int intensity, int state, int *next_state,float Mag_Advance,float max_angle, coord coord_robot){

 AdvanceAngle gen_vector;

 //Variables
 
 static float x_real = 0.061733;  //Value of the 9th sensor in de position: (0.595,0.5375)
 //static std::vector<float> X;       //Vector X of the values with noise from the 9th sensor.
 std::vector<float> X;
 static int i=0;                  //Counter for the 500 observations
 static float angle;

 printf("inimos Cuadrados. STATE %d", state);

 switch( state )
 {
    printf("\nVER SI ESTA ENTRANDO A LOS ESTADOS. STATE %d", state);
    case 0:
        //Move the robot from de initial position to the position (0.595,0.5375)
        /*
        if(( coord_robot.xc >= 0.59 && coord_robot.xc <= 0.60 ) && ( coord_robot.yc >= 0.53 && coord_robot.yc <= 0.54 ))
        {
            gen_vector = generate_output(STOP,Mag_Advance,max_angle);
            *next_state = 1;
        }           

        angle = atan((0.5375 - coord_robot.xc)/(0.595 - coord_robot.yc));

        gen_vector.angle = angle;
        gen_vector.distance = gen_vector.distance + 0.01;
        *next_state = 0;
        */
	gen_vector = generate_output(STOP,Mag_Advance,max_angle);
        *next_state = 1;
        break;
    
    case 1:
        if(i=500) //Duplicar el valor
        {
            //STOP the count 
            *next_state = 3;
            break;
        }

        //Save the value of the 9th sensor in the list
        X.push_back(observations.sensors[9]);
        printf("\n SAVING VALUE...");
        printf("value %f\n",X[i]);
	gen_vector = generate_output(STOP,Mag_Advance,max_angle);
        *next_state = 1;
	i++;
        break;
    
    case 2:
        i++;
        *next_state = 1; 
        break;
    
    case 3:
        //PRINT THE OBSERVATIONS
        printf("**** VALUES OF X ****\n");
        for(int j=0; j<500; j++)
        {
            printf("\n %f", X[j]);
        }
	gen_vector = generate_output(STOP,Mag_Advance,max_angle);
        break;


        /*
            En el peor de los casos:
                - Hacer una copia de la lista.
                - Obtener cada elemento de la lista con .pop() y guardar ese valor en una variable.
                - Usar la variable y reescribirla con el siguiente elemento de la variable con el método .pop()
        */
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

 switch ( state ) {

        case 0:
                if (intensity == 1){
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                        *next_state = 1;

                        printf("Present State: %d STOP\n", state);
			printf("\n **************** Reached light source ******************************\n");
                }
                else{

			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 1;

                        printf("Present State: %d FORWARD\n", state);
                }

                break;

        case 1:
                if (obs == 0){
			// There is not obstacle
                        gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 1;

                        printf("Present State: %d FORWARD\n", state);
                }
                else{
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                        printf("Present State: %d STOP\n", state);

                        if (obs == 1){
                                // obtacle in the  right
                                *next_state = 2;
                        }
                        else if (obs == 2){
                                // obtacle in the left
                                *next_state = 4;
                        }
                        else if (obs == 3){
				// obstacle in the front
                                *next_state = 7;
                        }
                }

                break;

        case 2: // Backward, obstacle in the right
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 3;

		printf("Present State: %d BACKWARD, obstacle right\n", state);
                break;

        case 3: // right turn
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 0;

		printf("Present State: %d TURN LEFT\n", state);
                break;

        case 4: // Backward, obstacle in the left
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                *next_state = 5;

		printf("Present State: %d BACKWARD, obstacle left\n", state);
                break;

        case 5: // left turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 0;

		printf("Present State: %d TURN RIGTH\n", state);
                break;

        case 6: // Backward, obstacle in front
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_state = 7;

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;

	case 7: /// Left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_state = 8;

		printf("Present State: %d TURN 1 LEFT\n", state);
                break;

        case 8:// Left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_state = 9;

		printf("Present State: %d TURN 2 LEFT\n", state);
                break;

        case 9: // Forward
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 10;

                printf("Present State: %d 1 FORWARD\n", state);
                break;

        case 10: // Forward
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 0;

                printf("Present State: %d 2 FORWARD\n", state);
                break;

	case 11: // Right turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 12;

                printf("Present State: %d turn 1 RIGHT\n", state);
                break;

        case 12: // Right turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 0;

                printf("Present State: %d turn 2 RIGHT\n", state);
                break;


        case 13: // // check destination
		 if (dest == 0){
                                // go right
                                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                                *next_state = 5;

                                printf("Present State: %d RIGHT\n", state);
                 }
                 else if (dest == 1){
                                // go left
                                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                                *next_state = 3;

                                printf("Present State: %d LEFT\n", state);
                 }
                 else if (dest == 2){
                                // go right single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                *next_state = 5;

                                printf("Present State: %d FORWARD\n", state);
                 }
                 else if (dest == 3){
                                // go left single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                *next_state = 3;

                                printf("Present State: %d FORWARD\n", state);
                 }
                break;

	default:
		printf("State %d not defined used ", state);
                gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                next_state = 0;
                break;

                
 }

 return gen_vector;

}


// Student BUG 1
AdvanceAngle bug1_students(Raw observations, int dest, coord coord_robot, int intensity, int state, int *next_state, float Mag_Advance, float max_angle, int num_sensors){

 AdvanceAngle gen_vector;
 int obs;
 int a; //variable por si encuentra un obstaculo
 int j;
 float left_side=0;
 float right_side=0;
 int value = 0;
 coord coord_obs;
 coord coord_per;
 //float x =coord_robot.xc;
 //float y =coord_robot.yc;
 printf("\n\n **************** Student State Machine BUG I *********************\n");

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
 
 if(a!=1) a=0;
 obs = value;
 printf("intensity %d obstacles %d dest %d\n",intensity,obs,dest);

 switch ( state ) {

        case 0:
                if (intensity == 1){
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                        *next_state = 1;

                        printf("Present State: %d STOP\n", state);
			printf("\n **************** Reached light source ******************************\n");
                }
                else{

			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 1;

                        printf("Present State: %d FORWARD\n", state);
                }

                break;

        case 1:
                if (obs == 0 && a == 0){
			// There is not obstacle
                        gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 20;

                        printf("Present State: %d FORWARD\n", state);
                }
                else{
			a=1;
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                        printf("Present State: %d STOP\n", state);

                        if (obs == 1){
                                // obtacle in the  right
                                *next_state = 7;
                        }
                        else if (obs == 2){
                                // obtacle in the left
                                *next_state = 4;
                        }
                        else if (obs == 3){
				// obstacle in the front
                                *next_state = 2;
                        }
                        else if (obs == 0){
                                // obtacle lost
                                *next_state = 5;
                        }
                }

                break;

        case 2: // Backward, obstacle in the front
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                *next_state = 3;

		printf("Present State: %d BACKWARD, obstacle right\n", state);
                break;

        case 3: // left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_state = 6;

		printf("Present State: %d TURN LEFT\n", state);
                break;

        case 4: // Backward, obstacle in the left
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                *next_state = 3;

		printf("Present State: %d BACKWARD, obstacle left\n", state);
                break;

        case 5: // right turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 9;

		printf("Present State: %d TURN RIGHT\n", state);
                break;

        case 6: // Backward, obstacle in front
		//coord_obs = coord_robot;
		//a=1;                
		gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 1;

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;
        case 7: // Backward, obstacle in front
		coord_per = coord_robot;                
		gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 8;

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;
        case 8: // Backward, obstacle in front
		if(coord_per.xc==coord_robot.xc){
                	gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                	*next_state = 6;
		}
		else{
			//gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                	*next_state = 6;
		}

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;

        case 9: // Backward, obstacle in front
		//coord_obs = coord_robot;                
		gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 1;

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;

        case 10: // Backward, obstacle in front
		//coord_obs = coord_robot;                
		gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 11;

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;
        case 11: // Backward, obstacle in front
		//coord_obs = coord_robot;                
		gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 12;

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;
        case 12: // Backward, obstacle in front
		//coord_obs = coord_robot;                
		gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 13;

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;
        case 13: // Backward, obstacle in front
		//coord_obs = coord_robot;                
		gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 14;

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;
        case 14: // Backward, obstacle in front
		if(obs == 3){
                	*next_state = 7;
		}
		else if(obs == 1){
                                // obtacle in the right
			gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                        *next_state = 8;
                }
		else{
			*next_state = 9;
		}

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;
        case 15: // Backward, obstacle in front              
		gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                *next_state = 7;

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;

        case 18: // Backward, obstacle in front
		if(coord_robot.xc == coord_obs.xc && coord_robot.yc == coord_obs.yc){
			gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                	*next_state = 6;
		}
		else{
			*next_state = 6;
		}

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;
	

        case 20: // // check destination
		 if (dest == 0){
                                // go right
                                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                                *next_state = 5; //Right

                                printf("Present State: %d RIGHT\n", state);
                 }
                 else if (dest == 1){
                                // go left
                                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                                *next_state = 3;

                                printf("Present State: %d LEFT\n", state);
                 }
                 else if (dest == 2){
                                // go right single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                *next_state = 5;

                                printf("Present State: %d FORWARD\n", state);
                 }
                 else if (dest == 3){
                                // go left single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                *next_state = 3;

                                printf("Present State: %d FORWARD\n", state);
                 }
                break;

	default:
		printf("State %d not defined used ", state);
                gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                next_state = 0;
                break;

                
 }

 return gen_vector;

}




// Campos potenciales 2
AdvanceAngle Campos_P3(Raw observations, float dest_angle, float distance, coord coord_robot, int intensity, int a, int *next_a, float xq, float *next_xq, float yq, float *next_yq, float Mag_Advance, float max_angle, int num_sensors, float angle_start, float angle_end, int dest, int obs){

 AdvanceAngle gen_vector;
 //int obs;
 int j,l=0,m=0; //variable por si encuentra un obstaculo
 float d;
 float angulo, angulo1;
 float left_side=0;
 float right_side=0;
 int state;
 int next_state;
 int value = 0;
 float e1 = 100;
 float x,xd,xo,y,yd,yo,xq1,yq1;
 int caso=5;
 float Fx,Fy,fx,fy,fx1,fy1, dobs, FRx,FRy, FAx,FAy;
 float d0=5.0, n = 0.05, obst_angle=3.14, mod;
 float r1,r2,r3,r4,modr,ks,ns,k1;
 coord coord_obs;
 coord coord_cer;
 coord next_coord_cer;
 x =coord_robot.xc;
 y =coord_robot.yc;
 
 d=distance;

 //dobs=10;

 k1=(abs(angle_start)+abs(angle_end));
 ns= num_sensors;
 ks=k1/ns;
 printf("k1 k: %f, %f \n",k1, ks);


 for(j=0;j<num_sensors;j++){

	if(observations.sensors[j] < 0.1 && (observations.sensors[j] < observations.sensors[j-1])){
		dobs=observations.sensors[j];
		obst_angle=angle_start+(j*ks);
		l=1;
		printf("Angulos1: %f, %f, %f \n", obst_angle, angle_start, angle_end);
		printf("j k: %d, %f \n", j, ks);
		xo=dobs*cos(obst_angle); 
		yo=dobs*sin(obst_angle); 
		
	}
	else if(l!=1){
		printf("Anguloss: %f \n", obst_angle);
		xo=0; 
		yo=0; 

	}	
 }

 
 next_coord_cer=coord_robot;
 xd=d*cos(dest_angle); 
 yd=d*sin(dest_angle);

 printf("xobs, yobs: %f, %f \n", xo, yo);

 if (a!=1) {

 	xq=0;
	yq=0;
	*next_a=1;
 
 }

 if (intensity == 1){
         gen_vector=generate_output(STOP,Mag_Advance,max_angle);
         printf("Present State: STOP\n");		
 }
 else{

 	 //x=xq;
	 //y=yq; 

	 printf("\n\n **************** Student reactive Campos *********************\n");

	 //U=(1/2)*e1*(((x-xd)*(x-xd))+((y-yd)*(y-yd)));

	 modr= sqrt(((x-xo)*(x-xo))+((y-yo)*(y-yo)));

	 r1 = ((1/modr)-(1/d0));
	 r2 = (1/(modr*modr));
	 r3 = ((x-xo)/modr);
	 r4 = ((y-yo)/modr);

	 printf("rs: %f, %f, %f, %f \n", r1,r2,r3,r4);
	 printf("xobs, yobs: %f, %f \n", d, dest_angle);
	 FRx=(-n)*r1*r2*r3;
	 FRy=(-n)*r1*r2*r4;

	 if(modr>d0 | (xo==0 && l!=1)){

	 	FRx=0;
		FRy=0;
	 }
	 else if(xo==0){

		m=1;

	 }

	 printf("Fuerza de repulsion: %f , %f \n", FRx,FRy);

	 //F=e1*((x-xd),(y-yd));
	 FAx= e1*(x-xd);
	 FAy= e1*(y-yd);

	 Fx= FRx+FAx;
	 Fy= FRy+FAy;

	 mod = (sqrt(((Fx)*(Fx))+((Fy)*(Fy))));

	 fx=Fx/mod;
	 fy=Fy/mod;

	 printf("Fuerza de atraccion: %f , %f \n", FAx,FAy);
	 printf("Fuerza total: %f , %f \n", Fx,Fy);
	 printf("Fuerza total m: %f , %f, %f \n", fx,fy, mod);

	 *next_xq=x-(Mag_Advance*fx);
	 *next_yq=y-(Mag_Advance*fy);

	 xq=x-(Mag_Advance*fx);
	 yq=y-(Mag_Advance*fy);

	 printf("Posicion actual: %f , %f \n", x,y);
	 printf("Posicion siguiente: %f , %f \n", xq,yq);
	 
	 angulo=atan(yq/xq);

	 if(xo<0 && yo<0){
		angulo=0-atan(yq/xq);
	 }
	 if(xo>0 && yo>0){
		angulo=0-atan(yq/xq);
	 }
	 else if(xo<0){
		angulo=0-atan(yq/xq);
	 }

	 else if(m==1){

	 	angulo=3.1415;
		printf("MMMMMMMMMMMM \n");

	 }
	 
	 //*next_coord_per = coord_robot;

	  if(FRx==0){
		mod = (sqrt(((FAx)*(FAx))+((FAy)*(FAy))));
		fx=FAx/mod;
		fy=FAy/mod;
		xq=x-(Mag_Advance*fx);
		yq=y-(Mag_Advance*fy);

	 	angulo=atan(yq/xq);

		if((xq<0 && yq<0) | xq<0){

			angulo=3.1415+atan(yq/xq);		
		
		}
		printf("Posicion actual: %f , %f \n", x,y);
		printf("Posicion siguiente: %f , %f \n", xq,yq);
		printf("Angulos: %f, %f \n", angulo, dest_angle);
		gen_vector=generate_output(caso,Mag_Advance,dest_angle);
		printf("Holas \n");
	 }
	 else{
		printf("Angulos: %f, %f \n", angulo, dest_angle);
	 	gen_vector=generate_output(caso,Mag_Advance,angulo);

	 }
 }


 return gen_vector;

}
                 




// Campos potenciales 2
AdvanceAngle Campos_P4(Raw observations, float dest_angle, float distance, coord coord_robot, int intensity, int a, int *next_a, float xq, float *next_xq, float yq, float *next_yq, float Mag_Advance, float max_angle, int num_sensors, float angle_start, float angle_end){

 AdvanceAngle gen_vector;
 //int obs;
 int j,l=0,m=0; //variable por si encuentra un obstaculo
 float d;
 float angulo, angulo1;
 float left_side=0;
 float right_side=0;
 int state;
 int next_state;
 int value = 0;
 float e1 = 100;
 float x,xd,xo,y,yd,yo,xq1,yq1;
 int caso=5;
 float Fx,Fy,fx,fy,fx1,fy1, dobs, FRx,FRy, FAx,FAy;
 float d0=5.0, n = 0.05, obst_angle=3.14, mod;
 float r1,r2,r3,r4,modr,ks,ns,k1;
 coord coord_obs;
 coord coord_cer;
 coord next_coord_cer;
 x =coord_robot.xc;
 y =coord_robot.yc;
 
 d=distance;

 //dobs=10;

 k1=(abs(angle_start)+abs(angle_end));
 ns= num_sensors;
 ks=k1/ns;
 printf("k1 k: %f, %f \n",k1, ks);


 for(j=0;j<num_sensors;j++){

	if(observations.sensors[j] < 0.1 && (observations.sensors[j] < observations.sensors[j-1])){
		dobs=observations.sensors[j];
		obst_angle=angle_start+(j*ks);
		l=1;
		printf("Angulos1: %f, %f, %f \n", obst_angle, angle_start, angle_end);
		printf("j k: %d, %f \n", j, ks);
		xo=dobs*cos(obst_angle); 
		yo=dobs*sin(obst_angle); 
		
	}
	else if(l!=1){
		printf("Anguloss: %f \n", obst_angle);
		xo=0; 
		yo=0; 

	}	
 }

 
 next_coord_cer=coord_robot;
 xd=d*cos(dest_angle); 
 yd=d*sin(dest_angle);

 printf("xobs, yobs: %f, %f \n", xo, yo);

 if (a!=1) {

 	xq=0;
	yq=0;
	*next_a=1;
 
 }

 if (intensity == 1){
         gen_vector=generate_output(STOP,Mag_Advance,max_angle);
         printf("Present State: STOP\n");		
 }
 else{

 	 //x=xq;
	 //y=yq; 

	 printf("\n\n **************** Student reactive Campos *********************\n");

	 //U=(1/2)*e1*(((x-xd)*(x-xd))+((y-yd)*(y-yd)));

	 modr= sqrt(((x-xo)*(x-xo))+((y-yo)*(y-yo)));

	 r1 = ((1/modr)-(1/d0));
	 r2 = (1/(modr*modr));
	 r3 = ((x-xo)/modr);
	 r4 = ((y-yo)/modr);

	 printf("rs: %f, %f, %f, %f \n", r1,r2,r3,r4);
	 printf("xobs, yobs: %f, %f \n", d, dest_angle);
	 FRx=(-n)*r1*r2*r3;
	 FRy=(-n)*r1*r2*r4;

	 if(modr>d0 | (xo==0 && l!=1)){

	 	FRx=0;
		FRy=0;
	 }
	 else if(xo==0){

		m=1;

	 }

	 printf("Fuerza de repulsion: %f , %f \n", FRx,FRy);

	 //F=e1*((x-xd),(y-yd));
	 FAx= e1*(x-xd);
	 FAy= e1*(y-yd);

	 Fx= FRx+FAx;
	 Fy= FRy+FAy;

	 mod = (sqrt(((Fx)*(Fx))+((Fy)*(Fy))));

	 fx=Fx/mod;
	 fy=Fy/mod;

	 printf("Fuerza de atraccion: %f , %f \n", FAx,FAy);
	 printf("Fuerza total: %f , %f \n", Fx,Fy);
	 printf("Fuerza total m: %f , %f, %f \n", fx,fy, mod);

	 *next_xq=x-(Mag_Advance*fx);
	 *next_yq=y-(Mag_Advance*fy);

	 xq=x-(Mag_Advance*fx);
	 yq=y-(Mag_Advance*fy);

	 printf("Posicion actual: %f , %f \n", x,y);
	 printf("Posicion siguiente: %f , %f \n", xq,yq);
	 
	 angulo=atan(yq/xq);

	 if(xo<0 && yo<0){
		angulo=0-atan(yq/xq);
	 }
	 if(xo>0 && yo>0){
		angulo=0-atan(yq/xq);
	 }
	 else if(xo<0){
		angulo=0-atan(yq/xq);
	 }

	 else if(m==1){

	 	angulo=3.1415;
		printf("MMMMMMMMMMMM \n");

	 }
	 
	 //*next_coord_per = coord_robot;

	 if(FRx==0){
		mod = (sqrt(((FAx)*(FAx))+((FAy)*(FAy))));
		fx=FAx/mod;
		fy=FAy/mod;
		xq=x-(Mag_Advance*fx);
		yq=y-(Mag_Advance*fy);

	 	angulo=atan(yq/xq);

		if((xq<0 && yq<0) | xq<0){

			angulo=3.1415+atan(yq/xq);		
		
		}
		printf("Posicion actual: %f , %f \n", x,y);
		printf("Posicion siguiente: %f , %f \n", xq,yq);
		printf("Angulos: %f, %f \n", angulo, dest_angle);
		gen_vector=generate_output(caso,Mag_Advance,dest_angle);
		printf("Holas \n");
	 }
	 else{
		printf("Angulos: %f, %f \n", angulo, dest_angle);
	 	gen_vector=generate_output(caso,Mag_Advance,angulo);

	 }
 }


 return gen_vector;

}







// Student State Machine 
AdvanceAngle state_machine_students(Raw observations, int dest, int intensity, int state, int *next_state, float Mag_Advance, float max_angle, int num_sensors, float angle_light){

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
 printf("Angle light %f\n",angle_light);

 switch ( state ) {

        case 0:
                if (intensity == 1){
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                        *next_state = 1;

                        printf("Present State: %d STOP\n", state);
			printf("\n **************** Reached light source ******************************\n");
                }
                else{

			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 1;

                        printf("Present State: %d FORWARD\n", state);
                }

                break;

        case 1:
                if (obs == 0){
			// There is not obstacle
                        gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        *next_state = 13;

                        printf("Present State: %d FORWARD\n", state);
                }
                else{
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                        printf("Present State: %d STOP\n", state);

                        if (obs == 1){
                                // obtacle in the  right
                                *next_state = 2;
                        }
                        else if (obs == 2){
                                // obtacle in the left
                                *next_state = 4;
                        }
                        else if (obs == 3){
				// obstacle in the front
                                *next_state = 6;
                        }
                }

                break;

        case 2: // Backward, obstacle in the right
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                *next_state = 3;

		printf("Present State: %d BACKWARD, obstacle right\n", state);
                break;

        case 3: // right turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_state = 0;

		printf("Present State: %d TURN LEFT\n", state);
                break;

        case 4: // Backward, obstacle in the left
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                *next_state = 5;

		printf("Present State: %d BACKWARD, obstacle left\n", state);
                break;

        case 5: // left turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 0;

		printf("Present State: %d TURN RIGTH\n", state);
                break;

        case 6: // Backward, obstacle in front
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                *next_state = 7;

		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                break;

	case 7: /// Left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_state = 8;

		printf("Present State: %d TURN 1 LEFT\n", state);
                break;

        case 8:// Left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_state = 9;

		printf("Present State: %d TURN 2 LEFT\n", state);
                break;

        case 9: // Forward
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 10;

                printf("Present State: %d 1 FORWARD\n", state);
                break;

        case 10: // Forward
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 11;

                printf("Present State: %d 2 FORWARD\n", state);
                break;

	case 11: // Right turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 12;

                printf("Present State: %d turn 1 RIGHT\n", state);
                break;

        case 12: // Right turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 0;

                printf("Present State: %d turn 2 RIGHT\n", state);
                break;


        case 13: // // check destination
		 if (dest == 0){
                                // go right
                                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                                *next_state = 5;

                                printf("Present State: %d RIGHT\n", state);
                 }
                 else if (dest == 1){
                                // go left
                                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                                *next_state = 3;

                                printf("Present State: %d LEFT\n", state);
                 }
                 else if (dest == 2){
                                // go right single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                *next_state = 5;

                                printf("Present State: %d FORWARD\n", state);
                 }
                 else if (dest == 3){
                                // go left single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                *next_state = 3;

                                printf("Present State: %d FORWARD\n", state);
                 }
                break;

	default:
		printf("State %d not defined used ", state);
                gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                next_state = 0;
                break;

                
 }

 return gen_vector;

}


// Campos potenciales 2
AdvanceAngle Campos_P2(Raw observations, float dest_angle, float distance, coord coord_robot, int intensity, int a, int *next_a, float xq, float *next_xq, float yq, float *next_yq, float Mag_Advance, float max_angle, int num_sensors, float angle_start, float angle_end){

 AdvanceAngle gen_vector;
 int obs;
 int j,l=0,m=0; //variable por si encuentra un obstaculo
 float d;
 float angulo, angulo1;
 float left_side=0;
 float right_side=0;
 int value = 0;
 float e1 = 200;
 float x,xd,xo,y,yd,yo,xq1,yq1;
 int caso=5;
 float Fx,Fy,fx,fy,fx1,fy1, dobs, FRx,FRy, FAx,FAy;
 float d0=5.0, n = 0.08, obst_angle=3.14, mod;
 float r1,r2,r3,r4,modr,ks,ns,k1;
 coord coord_obs;
 coord coord_per;
 //float x =coord_robot.xc;
 //float y =coord_robot.yc;
 
 d=distance;

 //dobs=10;

 k1=(abs(angle_start)+abs(angle_end));
 ns= num_sensors;
 ks=k1/ns;
 printf("k1 k: %f, %f \n",k1, ks);


 for(j=0;j<num_sensors;j++){

	if(observations.sensors[j] < 0.1){
		dobs=observations.sensors[j];
		obst_angle=angle_start+(j*ks);
		l=1;
		printf("Angulos1: %f, %f \n", obst_angle, angle_start);
		printf("j k: %d, %f \n", j, ks);
		xo=dobs*cos(obst_angle); 
		yo=dobs*sin(obst_angle); 
		
	}
	else if(l!=1){
		printf("Anguloss: %f \n", obst_angle);
		xo=0; 
		yo=0; 

	}	
 }

 

 xd=d*cos(dest_angle); 
 yd=d*sin(dest_angle);

 printf("xobs, yobs: %f, %f \n", xo, yo);

 if (a!=1) {

 	xq=0;
	yq=0;
	*next_a=1;
 
 }

 if (intensity == 1){
         gen_vector=generate_output(STOP,Mag_Advance,max_angle);
         printf("Present State: STOP\n");		
 }
 else{

 	 x=xq;
	 y=yq; 

	 printf("\n\n **************** Student reactive Campos *********************\n");

	 //U=(1/2)*e1*(((x-xd)*(x-xd))+((y-yd)*(y-yd)));

	 modr= sqrt(((-xo)*(-xo))+((-yo)*(-yo)));

	 r1 = ((1/modr)-(1/d0));
	 r2 = (1/(modr*modr));
	 r3 = ((-xo)/modr);
	 r4 = ((-yo)/modr);

	 printf("rs: %f, %f, %f, %f \n", r1,r2,r3,r4);

	 FRx=(-n)*r1*r2*r3;
	 FRy=(-n)*r1*r2*r4;

	 if(modr>d0 | (xo==0 && l!=1)){

	 	FRx=0;
		FRy=0;
	 }
	 else if(xo==0){

		m=1;

	 }

	 printf("Fuerza de repulsion: %f , %f \n", FRx,FRy);

	 //F=e1*((x-xd),(y-yd));
	 FAx= e1*(x-xd);
	 FAy= e1*(y-yd);

	 Fx= FRx+FAx;
	 Fy= FRy+FAy;

	 mod = (sqrt(((Fx)*(Fx))+((Fy)*(Fy))));

	 fx=Fx/mod;
	 fy=Fy/mod;

	 printf("Fuerza de atraccion: %f , %f \n", FAx,FAy);
	 printf("Fuerza total: %f , %f \n", Fx,Fy);
	 printf("Fuerza total m: %f , %f, %f \n", fx,fy, mod);

	 *next_xq=x-(Mag_Advance*fx);
	 *next_yq=y-(Mag_Advance*fy);

	 xq=x-(Mag_Advance*fx);
	 yq=y-(Mag_Advance*fy);

	 printf("Posicion siguiente: %f , %f \n", xq,yq);
	 
	 angulo=atan(yq/xq);

	 if((xq<0 && yq<0 && FRx==0) | (xq<0 && FRx==0)){

	 	angulo=3.1415+atan(yq/xq);
		printf("KKKKKKKKKKK \n");		
		
	 }
	 if(xo<0){

	 	angulo=3.1415+atan(yq/xq);
		printf("KKKKKKKKKKK \n");	

	 }
	 if(m==1){

	 	angulo=3.1415;
		printf("MMMMMMMMMMMM \n");

	 }

	 printf("Angulos: %f, %f \n", angulo, dest_angle);

	 if(FRx==0){
		gen_vector=generate_output(caso,Mag_Advance,angulo);
		printf("Holas \n");
	 }
	 else{
	 	gen_vector=generate_output(caso,Mag_Advance,angulo);
	 }
 }


 return gen_vector;

}


// Function to get next state
AdvanceAngle state_machine_campos(Raw observations, float dest_angle, float distance, coord coord_robot, int intensity, int a, int *next_a, float xq, float *next_xq, float yq, float *next_yq, float Mag_Advance, float max_angle, int num_sensors, float angle_start, float angle_end){
 
 int caso=5;
 AdvanceAngle gen_vector;

 if (intensity == 1){
        gen_vector=generate_output(STOP,Mag_Advance,max_angle);

 }
 else{

	gen_vector=generate_output(caso,Mag_Advance,dest_angle);

 }

 return gen_vector;

}



























//* -------------------------------- FUNCTIONS -------------------------------- *//
// En utilities/utilities.h
    // dif_vectors(coord vector1, coord vector2)        <- Resta de 2 puntos (vector 1 - vector 2)
    // magnitude(coord vector)                          <- Saca la magnitud de un vector 
    // distance (coord vector1, coord vector2)          <- Obtiene la distancia entre un punto y otro.
    // divide_vector_scalar(coord vector1,float cnt)    <- Divide un vector entre un escalar :p

//Funciones propias
    //Suma de 2 puntos
coord sum_vectors(coord vector1,coord vector2){

    coord sum;

    sum.xc=vector1.xc + vector2.xc;
    sum.yc=vector1.yc + vector2.yc;

    return(sum);
    
    }

    //Multiplicación por un escalar
coord multiply_vector_scalar(coord vector1,float cnt){

    coord mult;

    mult.xc=vector1.xc * cnt;
    mult.yc=vector1.yc * cnt;
    return(mult);}

    //Magnitud del vector
float magnitudVector(coord punto){
    float result;
    float aux;

    aux = (punto.xc * punto.xc) + (punto.yc * punto.yc);
    result = sqrt(aux);

    return result;  }

coord vetorNormalized(coord punto){
    coord result;
    float mag;
    
    //Se obtiene su magnitud:
    mag = magnitudVector(punto);

    //Se obtiene el vector normalizado
    result.xc = punto.xc / mag;
    result.yc = punto.yc / mag;

    return result;}

// Cálculo de 1 punto mediante campos potenciales
coord puntosRobot(coord q_0, coord q_dest, coord q_obs, float e1, float d0, float n, float delt_0){
    coord Fatr;
    coord Drep;
    coord resta1;
    coord F;
    coord f;
    coord q; //Punto calculado

    float escAux;

    //Fuerza de atracción
    Fatr = multiply_vector_scalar(dif_vectors(q_0,q_dest),e1);
    resta1 = dif_vectors(q_0,q_obs);

    //Escalar auxiliar
    escAux = -n * ((1 / magnitudVector(dif_vectors(q_0,q_obs))) -(1 / d0)) * (1 / ( magnitudVector(dif_vectors(q_0,q_obs)) * magnitudVector(dif_vectors(q_0,q_obs)) ) );

    //Fuerza de repulsión (?)
    Drep = multiply_vector_scalar(vetorNormalized(dif_vectors(q_0,q_obs)),escAux);

    //Fuerza:
    F = sum_vectors(Fatr,Drep);
    f=vetorNormalized(F);
    q = dif_vectors(q_0,multiply_vector_scalar(f,delt_0));

    return q;
    }


/* ---------------------------------------------------------- POTENTIAL FIELDS ---------------------------------------------------------------------*/
AdvanceAngle potential_fields(Raw observations, int dest, int intensity, int state, int *next_state, float Mag_Advance, float max_angle, int num_sensors, float angle_light, coord coord_robot, coord coord_dest){

 AdvanceAngle gen_vector;
 int obs;
 int j;
 float left_side=0;
 float right_side=0;
 int value = 0;
 static int step=0;

 //Variables:
 static coord p1;   
 static coord p2;   

 static coord newPoint;

 //Calculate the next point
 static coord q_0;
 static coord q_dest;
 static coord q_obs;

 static float e1 = 3; //Para el campo de atracción
 static float n = 1; //Para el campo de repulsión
 static float d0 = 5; //Permanece fijo 
 static float delt_0 = 0.1; //Magnitud de avance en el simulador

 //Calculate the advance and the angle to move the robot
 static float d1;
 static float theta;


 step++;
 printf("\n\n **************** POTENTIAL FIELDS %d *********************\n",step);

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


 /* ---------------------------------------- ALGORITHM ----------------------------------------------- */

 switch( state )
 {
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
        //Variables locales:
        static int count = 0;
        static float sumaObsSensors = 0.0;
        static float averageSensors = 0.0;
        static float sumaAngles = 0.0;
        static float averageAngles = 0.0;

        //PUNTO DESTINO 
        q_dest.xc = coord_dest.xc;
        q_dest.yc = coord_dest.yc;

        //PUNTO ACTUAL
        p1.xc = coord_robot.xc;
        p1.yc = coord_robot.yc;

        // Se obtienen los valores de todos los sensores:
        for(int i = 0; i < 16; i++) { // Asegúrate de que el índice i vaya hasta 15 inclusive
            if(observations.sensors[i] != 0.1) {
                averageSensors = averageSensors + observations.sensors[i];
                if(i>=8)
                {
                    //16.87/2 es el ángulo que hay entre el frente del sensor y el sensor 8.
                    //16.87 es el ángulo que hay entre sensores.
                    //Fórmula para obtener el ángulo del frente del sensor hacia los sensores del lado izquierdo.
                    sumaAngles = sumaAngles + 16.87/2 + (i-8)*16.87;
                }
                else if(i<=7)
                {
                    //16.87/2 es el ángulo que hay entre el frente del sensor y el sensor 8.
                    //16.87 es el ángulo que hay entre sensores.
                    //Fórmula para obtener el ángulo del frente del sensor hacia los sensores del lado derecho (de este lado los ángulos son negativos).
                    sumaAngles = sumaAngles - 16.87/2 - (7-i)*16.87; 
                }
                count++;
            }
        }

        //Promedio de los sensores
        averageSensors = sumaObsSensors / count; //Promedio de la distancia entre el robot y el obstáculo
        averageAngles = sumaAngles / count; //Promedio de los ángulos de los sensores

        //Coordenada del obstáculo:
        q_obs.xc = averageSensors * cos(averageAngles);
        q_obs.yc = averageSensors * sin(averageAngles);

        //PUNTO SIGUIENTE 
        p2 = puntosRobot(p1, q_dest, q_obs, e1, d0, n, delt_0);
        //p2 = puntosRobot(p1, q_dest, q_obs, 0.3, 0.3, 0.3, 0.3);

        printf("\nCOORD. ACTUAL: ( %f , %f )", p1.xc, p1.yc);
        printf("\nCOORD. DEST: ( %f , %f )", q_dest.xc, q_dest.yc);
        printf("\nCOORD. OBS: ( %f , %f )", q_obs.xc, q_obs.yc);
        printf("\nCOORD. SIGUIENTE: ( %f , %f )", p2.xc, p2.yc);

        //d1 = sqrt((p2.xc - p1.xc)*(p2.xc - p1.xc) + (p2.yc - p1.yc)*(p2.yc - p1.yc));
        d1 = magnitudVector(dif_vectors(p2,p1));
        theta = asin((p2.yc - p1.yc) / d1);

        //Move the robot to the p2 calculated
        gen_vector.angle = gen_vector.angle + theta;
        gen_vector.distance = gen_vector.distance + d1;

        *next_state = 0;
        break;

 }

 return gen_vector;

}

