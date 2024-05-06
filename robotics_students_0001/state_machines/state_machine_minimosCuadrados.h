/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_minimosCuadrados.h               	*
 *                                                      *
 *		Hernández Chagoyán, Vanessa Mariana           	*
 *      Sánchez Ledesma, Alejandro                      *
 *                                                      *
 *		FI-UNAM					                        *
 *		04/05/2024                                      *
 *                                                      *
 ********************************************************/


// Function to get next state
AdvanceAngle minimosCuadrados(Raw observations, int dest, int intensity, int state, int *next_state,float Mag_Advance,float max_angle, coord coord_robot){

 AdvanceAngle gen_vector;

 //Variables

 static float x_real = 0.061733;  //Value of the 9th sensor in de position: (0.595,0.5375)
 static std::vector<float> X;       //Vector X of the values with noise from the 9th sensor.
 static int i=0;                  //Counter for the 500 observations
 static float angle;


 switch( state )
 {
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
        *next_state = 1;
        break;
    
    case 1:
        if(i=500)
        {
            //STOP the count 
            *next_state = 3;
            break;
        }

        //Save the value of the 9th sensor in the list
        X.push_back(observations.sensors[9]);
        *next_state = 2;
        break;
    
    case 2:
        i++;
        *next_state = 1;
        break;
    
    case 3:
        //PRINT THE OBSERVATIONS
        printf("**** VALUES OF X ****\n");
        for(int j=0; i<500; i++)
        {
            printf("\n %f", X[j]);
        }
        break;
 }

 return gen_vector;
}
