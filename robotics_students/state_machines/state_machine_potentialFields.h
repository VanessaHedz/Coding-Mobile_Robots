/********************************************************
 *                                                      *
 *      state_machine_potentialFields.h            		*
 *                                                      *
 *		Student:	Vanessa Hernández                   *
 *		FI-UNAM				                        	*
 *		3-14-2024                                       *
 *                                                      *
 ********************************************************/

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
coord puntosRobot_UnObs(coord q_0, coord q_dest, coord q_obs, float e1, float d0, float n, float delt_0){
//coord puntosRobot(coord q_0, coord q_dest, coord q_obs1, coord q_obs2, float e1, float d0, float n, float delt_0){
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

    //Editarlo para que lea 2 obstáculos:
    //obs1 = multiply_vector_scalar(vetorNormalized(dif_vectors(q_0,q_obs1)),escAux);
    //obs2 = multiply_vector_scalar(vetorNormalized(dif_vectors(q_0,q_obs2)),escAux);
    //Drep = obs1 + obs2;

    //Fuerza:
    F = sum_vectors(Fatr,Drep);
    f=vetorNormalized(F);
    q = dif_vectors(q_0,multiply_vector_scalar(f,delt_0));

    return q;
    }

coord puntosRobot_TresObs(coord q_0, coord q_dest, coord q_obs1, coord q_obs2, coord q_obs3, float e1, float d0, float n, float delt_0){
    coord Fatr;
    coord Drep;
    //coord resta1;
    coord F;
    coord f;
    coord q; //Punto calculado

    coord obs1;
    coord obs2;
    coord obs3;

    float escAux1;
    float escAux2;
    float escAux3;

    //Fuerza de atracción
    Fatr = multiply_vector_scalar(dif_vectors(q_0,q_dest),e1);
    //resta1 = dif_vectors(q_0,q_obs);

    //Escalar auxiliar
    escAux1 = -n * ((1 / magnitudVector(dif_vectors(q_0,q_obs1))) -(1 / d0)) * (1 / ( magnitudVector(dif_vectors(q_0,q_obs1)) * magnitudVector(dif_vectors(q_0,q_obs1)) ) );
    escAux2 = -n * ((1 / magnitudVector(dif_vectors(q_0,q_obs2))) -(1 / d0)) * (1 / ( magnitudVector(dif_vectors(q_0,q_obs2)) * magnitudVector(dif_vectors(q_0,q_obs2)) ) );
    escAux3 = -n * ((1 / magnitudVector(dif_vectors(q_0,q_obs3))) -(1 / d0)) * (1 / ( magnitudVector(dif_vectors(q_0,q_obs3)) * magnitudVector(dif_vectors(q_0,q_obs3)) ) );

    //Fuerza de repulsión (?)
    //Editarlo para que lea 2 obstáculos:
    obs1 = multiply_vector_scalar(vetorNormalized(dif_vectors(q_0,q_obs1)),escAux1);
    obs2 = multiply_vector_scalar(vetorNormalized(dif_vectors(q_0,q_obs2)),escAux2);
    obs3 = multiply_vector_scalar(vetorNormalized(dif_vectors(q_0,q_obs2)),escAux3);
    
    Drep = sum_vectors(sum_vectors(obs1,obs2),obs3);
    //Drep = obs1 + obs2 + obs3;

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
 static coord q_obs1;
 static coord q_obs2;
 static coord q_obs3;

 static float ang_obs1 = -0.7853; //[ rad ]     //Estos se deben definir en el inicio. Los obtengo desde el GUI
 static float ang_obs2 = 0;         //Es un rango de 45grados
 static float ang_obs3 = 0.7853;

 static float d_obs1;
 static float d_obs2;
 static float d_obs3;

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
        //PUNTO DESTINO 
        q_dest.xc = coord_dest.xc;
        q_dest.yc = coord_dest.yc;

        //PUNTO ACTUAL
        p1.xc = coord_robot.xc;
        p1.yc = coord_robot.yc;


        //Detectar q_obs1 y q_obs2
        d_obs1 = observations.sensors[0];
        d_obs2 = observations.sensors[1];
        d_obs2 = observations.sensors[3];

        q_obs1.xc = d_obs1 * cos(ang_obs1);
        q_obs1.yc = d_obs1 * sin(ang_obs1);

        q_obs2.xc = d_obs2 * cos(ang_obs2);
        q_obs2.yc = d_obs2 * sin(ang_obs2);

        q_obs3.xc = d_obs3 * cos(ang_obs3);
        q_obs3.yc = d_obs3 * sin(ang_obs3);

        //p2 = puntosRobot_TresObs(p1, q_dest, q_obs1,q_obs2,q_obs3, e1,   d0,  n, delt_0);
        p2 = puntosRobot_TresObs(p1, q_dest, q_obs1, q_obs2, q_obs3, 3.0, 1.0, 1.0, 0.1);

        printf("\nCOORD. ACTUAL: ( %f , %f )", p1.xc, p1.yc);
        printf("\nCOORD. DEST: ( %f , %f )", q_dest.xc, q_dest.yc);
        printf("\nCOORD. OBS 1: ( %f , %f )", q_obs1.xc, q_obs1.yc);
        printf("\nCOORD. OBS 2: ( %f , %f )", q_obs2.xc, q_obs2.yc);
        printf("\nCOORD. OBS 3: ( %f , %f )", q_obs3.xc, q_obs3.yc);
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
