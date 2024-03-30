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
    return(mult);
}

    //Magnitud del vector
float magnitudVector(coord punto)
{
    float result;
    float aux;

    aux = (punto.xc * punto.xc) + (punto.yc * punto.yc);
    result = sqrt(aux);

    return result;  
}

coord vetorNormalized(coord punto)
{
    coord result;
    float mag;
    
    //Se obtiene su magnitud:
    mag = magnitudVector(punto);

    //Se obtiene el vector normalizado
    result.xc = punto.xc / mag;
    result.yc = punto.yc / mag;

    return result;
}

// Cálculo de 1 punto mediante campos potenciales
coord puntosRobot(coord q_0, coord q_dest, coord q_obs, float e1, float d0, float n, float delt_0)
{
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
    escAux = -n * (1 / magnitudVector(dif_vectors(q_0,q_obs))) -(1 / d0) * (1 / ( magnitudVector(dif_vectors(q_0,q_obs)) * magnitudVector(dif_vectors(q_0,q_obs)) ) );

    //Fuerza de repulsión (?)
    Drep = multiply_vector_scalar(vetorNormalized(resta1),escAux);

    //Fuerza:
    F = sum_vectors(Fatr,Drep);
    f=vetorNormalized(F);
    q = dif_vectors(q_0,multiply_vector_scalar(f,delt_0));

    return q;
}


/* ---------------------------------------------------------- POTENTIAL FIELDS ---------------------------------------------------------------------*/
AdvanceAngle potential_fields(Raw observations, int dest, int intensity, float Mag_Advance, float max_angle, int num_sensors, coord coord_robot, coord coord_dest){

 AdvanceAngle gen_vector;
 int obs;
 int j;
 float left_side=0;
 float right_side=0;
 int value = 0;
 static int step=0;

 //static coord obs; //Keep the coords of the obstacle according to the sensors.

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

 /* ---------------------------------------- ALGORITHM ----------------------------------------------- */
 
 return gen_vector;

}

