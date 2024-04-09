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

 std::list<int> Obs_Sensors;

 float sumaObsSensors = 0.0;
 float averageSensors = 0.0;
 float sumaAngles = 0.0;
 float averageAngles = 0.0;


 //Variables:
 static coord p1;   
 static coord p2;   

 static coord newPoint;

 //Calculate the next point
 static coord q_0;
 static coord q_dest;
 static coord q_obs;

 static float e1 = 1.0;
 static float n = 1.0;
 static float d0 = 1.0;
 static float delt_0 = 1.0;

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

        // Se obtienen los valores de todos los sensores:
        for(int i = 0; i < 16; i++) { // Asegúrate de que el índice i vaya hasta 15 inclusive
            if(observations.sensors[i] != 0.1) {
                Obs_Sensors.push_back(observations.sensors[i]); 

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

            }
        }

        // Se obtiene un promedio de los sensores que detectaron un obstáculo
        /*for(int i = 0; i < Obs_Sensors.size(); i++) { 
            sumaObsSensors = sumaObsSensors + Obs_Sensors[i];
        }*/

        //No permite la forma anterior, así que la sustituimos por esta: 
        //*it es un apuntador que recorre toda la lista de Obs_sensors y con él podemos hacer la suma de los sensores.
        for(auto it = Obs_Sensors.begin(); it != Obs_Sensors.end(); ++it) {
            sumaObsSensors = sumaObsSensors + *it;
        }

        //Promedio de los sensores
        averageSensors = sumaObsSensors / Obs_Sensors.size(); //Promedio de la distancia entre el robot y el obstáculo
        averageAngles = sumaAngles / Obs_Sensors.size(); //Promedio de los ángulos de los sensores

        //Coordenada del obstáculo:
        q_obs.xc = averageSensors * cos(averageAngles);
        q_obs.yc = averageSensors * sin(averageAngles);

        //PUNTO SIGUIENTE 
        //p2 = puntosRobot(p1, q_dest, q_obs, e1, d0, n, delt_0);
        p2 = puntosRobot(p1, q_dest, q_obs, 0.3, 0.3, 0.3, 0.3);

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

