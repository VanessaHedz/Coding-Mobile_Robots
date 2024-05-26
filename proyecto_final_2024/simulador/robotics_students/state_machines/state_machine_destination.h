/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_destination.h                    	*
 *                                                      *
 *		Jesus Savage				*
 *		FI-UNAM					*
 *		6-1-2015                                *
 *                                                      *
 ********************************************************/


// Function to get next state
AdvanceAngle state_machine_destination(int a, Raw observations, int dest, int intensity, int state, int *next_state,int i, int *next_i,float Mag_Advance,float max_angle){

 AdvanceAngle gen_vector;
 FILE *archivo;
 char linea[100];
 float fxi=0.061735;
 float s1=0,s2=0,s3=0,s4=0,s5=0;
 float a1,a2,E1;
 float xi;

 printf("obs intensity %d dest %d\n",intensity,dest);

 switch ( state ) {

        case 0:
                if (a == 1){
                        gen_vector=generate_output(STOP,0,0);
                        printf("Present State: %d STOP\n", state);
			archivo = fopen("/home/alejandro/robotics_students/state_machines/datos2.txt","a");

			if (archivo==NULL){
				printf("Error al abrir el archivo");
			}

			fprintf(archivo,"%lf\n",observations.sensors[9]);
			fclose(archivo);
			*next_i = i+1;
			if(*next_i>=500) *next_state = 1;
			else *next_state = 0;
                }
                else{
			gen_vector=generate_output(STOP,0,0);
                        printf("Present State: %d EEEEEEEEEENNNNNNNNNNTROOOOOOOOOOOO\n", state);
			*next_i = i+1;
			if(*next_i>=1) *next_state = 1;
			else *next_state = 1;
                }

                break;
        
	case 1:
                archivo = fopen("/home/alejandro/robotics_students/state_machines/datos2.txt","r");
		if (archivo==NULL){
			printf("Error al abrir el archivo");
		}
		while (fgets(linea, sizeof(linea),archivo)){
			xi = atof(linea);
			//printf("xi: %f EEEEEEEEEEQUIS\n", xi);
			s1 = s1 + fxi;
			s2 = s2 + (xi*xi);
			s3 = s3 + (fxi*xi);
			s4 = s4 + xi;
			a1 = ((s1*s2)-(s3*s4))/((500*s2)-(s4*s4));
			//printf("a1: %f \n", a1);
			//printf("a2: %f \n", a2);
			//printf("s1: %f \n", s1);
			//printf("s2: %f \n", s2);
			//printf("s3: %f \n", s3);
			//printf("s4: %f \n", s4);

		}
		fclose(archivo);
		a1 = ((s1*s2)-(s3*s4))/((500*s2)-(s4*s4));
		a2 = ((s3)-((s1*s4)/500))/((s2)-((s4*s4)/500));

		printf("a1: %f ", a1);
		printf("a2: %f ", a2);

		archivo = fopen("/home/alejandro/robotics_students/state_machines/datos2.txt","r");
		if (archivo==NULL){
			printf("Error al abrir el archivo");
		}
		while (fgets(linea, sizeof(linea),archivo)){
			xi = atof(linea);
			//printf("xi: %f IIIIIIIIII\n", xi);
			s5=s5+(((fxi-a1-(a2*xi))*(fxi-a1-(a2*xi)))/(500-2));

		}
		fclose(archivo);
		//s5=s5+(((fxi-a1-(a2*xi))*(fxi-a1-(a2*xi)))/(500-2));
		printf("E1: %f ", s5);

		gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                *next_state = 1;

                break;
         
      
	default:
                printf("State %d not defined used ", state);
                gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                *next_state = 0;
                break;

 }

 //printf("Next State: %d\n", *next_state);
 return gen_vector;

}


