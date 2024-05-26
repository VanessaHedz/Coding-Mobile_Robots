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





// Campos potenciales 2
AdvanceAngle Campos_P5(Raw observations, float dest_angle, float distance, coord coord_robot, int intensity, int a, int *next_a, float xq, float *next_xq, float yq, float *next_yq, float Mag_Advance, float max_angle, int num_sensors, float angle_start, float angle_end, int dest, int obs){

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
 float Fx,Fy,fx,fy,fx1,fy1, dobs, FRx,FRy, FAx,FAy, medio;
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
		if ((abs(obst_angle))<=0.4) medio=1;
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

	 if ((coord_cer.xc/coord_robot.xc)>=0.8 && (coord_cer.xc/coord_robot.xc)<=1.2 && (coord_cer.yc/coord_robot.yc)>=0.8 && (coord_cer.yc/coord_robot.yc)<=1.2) {
                // obtacle in the right
                state=next_state;
		gen_vector=state_machine_avoidance_destination(obs, dest, intensity, state,&next_state, Mag_Advance, max_angle);
        	
                
	 }
	 
	 else if(FRx==0){
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
