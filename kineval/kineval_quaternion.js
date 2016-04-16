//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply


function quaternion_from_axisangle(x){
//console.log(x);
//console.log(robot.joints[x].angle);

//y = robot.joints[x].angle;

//console.log(y);
        q=[];
        q.y=[];
        q.y[0]=Math.cos(robot.joints[x].angle/2);
        q.y[1]=robot.joints[x].axis[0]*Math.sin(robot.joints[x].angle/2);
        q.y[2]=robot.joints[x].axis[1]*Math.sin(robot.joints[x].angle/2);
        q.y[3]=robot.joints[x].axis[2]*Math.sin(robot.joints[x].angle/2);

return q.y;

}



function quaternion_normalize(q){

norm=Math.pow(q[0],2)+Math.pow(q[1],2)+Math.pow(q[2],2)+Math.pow(q[3],2);
norm=Math.sqrt(norm);


q[0]=q[0]/norm;
q[1]=q[1]/norm;
q[2]=q[2]/norm;
q[3]=q[3]/norm;


return q;
}


function quaternion_multiply(q1,q2){

q=[];
a=q1[0]; b = q1[1] ; c = q1[2]; d = q1[3];
e=q2[0]; f = q2[1] ; g = q2[2]; h = q2[3];

q[0]=a*e-b*f-c*g-d*h;
q[1]=a*f+b*e+c*h-d*g;
q[2]=a*g-b*h+c*e+d*f;
q[3]=a*h+b*g-c*f+d*e;

return q;

}


function quaternion_to_rotation_matrix(x){

//q=quaternion_normalize(quaternion_from_axisangle(x));

q = x;

q0=q[0];q1=q[1];q2=q[2];q3=q[3];



Q=[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]];

Q[0][0]=1-2*(q2*q2+q3*q3);
Q[0][1]=2*(q1*q2-q0*q3);
Q[0][2]=2*(q0*q2+q1*q3);
Q[0][3]=0;
Q[1][0]=2*(q1*q2+q0*q3);
Q[1][1]=1-2*(q1*q1+q3*q3);
Q[1][2]=2*(q2*q3-q0*q1);
Q[1][3]=0;
Q[2][0]=2*(q1*q3-q0*q2);
Q[2][1]=2*(q0*q1+q2*q3);
Q[2][2]=1-2*(q1*q1+q2*q2);
Q[2][3]=0;
Q[3][0]=0;
Q[3][1]=0;
Q[3][2]=0;
Q[3][3]=1;

return Q;



}




















