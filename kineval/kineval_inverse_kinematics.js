
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

var jacobian=[];

 fin_jacobian=gen_jacobian(endeffector_joint,endeffector_position_local,jacobian);

var end_loc = matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_position_local);
var end_loc_f=generate_identity();

end_loc_f[0][3]=end_loc[0][0];
end_loc_f[1][3]=end_loc[1][0];
end_loc_f[2][3]=end_loc[2][0];
end_loc_f[3][3]=end_loc[3][0];

//var tar_loc =matrix_multiply(robot.links["base"].xform,endeffector_target_world.position);
var tar_loc=matrix_multiply(generate_identity(),endeffector_target_world.position);
var tar_loc_f=generate_identity();

tar_loc_f[0][3]=tar_loc[0][0];
tar_loc_f[1][3]=tar_loc[1][0];
tar_loc_f[2][3]=tar_loc[2][0];
tar_loc_f[3][3]=tar_loc[3][0];

 dx= vector_minus(tar_loc,end_loc);
dx[3]=[];dx[4]=[];dx[5]=[];

//dx[0][0] =0 ;
//dx[1][0] =0 ;
//dx[2][0] =0 ;




//var dor= vector_minus_2(endeffector_target_world.orientation,robot.joints[endeffector_joint].origin.rpy);


fin_rpy=[];
//var k;
fin_rpy[0]=0;
fin_rpy[1]=0;
fin_rpy[2]=0;

//k= iterate_rpy(endeffector_joint,fin_rpy);

//console.log(k);


m12 = robot.joints[endeffector_joint].xform[1][2];
m22 = robot.joints[endeffector_joint].xform[2][2];
m00 = robot.joints[endeffector_joint].xform[0][0];
m01 = robot.joints[endeffector_joint].xform[0][1];
m02 = robot.joints[endeffector_joint].xform[0][2];
m20 = robot.joints[endeffector_joint].xform[2][0];
m10 = robot.joints[endeffector_joint].xform[1][0];
m11 = robot.joints[endeffector_joint].xform[1][1];
m21 = robot.joints[endeffector_joint].xform[2][1];
c2 = Math.sqrt(Math.pow(m00,2) + Math.pow(m01,2));



//m12 = robot.joints[robot.endeffector.frame].xform[1][2];
//m22 = robot.joints[robot.endeffector.frame].xform[2][2];
//m00 = robot.joints[robot.endeffector.frame].xform[0][0];
//m01 = robot.joints[robot.endeffector.frame].xform[0][1];
//m02 = robot.joints[robot.endeffector.frame].xform[0][2];
//m20 = robot.joints[robot.endeffector.frame].xform[2][0];
//m10 = robot.joints[robot.endeffector.frame].xform[1][0];
//m11 = robot.joints[robot.endeffector.frame].xform[1][1];
//m21 = robot.joints[robot.endeffector.frame].xform[2][1];
//c2 = Math.sqrt(Math.pow(m00,2) + Math.pow(m10,2));

fin_rpy[0] = Math.atan2(m12,m22);
fin_rpy[1] = Math.atan2(-m02,c2);
s1 = Math.sin(fin_rpy[0]);
c1 = Math.cos(fin_rpy[0]);
//fin_rpy[2] = Math.atan2(m10,m00);

fin_rpy[2] = Math.atan2(s1*m20 - c1*m10, c1*m11 - s1*m21);

fin_rpy[0] = - fin_rpy[0];
fin_rpy[1] = - fin_rpy[1];
fin_rpy[2] = - fin_rpy[2];





//fin_rpy[0] = Math.atan2(m21,m22);
//fin_rpy[2] = Math.atan2(m10,m00);
//fin_rpy[1] = Math.atan2(-m20,Math.cos(fin_rpy[2])*m00 +Math.sin(fin_rpy[2])*m10);

//fin_rpy[1] = Math.atan2(-m20, Math.sqrt(Math.pow(m21,2)+Math.pow(m22,2)));


	if ( kineval.params.ik_orientation_included === true){

//	dx[3][0]=endeffector_target_world.orientation[0]-fin_rpy[0];
//	dx[4][0]=endeffector_target_world.orientation[1]-fin_rpy[1];
//	dx[5][0]=endeffector_target_world.orientation[2]-fin_rpy[2];
////	dx[5][0]=0;
//console.log(fin_rpy[0]);

n12 =target_mat_test[1][2];
n22 =target_mat_test[2][2];
n00 =target_mat_test[0][0];
n01 =target_mat_test[0][1];
n02 =target_mat_test[0][2];
n20 =target_mat_test[2][0];
n10 =target_mat_test[1][0];
n11 =target_mat_test[1][1];
n21 =target_mat_test[2][1];
n2 = Math.sqrt(Math.pow(n00,2) + Math.pow(n01,2));


target_rpy=[];
//var k;
target_rpy[0]=0;
target_rpy[1]=0;
target_rpy[2]=0;



	target_rpy[0] = Math.atan2(n12,n22);
	target_rpy[1] = Math.atan2(-n02,n2);
	s1_n = Math.sin(target_rpy[0]);
	c1_n = Math.cos(target_rpy[0]);
	//fin_rpy[2] = Math.atan2(m10,m00);

	target_rpy[2] = Math.atan2(s1_n*n20 - c1_n*n10, c1_n*n11 - s1_n*n21);

	target_rpy[0] = -target_rpy[0]; 
	target_rpy[1] = -target_rpy[1];
	target_rpy[2] = -target_rpy[2]; 

	
	dx[3][0]=0;
	dx[4][0]=0;	
	dx[5][0]=0;
	
	dx_2=[];
	dx_2[0]=[];
	dx_2[1]=[]; 
	dx_2[2]=[];
	dx_2[3]=[];
	dx_2[4]=[]; 
	dx_2[5]=[];
	dx_2[0][0] = dx[0][0]; 
	dx_2[1][0] = dx[1][0];
	dx_2[2][0] = dx[2][0];

//	dx_2[0][0] =0;
//	dx_2[1][0] =0;
//	dx_2[2][0] =0;



	dx_2[3][0]=target_rpy[0]-fin_rpy[0];
	dx_2[4][0]=target_rpy[1]-fin_rpy[1];
	dx_2[5][0]=target_rpy[2]-fin_rpy[2];

	dq_2= matrix_multiply((fin_jacobian),dx_2);
	var n_2=dq_2.length-1;
	//console.log(n_2);
	assign_control(dq_2,endeffector_joint,jacobian,n_2);

	}


	 else {
	dx[3][0]=0;
	dx[4][0]=0;
	dx[5][0]=0;
	
	}


	if( kineval.params.ik_pseudoinverse === false){
//	dx = numeric.transpose(dx);
	
	dq= matrix_multiply((fin_jacobian),dx);
//	dq_2= matrix_multiply((fin_jacobian),dx_2);
	}


	else if( kineval.params.ik_pseudoinverse === true && kineval.params.ik_orientation_included === true){
	fin_jacobian=numeric.transpose(fin_jacobian);
        fin_jacobian = matrix_multiply(numeric.transpose(fin_jacobian),numeric.inv(matrix_multiply(fin_jacobian,numeric.transpose(fin_jacobian))));

        dq_2= matrix_multiply((fin_jacobian),dx_2);
	}

	else {
	fin_jacobian=numeric.transpose(fin_jacobian);
	fin_jacobian = matrix_multiply(numeric.transpose(fin_jacobian),numeric.inv(matrix_multiply(fin_jacobian,numeric.transpose(fin_jacobian))));

	dq= matrix_multiply((fin_jacobian),dx);
	
//	dq_2= matrix_multiply((fin_jacobian),dx_2);

	}


	if ( kineval.params.ik_orientation_included === false){


	var n=dq.length-1;
	assign_control(dq,endeffector_joint,jacobian,n);
	}
}


function gen_jacobian(l,endeffector_position_local,jacobian){

var joint_axis=[];
joint_axis[0]=[]; joint_axis[1]=[]; joint_axis[2]=[]; joint_axis[3]=[];
joint_axis[0][0]=robot.joints[l].axis[0];
joint_axis[1][0]=robot.joints[l].axis[1];
joint_axis[2][0]=robot.joints[l].axis[2];
joint_axis[3][0]=1;
var zi=matrix_multiply(robot.joints[l].xform,joint_axis);

//var zi =robot.joints[l].axis;
var Oi=matrix_multiply(robot.joints[l].xform,[[0],[0],[0],[1]]);
zi[0][0] = zi[0][0]-Oi[0][0];
zi[1][0] = zi[1][0]-Oi[1][0];
zi[2][0] = zi[2][0]-Oi[2][0];


var On=matrix_multiply(robot.joints[l].xform,endeffector_position_local);
var up_jacob=vector_cross_f(zi,vector_minus(On,Oi));
var jacob=[];
	if ( robot.joints[j].type == 'prismatic') {

	 jacob[3]=0;
	 jacob[4]=0;
	 jacob[5]=0;
	 jacob[0]=zi[0][0];
	 jacob[1]=zi[1][0];
	 jacob[2]=zi[2][0];

//	jacob[0]=zi[0];
//	jacob[1]=zi[1];
//	jacob[2]=zi[2];

	}	

	else {

	 jacob[0]=up_jacob[0][0];
	 jacob[1]=up_jacob[1][0];
	 jacob[2]=up_jacob[2][0];

	 jacob[3]=zi[0][0];
	 jacob[4]=zi[1][0];
	 jacob[5]=zi[2][0];
	
//jacob[3] = 0;
//jacob[4] = 0;
//jacob[5] = 0;


	}

if (l != "base"){

        jacobian.unshift(jacob);

        if (typeof robot.links[robot.joints[l].parent].parent === 'undefined'){
        return jacobian;
                }
        else{
        gen_jacobian(robot.links[robot.joints[l].parent].parent,endeffector_position_local,jacobian);

                }
return jacobian;


        }
}

/*
function iterate_rpy(l,fin_rpy){

	 if (typeof robot.links[robot.joints[l].parent].parent === 'undefined'){
     
	console.log(fin_rpy);
	   return fin_rpy;
                 }
         else{
	 fin_rpy[0] += robot.joints[l].origin.rpy[0] ;
	 fin_rpy[1] += robot.joints[l].origin.rpy[1] ;
	 fin_rpy[2] += robot.joints[l].origin.rpy[2] ;
      	 iterate_rpy(robot.links[robot.joints[l].parent].parent,fin_rpy);

                 }




}

*/
function assign_control(dq,l,jacobian,n){

robot.joints[l].control=kineval.params.ik_steplength*dq[n][0];
//console.log(n);
        if (typeof robot.links[robot.joints[l].parent].parent === 'undefined'){
        return ;
                 }
         else{
         n=n-1;
         assign_control(dq,robot.links[robot.joints[l].parent].parent,jacobian,n);

                 }

}




