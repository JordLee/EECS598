
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();
	if (robot.name == "fetch"){
	kineval.buildFKTransformsFetch();
	}
	else{
	kineval.buildFKTransforms();
	}
}
kineval.buildFKTransformsFetch = function traverseFKBaseFetch () {

var xyz = generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]);
var roty = generate_rotation_matrix_Y(robot.origin.rpy[2]);
var rotx = generate_rotation_matrix_X(robot.origin.rpy[0]-Math.PI/2);
var rotz = generate_rotation_matrix_Z(robot.origin.rpy[1]);
var rotxyz = matrix_multiply_3(rotx,roty,rotz);
var mstack = matrix_multiply(xyz,rotxyz);

 robot.origin.xform = mstack;
 traverseFKLinkFetch(mstack,robot.base);
 }


function traverseFKLinkFetch (mstack,l){

robot.links[l].xform = mstack;
var i ;

if (typeof robot.links[l].children === 'undefined')
return;

 for ( i=0 ; i<robot.links[l].children.length; i++){
 j= robot.links[l].children[i];
 traverseFKJointFetch(mstack,j);
  }
}

function traverseFKJointFetch (mstack,j){

var xyz = generate_translation_matrix(robot.joints[j].origin.xyz[0], robot.joints[j].origin.xyz[1], robot.joints[j].origin.xyz[2]);
var roty = generate_rotation_matrix_Y(robot.joints[j].origin.rpy[0]);
var rotx = generate_rotation_matrix_X(robot.joints[j].origin.rpy[2]);
var rotz = generate_rotation_matrix_Z(robot.joints[j].origin.rpy[1]);
var rotxyz = matrix_multiply_3(rotx,roty,rotz);
var transform=matrix_multiply(xyz,rotxyz);

robot.joints[j].xform = matrix_multiply(mstack,transform);
var mstack = robot.joints[j].xform;
l = robot.joints[j].child;
traverseFKLinkFetch(mstack,l);
	
} 
 
kineval.buildFKTransforms = function traverseFKBase () {

var xyz = generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]);
var rotz = generate_rotation_matrix_Z(robot.origin.rpy[0]);
var roty = generate_rotation_matrix_Y(robot.origin.rpy[1]);
var rotx = generate_rotation_matrix_X(robot.origin.rpy[2]);
var rotxyz = matrix_multiply_3(rotx,roty,rotz);
var mstack = matrix_multiply(xyz,rotxyz);

robot.origin.xform = mstack;
traverseFKLink(mstack,robot.base);
 }


function traverseFKLink (mstack,l){
robot.links[l].xform = mstack;
var i ;
if (typeof robot.links[l].children === 'undefined')
return;
	for ( i=0 ; i<robot.links[l].children.length; i++){
        j= robot.links[l].children[i];

        traverseFKJoint(mstack,j);
        }
}

function traverseFKJoint (mstack,j){

var xyz = generate_translation_matrix(robot.joints[j].origin.xyz[0], robot.joints[j].origin.xyz[1], robot.joints[j].origin.xyz[2]);
var rotz = generate_rotation_matrix_Z(robot.joints[j].origin.rpy[2]);
var roty = generate_rotation_matrix_Y(robot.joints[j].origin.rpy[1]);
var rotx = generate_rotation_matrix_X(robot.joints[j].origin.rpy[0]);
var rotxyz = matrix_multiply_3(rotx,roty,rotz);
var transform=matrix_multiply(xyz,rotxyz);

robot.joints[j].xform = matrix_multiply(mstack,transform);
var mstack = robot.joints[j].xform;
l = robot.joints[j].child;

traverseFKLink(mstack,l);

 } 
    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
    //   if (robot.links_geom_imported) {
    //       var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));

