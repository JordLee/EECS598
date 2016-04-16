
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        
	console.log(T_a.vertices.length);


	var motion_plan_a = [];
	var motion_plan_b = [];
//	motion_plan_A = find_path(T_a.vertices[0],T_a,motion_plan_a);


	find_path_2(T_a.vertices[0],T_a.vertices[T_a.newest],motion_plan_a);
	find_path_2_B(T_b.vertices[0],T_b.vertices[T_b.newest],motion_plan_b);

	var C =A.concat(B);
	for (i=0;i<C.length;i++){

	C[i].geom.material.color = { r:1,g:0,b:0 };
	}
	kineval.motion_plan = C;

//	for (i=0;i<B.length;i++){
//
//	B[i].geom.material.color = { r:1,g:0,b:0 };
//	}


	//console.log(motion_plan_a);
//	var motion_plan_b = [];	
//	motion_plan_b = find_path(T_b.vertices[0],T_b,motion_plan_b);

//	kineval.motion_plan = motion_plan_a.push(motion_plan_b);




	}
        else kineval.params.planner_state = "searching";
   	}
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0.001;


	q_init = q_start_config;
	q_goal = q_goal_config;

	T_a = tree_init(q_init);
	T_b = tree_init(q_goal);




    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
}



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations

	var q_rand = random_config();

//console.log(rrt_extend(T_a,q_rand));
		if (rrt_extend(T_a,q_rand) !== "trapped"){	
	

//		console.log(q_new);
			if (rrt_connect(T_b,T_a.vertices[T_a.newest].vertex) == "reached"){
			console.log("terminate")
			return "reached";
			}			
		T_c = T_a;
		T_a = T_b;		 
		T_b = T_c;

		}	


//	rrt_extend(T_a,q_rand);
    }
//	 tree = tree_init(q);
 //      	console.log(tree_add_vertex(tree,q));


}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
  //  tree.vertices[q1_idx].child.push(tree.vertices[q2_idx]);

  //   tree.vertices[q2_idx].parent.push(tree.vertices[q1_idx]);


}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

function rrt_extend(tree,q) {

qnear = nearest_neighbor(q,tree);

q_near= qnear[1];
q_near_index = qnear[0];

//if ((new_config(q,q_near) && robot_collision_forward_kinematics(q_near))==false) {


//console.log(robot_collision_forward_kinematics(q_near));
if (new_config(tree,q,q_near)) {

	var q_new=new_config(tree,q,q_near);
	tree_add_vertex(tree,q_new);	
	tree_add_edge(tree,q_near_index,tree.newest);

//	console.log(q_new[0]-q[0]);


	if(Math.abs(Math.pow(q_new[0]-q[0],2)+Math.pow(q_new[2]-q[2],2))<0.01){
	return "reached"
	} else{

//	console.log("working/?");
	return "advanced"
	}
	
	
	}
return "trapped"



}



function new_config(tree,q,q_near) {

stepsize =0.1;
var q_new = [];

//console.log(q_near);
//console.log(q);
q_new[0]=stepsize*(q[0]-q_near[0])/Math.abs(q[0]-q_near[0])+q_near[0];
q_new[1]=0;
q_new[2]=stepsize*(q[2]-q_near[2])/Math.abs(q[2]-q_near[2])+q_near[2];

//q_new[0] = q[0];
//q_new[2] = q[2];


q_new[3] = q[3];
q_new[4] = q[4];
q_new[5] = q[5];

//console.log(q_new);
//console.log(tree.newest);

	if (tree.newest == 0){
	return q_new
	}


	else if(tree.newest !==0 &&robot_collision_forward_kinematics(q_new)==false){
	return q_new
	}
}


function nearest_neighbor(q,tree) {

	 distance = [];

	for (i=0;i<tree.vertices.length;i++) {
	
	 distance[i] = Math.pow(q[0]-tree.vertices[i].vertex[0],2) + Math.pow(q[2]-tree.vertices[i].vertex[2],2); 
	}

	 nearest_index = distance.indexOf(Math.min(...distance));

	return [nearest_index,tree.vertices[nearest_index].vertex]

}


function random_config() {


//	if (robot_collision_forward_kinematics(q)){
	var q_rand = [  [2*(Math.random()-0.5)*robot_boundary[0][0]],
                [0],
              [2*(Math.random()-0.5)*robot_boundary[0][2]],[0],[Math.random()*2*Math.PI],[0] ];

	return q_rand

//	}

}

function rrt_connect(tree,q) {
//console.log("getting in");
	var s = "trapped";
	var s;	
//	while ( s !== "advanced") {

//	console.log("while?");
	s = rrt_extend(tree,q);
	console.log(s);	
//	}
	return s
}



function find_path_2(TV,tree,motion_plan){

var k = motion_plan.length;

console.log(k);
if (Math.pow(TV.vertex[0]-tree.vertex[0],2) + Math.pow(TV.vertex[2]-tree.vertex[2],2)>0.01){
motion_plan.push(tree);
console.log(motion_plan);
}
else if(Math.pow(TV.vertex[0]-tree.vertex[0],2) + Math.pow(TV.vertex[2]-tree.vertex[2],2)<0.001){
A = motion_plan;
//console.log(A);

return A;

}
find_path_2(TV,tree.edges[0],motion_plan);
}


function find_path_2_B(TVB,treeB,motion_planB){

var k = motion_planB.length;

console.log(k);
if (Math.pow(TVB.vertex[0]-treeB.vertex[0],2) + Math.pow(TVB.vertex[2]-treeB.vertex[2],2)>0.01){
motion_planB.push(treeB);
//console.log(motion_plan);
}
else if(Math.pow(TVB.vertex[0]-treeB.vertex[0],2) + Math.pow(TVB.vertex[2]-treeB.vertex[2],2)<0.001){
B = motion_planB;
//console.log(A);

return B;

}



find_path_2_B(TVB,treeB.edges[0],motion_planB);

}


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path

    //   path_dfs










