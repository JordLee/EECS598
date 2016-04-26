
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
            textbar.innerHTML = "planner execution complete. Press 2 to execute RRT *";
            kineval.params.planner_state = "complete";
        
	//console.log(T_a.vertices.length);


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


//	kineval.motion_plan = C;

	console.log("geeting");
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

//	if (( keyboard.pressed("n"))
// 	if ((keyboard.pressed("n"))&&(kineval.motion_plan_traversal_index<kineval.motion_plan.length-1)) {
//                kineval.motion_plan_traversal_index++; 
//	}
	

        // KE 2 : need to move q_names into a global parameter
        for (var x in robot.joints) {
      		
            //console.log(kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]]);
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

    for (var x in robot.joints) {
        q_names[x] = q_start_config.length;
	q_index[q_start_config.length] = x;
    
	q_start_config[q_names[x]] = robot.joints[x].angle;
//	q_start_config = q_start_config.concat(robot.joints[x].angle);
	}

	
    //console.log(q_names);

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;


	q_init = q_start_config;
	q_goal = q_goal_config;

	T_a = tree_init(q_init);
	T_b = tree_init(q_goal);
	console.log("T_a");
	console.log(T_a);



    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
}



function robot_rrt_planner_iterate() {

    var i;
    if (typeof rrt_alg === 'undefined'){
    rrt_alg =2 ;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED) 2: rrt_*
    }		
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
	if (rrt_alg == 1){
	var q_rand = random_config();

//console.log(rrt_extend(T_a,q_rand));
		if (rrt_extend(T_a,q_rand) !== "trapped"){	
	

//		console.log(q_new);
			if (rrt_connect(T_b,T_a.vertices[T_a.newest].vertex) == "reached"){
			console.log("terminate")
			
			rrt_iterate = false;
//			return "reached";
			}			
	

		T_c = T_a;
		T_a = T_b;		 
		T_b = T_c;

		}	


//	rrt_extend(T_a,q_rand);
    
//	 tree = tree_init(q);
 //      	console.log(tree_add_vertex(tree,q));

	if (rrt_iterate == false){
	var motion_plan_a = [];
	var motion_plan_b = [];
//	motion_plan_A = find_path(T_a.vertices[0],T_a,motion_plan_a);


	find_path_2(T_a.vertices[0],T_a.vertices[T_a.newest],motion_plan_a);
	find_path_2_B(T_b.vertices[0],T_b.vertices[T_b.newest],motion_plan_b);
	A= A.reverse();
//	B= B.reverse();

	var C =A.concat(B);
		for (i=0;i<C.length;i++){

		C[i].geom.material.color = { r:1,g:0,b:0 };
		}
	kineval.motion_plan = C;
	return "reached"
	}

	
       // kineval.params.generating_motion_plan = false;
	}
	if (rrt_alg == 2){
	stepsize= 0.5;
	rrt_extend_2(T_a);

		if (rrt_iterate == false){
			kineval.motion_plan = D;
			for (var i=0;i<D.length;i++){

			D[i].geom.material.color = { r:0,g:0,b:1 };
			}
		return "reached"
		}

	}



   }


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
    tree.vertices[0].cost = [];
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
    new_vertex.cost = tree.vertices[tree.newest].cost;
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
//console.log("q in rrtextend");
//console.log(q);
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


	if(Math.sqrt(Math.pow(q_new[0]-q[0],2)+Math.pow(q_new[2]-q[2],2))<stepsize){
	return "reached"
	} else{

//	console.log("working/?");
	return "advanced"
	}
	
	
	}
return "trapped"



}



function new_config(tree,q,q_near) {

stepsize =0.5;
var q_new = [];

//if (tree.vertices.length == 1){
var q_new_normalize = Math.sqrt(Math.pow(q[0]-q_near[0],2)+Math.pow(q[2]-q_near[2],2));
q_new[0]=stepsize*(q[0]-q_near[0])/q_new_normalize+q_near[0];
q_new[1]=0;
q_new[2]=stepsize*(q[2]-q_near[2])/q_new_normalize+q_near[2];
//
////q_new[0] = [0]
//
//
//
////console.log(q_names);
////for (i=0;i< robot.joints.length;i++) {
////q_new[3+i*3] = q[3];
////q_new[4+i*3] = 0;
////q_new[5+i*3] = q[5];
////}
//
q_new[3] = q[3];
q_new[4] = q[4];
q_new[5] = q[5];
//
	for (x in robot.joints) {
//
//      	if (typeof robot.joints[x].limit === 'undefined')
//	q_new[q_names[x]]=2*(Math.random()-0.5)*(Math.PI);
//	else
//	q_new[q_names[x]] = Math.random()*(robot.joints[x].limit.lower-robot.joints[x].limit.upper)+robot.joints[x].limit.lower;
////	robot.joints[x].limit[0]-robot.joints[x].limit[1]
//	//q_new[q_names[x]]=2*(Math.random()-0.5)*(Math.PI); 
	q_new[q_names[x]] = q[q_names[x]];
	}


//	console.log("stuck");
//
//}
//else{
//console.log(q);
//console.log(q_near);
//console.log(vector_minus_2(q,q_near));
//console.log("vector_normailize")
//console.log(vector_normalize_2(vector_minus_2(q,q_near)));
//q_new=vector_plus_2(vector_normalize_2(vector_minus_2(q,q_near)).map(function(x) x*stepsize),q_near);
//}
//console.log("q_new");
//console.log(q_new);
//	if (tree.newest == 0){
	
//	return q_new
//	}


	 if(kineval.poseIsCollision(q_new)==false){
//	console.log(robot_collision_forward_kinematics(q_new));

	return q_new
	}
//	else if(tree.newest !==0 &&robot_collision_forward_kinematics(q_new)!==false){
	else{
//	new_config(tree,q,q_near);
	return false
	}
	

}


function nearest_neighbor(q,tree) {

	var  distance = [];
	for ( i = 0; i< tree.vertices.length;i++){
	//distance[i]=vector_distance(q,tree.vertices[i].vertex);
        distance[i] = Math.sqrt(Math.pow(q[0]-tree.vertices[i].vertex[0],2)+Math.pow(q[2]-tree.vertices[i].vertex[2],2));
	}
		//console.log(distance);
	 nearest_index = distance.indexOf(Math.min(...distance));
//	console.log(nearest_index);

	return [nearest_index,tree.vertices[nearest_index].vertex]

}


function random_config() {

var q_rand = [];
//	if (robot_collision_forward_kinematics(q)){
//	var q_rand = [  [2*(Math.random()-0.5)*robot_boundary[0][0]],
//                [0],
//              [2*(Math.random()-0.5)*robot_boundary[0][2]],[0],[2*(Math.random()-0.5)*(Math.PI)],[0] ];
//

q_rand[0] =robot_boundary[0][0]+Math.random()*(robot_boundary[1][0]-robot_boundary[0][0]);
q_rand[1] = 0; 
q_rand[2] = robot_boundary[0][2]+Math.random()*(robot_boundary[1][2]-robot_boundary[0][2]); 
q_rand[3] = 0; 
q_rand[4] = 2*(Math.PI); 
q_rand[5] = 0; 



	for (x in robot.joints) {

      	if (typeof robot.joints[x].limit =='undefined')
	q_rand[q_names[x]]=2*(Math.PI);
	else
	q_rand[q_names[x]] = Math.random()*(robot.joints[x].limit.upper-robot.joints[x].limit.lower)+robot.joints[x].limit.lower;
//	robot.joints[x].limit[0]-robot.joints[x].limit[1]
	//q_new[q_names[x]]=2*(Math.random()-0.5)*(Math.PI); 
	}

	return q_rand

//	}

}

function rrt_connect(tree,q) {
//console.log("getting in");
	var s = rrt_extend(tree,q);
	while ( s == "advanced") {

//	console.log("while?");
	s = rrt_extend(tree,q);
//	console.log(s);	
	}
	return s
}



function find_path_2(TV,tree,motion_plan){

var k = motion_plan.length;

//console.log(k);
//if (Math.pow(TV.vertex[0]-tree.vertex[0],2) + Math.pow(TV.vertex[2]-tree.vertex[2],2)>0.1){
motion_plan.push(tree);
//console.log(motion_plan);
//}
if(Math.pow(TV.vertex[0]-tree.vertex[0],2) + Math.pow(TV.vertex[2]-tree.vertex[2],2)<0.001){
A = motion_plan;
//console.log(A);

return A;

}
find_path_2(TV,tree.edges[0],motion_plan);
}


function find_path_2_B(TVB,treeB,motion_planB){

var k = motion_planB.length;

//console.log(k);
//if (Math.pow(TVB.vertex[0]-treeB.vertex[0],2) + Math.pow(TVB.vertex[2]-treeB.vertex[2],2)>0.1){
motion_planB.push(treeB);
//console.log(motion_plan);
//}
if(Math.pow(TVB.vertex[0]-treeB.vertex[0],2) + Math.pow(TVB.vertex[2]-treeB.vertex[2],2)<0.001){
B = motion_planB;
//console.log(A);

return B;

}



find_path_2_B(TVB,treeB.edges[0],motion_planB);

}

function rrt_extend_2(tree){

var q_rand = random_config();
var q_nearest = nearest_neighbor(q_rand,tree);
var q_nearest_index=q_nearest[0];

var q_nearest = q_nearest[1];
var q_new = steer(q_nearest,q_rand);
//console.log("q_new");
//console.log(q_new);
      if (kineval.poseIsCollision(q_new)==false){
                var Q_near = near_vertices(q_new,tree);
//		console.log("Q_near");
//		console.log(Q_near);
                var q_min = choose_parent(Q_near,q_nearest,q_nearest_index,q_new,tree);
//		console.log("q_min");
//		console.log(q_min);
                var q_min_index = index_find(q_min,tree);
//		console.log("q_min_index");
//		console.log(q_min_index);
                tree_add_vertex(tree,q_new);
                tree_add_edge(tree,q_min_index,tree.newest);
                tree= rewire(tree,Q_near,q_min,q_new);

//              q_new_final_index = index_find(q_new,tree);

        }

        if ((kineval.poseIsCollision(q_new)==false)&&Math.sqrt(Math.pow(q_new[0]-q_goal[0],2)+Math.pow(q_new[2]-q_goal[2],2))<stepsize){
	console.log("if?");
	//return "reached"
        rrt_iterate =false;

//      tree_add_vertex(tree,q_new);
//      q_new_index = index_find(q_new,tree); //does not have a q_new index.. why??
                var Q_near = near_vertices(q_new,tree);
                var q_min = choose_parent(Q_near,q_nearest,q_nearest_index,q_new,tree);
                var q_min_index = index_find(q_min,tree);
         //     tree_add_vertex(tree,q_new);
                tree= rewire(tree,Q_near,q_min,q_new);
                tree_add_vertex(tree,q_new);
                tree_add_edge(tree,q_min_index,tree.newest);
                q_new_index = index_find(q_new,tree);
        var motion_plan =[];

        find_path(tree.vertices[q_new_index],motion_plan);


	}
	


}

function find_path(tree,motion_plan){
var k = motion_plan.length;
console.log("finnd");
motion_plan.push(tree);

//if (vector_distance(tree.vertex,q_init) < stepsize){

if (Math.sqrt(Math.pow(q_init[0]-tree.vertex[0],2)+Math.pow(q_init[2]-tree.vertex[2],2))<0.01){
        D = motion_plan;
        return;

        }
console.log(motion_plan);
find_path(tree.edges[0],motion_plan);

}

function index_find(q,tree){

        for ( var i=0 ; i<tree.vertices.length; i++){
//                if (vector_distance(q,tree.vertices[i].vertex)<0.001)
	if (Math.sqrt(Math.pow(q[0]-tree.vertices[i].vertex[0],2)+Math.pow(q[2]-tree.vertices[i].vertex[2],2))<0.001)
                return i
        }
}
function near_vertices(q,tree) {
        gamma = 10;
        var n = tree.vertices.length;
        var l = gamma*(Math.log(n)/n);
        var Z = [];
        var Q =[];
        for (var i=0;i<tree.vertices.length;i++) {

                if (i==0)
                Q.push(tree.vertices[0]);

//                else if (vector_distance(q,tree.vertices[i].vertex) <= l)
		else if (Math.sqrt(Math.pow(q[0]-tree.vertices[i].vertex[0],2)+Math.pow(q[2]-tree.vertices[i].vertex[2],2))<=l)
                Q.push(tree.vertices[i]);

        }
        //console.log(Q);
        return Q;

}
function steer(q1,q2) {

//        if (vector_distance(q1,q2) < stepsize)
	if (Math.sqrt(Math.pow(q1[0]-q2[0],2)+Math.pow(q1[2]-q2[2],2))<stepsize)

        return q2
        else{
      var theta = Math.atan2(q2[2]-q1[2],q2[0]-q1[0]);
	var q = [];
      q[0]= q1[0] + stepsize*Math.cos(theta);
      q[1] =q2[1];
      q[2]= q1[2] + stepsize*Math.sin(theta);
      q[3]= q2[3];	
      q[4]= q2[4];	
      q[5]= q2[5];	


	
		for (x in robot.joints) {

		q[q_names[x]]=q2[q_names[x]];
		}
        
	return q;
	}
}
function choose_parent(Q_near,q_nearest,q_nearest_index,q_new,tree){
        var q_min = q_nearest;
//        var c_min = vector_distance(q_new,tree.vertices[0].vertex) + tree.vertices[q_nearest_index].cost;
	var c_min =Math.sqrt(Math.pow(q_new[0]-tree.vertices[0].vertex[0],2)+Math.pow(q_new[2]-tree.vertices[0].vertex[2],2)) +tree.vertices[q_nearest_index].cost;
	

        for(var p=0;p<Q_near.length;p++){

                var q_prime = steer(Q_near[p].vertex,q_new);
               if (kineval.poseIsCollision(q_prime)==false && (q_prime == q_new)){
//                   if ((q_prime == q_new)){
                        //console.log("if");
//                        c_prime = Q_near[p].cost + vector_distance(q_new,tree.vertices[0].vertex);
			c_prime = Q_near[p].cost + Math.sqrt(Math.pow(q_new[0]-tree.vertices[0].vertex[0],2)+Math.pow(q_new[2]-tree.vertices[0].vertex[2],2)) +tree.vertices[q_nearest_index].cost;

                        if ((c_prime < tree.vertices[tree.newest].cost) && (c_prime < c_min))
                                        q_min = Q_near[p].vertex;
                }
        }

return q_min
}

function rewire(tree,Q_near,q_min,q_new){

        for (var i=0; i<Q_near.length;i++) {
        var q_prime = steer(q_new,Q_near[i].vertex);
//               if((kineval.poseIsCollision(q_prime)==false) && (q_prime == Q_near[i].vertex) &&(tree.vertices[tree.newest].cost+vector_distance(q_prime,tree.vertices[0].vertex)<Q_near[i].cost))
 
//	if((kineval.poseIsCollision(q_prime)==false) && (q_prime == Q_near[i].vertex) &&(tree.vertices[tree.newest].cost+vector_distance(q_prime,tree.vertices[0].vertex)<Q_near[i].cost))

if ((kineval.poseIsCollision(q_prime)==false) && (q_prime == Q_near[i].vertex) && (tree.vertices[tree.newest].cost +Math.sqrt(Math.pow(q_new[0]-tree.vertices[0].vertex[0],2)+Math.pow(q_new[2]-tree.vertices[0].vertex[2],2))<Q_near[i].cost))


         //        if((q_prime == Q_near[i].vertex) &&(tree.vertices[tree.newest].cost+vector_distance(q_prime,tree.vertices[0].vertex)<Q_near[i].cost))
                 Q_near[i].edges[0]=q_new;
        }

return tree



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










