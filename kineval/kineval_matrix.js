//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}



function matrix_multiply(a, b) {
   var aNumRows = a.length, aNumCols = a[0].length,
       bNumRows = b.length, bNumCols = b[0].length,
       m = new Array(aNumRows);  // initialize array of rows
   for (var r = 0; r < aNumRows; ++r) {
     m[r] = new Array(bNumCols); // initialize the current row
     for (var c = 0; c < bNumCols; ++c) {       m[r][c] = 0;      // initialize the current cell
      for (var i = 0; i < aNumCols; ++i) {
       m[r][c] += a[r][i] * b[i][c];
      }
     }   
   }
  
   return m;
 }

function matrix_multiply_3(a, b, f) {
   var aNumRows = a.length, aNumCols = a[0].length,
       bNumRows = b.length, bNumCols = b[0].length,
       fNumRows = f.length, fNumCols = f[0].length,
       m = new Array(bNumRows);  // initialize array of rows
   for (var r = 0; r < bNumRows; ++r) {
     m[r] = new Array(fNumCols); // initialize the current row
     for (var c = 0; c < fNumCols; ++c) {       m[r][c] = 0;      // initialize the current cell
      for (var i = 0; i < bNumCols; ++i) {
       m[r][c] += b[r][i] * f[i][c];
      }
     }   
    }  
      mNumRows = m.length, mNumCols = m[0].length,
       
      n = new Array(aNumRows);

   for (var k = 0; k < aNumRows; ++k) {
	n[k] = new Array(mNumCols);
	for ( var l = 0; l <  mNumCols; ++l) {
	      n[k][l] = 0;
		for (var j = 0; j < aNumCols; ++j) {
		n[k][l] += a[k][j] * m[j][l];
		}
	}
   }
  return n;
 
}

  
 

function matrix_transpose(a) {
    var aNumRows = a.length, aNumCols = a[0].length
        m = new Array(aNumRows);
 for (var r = 0; r < aNumRows; ++r) {
       m[r] = new Array(aNumCols);
 for (var c = 0; c < aNumCols; ++c) {
       m[r][c]= a[c][r];
      }
     }
     return m;
}

function vector_cross(x,y){

result=[];
result[0]=x[1]*y[2]-x[2]*y[1];
result[1]=x[2]*y[0]-x[0]*y[2];
result[2]=x[0]*y[1]-x[1]*y[0];
return result;
} 

function vector_normalize(x){
norm=x[0]*x[0]+x[1]*x[1]+x[2]*x[2];
norm=Math.sqrt(norm);

x[0]=x[0]/norm;
x[1]=x[1]/norm;
x[2]=x[2]/norm;

return x;

}


 function generate_identity() {

 var identity = [
                     [1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]
 ];
 return identity; 
}

function generate_translation_matrix(x,y,z) {
 
var trans = [
                 [1,0,0,x],
                 [0,1,0,y],
                 [0,0,1,z],
                 [0,0,0,1]
];
 
return trans;
}


function generate_rotation_matrix_Z(x) {

var rot = [
                [Math.cos(x),-Math.sin(x),0,0],
                [Math.sin(x),Math.cos(x),0,0],
                [0,0,1,0],
                [0,0,0,1]
];
return rot;
}

function generate_rotation_matrix_Y(y) {

var rot_y = [
                [Math.cos(y),0,Math.sin(y),0],
                [0,1,0,0],
                [-Math.sin(y),0,Math.cos(y),0],
                [0,0,0,1]
];
return rot_y;
}

function generate_rotation_matrix_X(x) {

var rot_z = [
                [1,0,0,0],
                [0,Math.cos(x),-Math.sin(x),0],
                [0,Math.sin(x),Math.cos(x),0],
                [0,0,0,1]

];
return rot_z;
}


    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
    //   matrix_transpose
    //   matrix_pseudoinverse
    //   matrix_invert_affine
    //   vector_normalize
    //   vector_cross
    //   generate_identity
    //   generate_translation_matrix
    //   generate_rotation_matrix_X
    //   generate_rotation_matrix_Y
    //   generate_rotation_matrix_Z

