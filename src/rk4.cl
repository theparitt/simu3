
__kernel 
void hello(global const float* X, global float* X_next )
{ 													
	int global_id = get_global_id(0);
};







kernel void rk4_solve(float t, float h, global const float* X, global float* X_next)
{ 													
	int gid = get_global_id(0);
	X_next[gid] = X[gid];
};




