
#ifndef MYCL_HPP_
#define MYCL_HPP_


#ifdef __APPLE__
	#include <OpenCL/opencl.h>
#else
	#include <CL/cl.h>
#endif



#define VECTOR_SIZE 1024


//#ifdef cl_khr_fp64
//    #pragma OPENCL EXTENSION cl_khr_fp64 : enable
//#elif defined(cl_amd_fp64)
//    #pragma OPENCL EXTENSION cl_amd_fp64 : enable
//#else
//    #error "double type not supported by OpenCL implementation."
//#endif










const char *saxpy_kernel =
"__kernel \n"
"void saxpy_kernel(float alpha,	 			\n"
"	__global float *A,				 		\n"
" 	__global float *B,				 		\n"
" 	__global float *C)				 		\n"
"{ 											\n"
" //Get the index of the work-item			\n"
" 	int index = get_global_id(0); 			\n"
" 	C[index] = alpha* A[index] + B[index];  \n"
"} 											\n";






void print_device_info(cl_device_id device)
{
	char queryBuffer[1024];
	int queryInt;
	cl_int clError;
	clError = clGetDeviceInfo(device, CL_DEVICE_NAME,
	sizeof(queryBuffer),
	&queryBuffer, NULL);
	printf("\tCL_DEVICE_NAME: %s\n", queryBuffer);
	queryBuffer[0] = '\0';
	clError = clGetDeviceInfo(device, CL_DEVICE_VENDOR,
	sizeof(queryBuffer), &queryBuffer,
	NULL);
	printf("\tCL_DEVICE_VENDOR: %s\n", queryBuffer);
	queryBuffer[0] = '\0';
	clError = clGetDeviceInfo(device, CL_DRIVER_VERSION,
	sizeof(queryBuffer), &queryBuffer,
	NULL);
	printf("\tCL_DRIVER_VERSION: %s\n", queryBuffer);
	queryBuffer[0] = '\0';
	clError = clGetDeviceInfo(device, CL_DEVICE_VERSION,
	sizeof(queryBuffer), &queryBuffer,
	NULL);
	printf("\tCL_DEVICE_VERSION: %s\n", queryBuffer);
	queryBuffer[0] = '\0';
	clError = clGetDeviceInfo(device, CL_DEVICE_MAX_COMPUTE_UNITS,
	sizeof(int), &queryInt, NULL);
	printf("\tCL_DEVICE_MAX_COMPUTE_UNITS: %d\n", queryInt);
	fflush( stdout );
}




void CL_info()
{
	cl_platform_id* platforms = NULL;
	cl_uint num_platforms;
	cl_int clStatus = clGetPlatformIDs(0, NULL, &num_platforms);
	platforms = (cl_platform_id*)malloc(sizeof(cl_platform_id) * num_platforms);


	clStatus = clGetPlatformIDs(num_platforms, platforms, NULL);
	//get device from the list
	cl_device_id* device_list = NULL;
	cl_uint num_devices;

	for(cl_uint m=0; m<num_platforms; m++)
	{
		clStatus = clGetDeviceIDs(platforms[m], CL_DEVICE_TYPE_GPU, 0, NULL, &num_devices);
		device_list = (cl_device_id *) malloc(sizeof(cl_device_id) * num_devices);
		clStatus = clGetDeviceIDs(platforms[m], CL_DEVICE_TYPE_GPU, num_devices, device_list, NULL);
		//---- print OpenCL device information ---//
		printf("platform[%d]\n", m);
		for(cl_uint i=0; i<num_devices; i++)
			print_device_info( device_list[i] );
	}

}



#define MEM_SIZE (128)
#define MAX_SOURCE_SIZE (0x100000)



int init_CL()
{
	cl_device_id device_id = NULL;
	cl_context context = NULL;
	cl_command_queue command_queue = NULL;
	cl_mem memobj = NULL;
	cl_program program = NULL;
	cl_kernel kernel = NULL;
	cl_platform_id platform_id = NULL;
	cl_uint ret_num_devices;
	cl_uint ret_num_platforms;
	cl_int ret;

	char string[MEM_SIZE];

	FILE *fp;
	char filename[] = "./src/kernel.cl";
	char *source_str;
	size_t source_size;

	/* Load the source code containing the kernel*/
	fp = fopen(filename, "r");
	if(!fp)
	{
		fprintf(stderr, "cannot find file %s\n", filename);
		exit(1);
	}

	source_str = (char*)malloc(MAX_SOURCE_SIZE);
	source_size = fread(source_str, 1, MAX_SOURCE_SIZE, fp);
	fclose(fp);


	/* Get Platform and Device Info */
	ret = clGetPlatformIDs(1, &platform_id, &ret_num_platforms);
	ret = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_DEFAULT, 1, &device_id, &ret_num_devices);

	/* Create OpenCL context */
	context = clCreateContext(NULL, 1, &device_id, NULL, NULL, &ret);

	/* Create Command Queue */
	command_queue = clCreateCommandQueue(context, device_id, 0, &ret);

	/* Create Memory Buffer */
	memobj = clCreateBuffer(context, CL_MEM_READ_WRITE,MEM_SIZE * sizeof(char), NULL, &ret);

	/* Create Kernel Program from the source */
	program = clCreateProgramWithSource(context, 1, (const char **)&source_str,(const size_t *)&source_size, &ret);

	/* Create Memory Buffer */
	memobj = clCreateBuffer(context, CL_MEM_READ_WRITE,MEM_SIZE * sizeof(char), NULL, &ret);

	/* Create Kernel Program from the source */
	program = clCreateProgramWithSource(context, 1, (const char **)&source_str,(const size_t *)&source_size, &ret);

	/* Build Kernel Program */
	ret = clBuildProgram(program, 1, &device_id, NULL, NULL, NULL);

	/* Create OpenCL Kernel */
	kernel = clCreateKernel(program, "hello", &ret);
	printf("kernel ret: %d", ret ); fflush( stdout );
	assert( ret == 0 );


	/* Set OpenCL Kernel Parameters */
	ret = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void *)&memobj);

	/* Execute OpenCL Kernel */
	ret = clEnqueueTask(command_queue, kernel, 0, NULL,NULL);

	/* Copy results from the memory buffer */
	ret = clEnqueueReadBuffer(command_queue, memobj, CL_TRUE, 0,
	MEM_SIZE * sizeof(char),string, 0, NULL, NULL);

	/* Display Result */
	puts(string);

	/* Finalization */
	ret = clFlush(command_queue);
	ret = clFinish(command_queue);
	ret = clReleaseKernel(kernel);
	ret = clReleaseProgram(program);
	ret = clReleaseMemObject(memobj);
	ret = clReleaseCommandQueue(command_queue);
	ret = clReleaseContext(context);

	free(source_str);

	return 0;
}



const char* cl_err_string(cl_int error)
{
    static const char* strings[] =
    {
        // Error Codes
          "CL_SUCCESS"                                  //   0
        , "CL_DEVICE_NOT_FOUND"                         //  -1
        , "CL_DEVICE_NOT_AVAILABLE"                     //  -2
        , "CL_COMPILER_NOT_AVAILABLE"                   //  -3
        , "CL_MEM_OBJECT_ALLOCATION_FAILURE"            //  -4
        , "CL_OUT_OF_RESOURCES"                         //  -5
        , "CL_OUT_OF_HOST_MEMORY"                       //  -6
        , "CL_PROFILING_INFO_NOT_AVAILABLE"             //  -7
        , "CL_MEM_COPY_OVERLAP"                         //  -8
        , "CL_IMAGE_FORMAT_MISMATCH"                    //  -9
        , "CL_IMAGE_FORMAT_NOT_SUPPORTED"               //  -10
        , "CL_BUILD_PROGRAM_FAILURE"                    //  -11
        , "CL_MAP_FAILURE"                              //  -12

        , ""    //  -13
        , ""    //  -14
        , ""    //  -15
        , ""    //  -16
        , ""    //  -17
        , ""    //  -18
        , ""    //  -19

        , ""    //  -20
        , ""    //  -21
        , ""    //  -22
        , ""    //  -23
        , ""    //  -24
        , ""    //  -25
        , ""    //  -26
        , ""    //  -27
        , ""    //  -28
        , ""    //  -29

        , "CL_INVALID_VALUE"                            //  -30
        , "CL_INVALID_DEVICE_TYPE"                      //  -31
        , "CL_INVALID_PLATFORM"                         //  -32
        , "CL_INVALID_DEVICE"                           //  -33
        , "CL_INVALID_CONTEXT"                          //  -34
        , "CL_INVALID_QUEUE_PROPERTIES"                 //  -35
        , "CL_INVALID_COMMAND_QUEUE"                    //  -36
        , "CL_INVALID_HOST_PTR"                         //  -37
        , "CL_INVALID_MEM_OBJECT"                       //  -38
        , "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR"          //  -39
        , "CL_INVALID_IMAGE_SIZE"                       //  -40
        , "CL_INVALID_SAMPLER"                          //  -41
        , "CL_INVALID_BINARY"                           //  -42
        , "CL_INVALID_BUILD_OPTIONS"                    //  -43
        , "CL_INVALID_PROGRAM"                          //  -44
        , "CL_INVALID_PROGRAM_EXECUTABLE"               //  -45
        , "CL_INVALID_KERNEL_NAME"                      //  -46
        , "CL_INVALID_KERNEL_DEFINITION"                //  -47
        , "CL_INVALID_KERNEL"                           //  -48
        , "CL_INVALID_ARG_INDEX"                        //  -49
        , "CL_INVALID_ARG_VALUE"                        //  -50
        , "CL_INVALID_ARG_SIZE"                         //  -51
        , "CL_INVALID_KERNEL_ARGS"                      //  -52
        , "CL_INVALID_WORK_DIMENSION"                   //  -53
        , "CL_INVALID_WORK_GROUP_SIZE"                  //  -54
        , "CL_INVALID_WORK_ITEM_SIZE"                   //  -55
        , "CL_INVALID_GLOBAL_OFFSET"                    //  -56
        , "CL_INVALID_EVENT_WAIT_LIST"                  //  -57
        , "CL_INVALID_EVENT"                            //  -58
        , "CL_INVALID_OPERATION"                        //  -59
        , "CL_INVALID_GL_OBJECT"                        //  -60
        , "CL_INVALID_BUFFER_SIZE"                      //  -61
        , "CL_INVALID_MIP_LEVEL"                        //  -62
        , "CL_INVALID_GLOBAL_WORK_SIZE"                 //  -63
        , "CL_UNKNOWN_ERROR_CODE"
    };

    if (error >= -63 && error <= 0)
         return strings[-error];
    else
         return strings[64];
}




#endif
