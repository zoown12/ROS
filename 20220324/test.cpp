#include <stdio.h>
#include <stdlib.h>

int main(int argc, char* argv[])
{
	 int i;  
  
        for (i = 0; i < argc; i++)  
                printf("argv[%d]: %s\n", i, argv[i]);  
	int a = atoi(argv[1]);
	int b = atoi(argv[2]);
	
	printf("%d + %d = %d \n",a,b,a+b);
	printf("%d - %d = %d \n",a,b,a-b);
	printf("%d * %d = %d \n",a,b,a*b);
	printf("%d / %d = %d \n",a,b,a/b);
 
 return 1;
 
}
