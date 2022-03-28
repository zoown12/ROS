#include <stdio.h>

int main(int argc, char* argv[])
{
 int num1,num2;
 printf("Enter num");
 scanf("%d %d", &num1,&num2);
 
 printf("%d + %d = %d\n", num1,num2,num1+num2 );
 printf("%d - %d = %d\n", num1,num2,num1-num2 );
 printf("%d * %d = %d\n", num1,num2,num1*num2 );
 printf("%d / %d = %d\n", num1,num2,num1/num2 );
 
 
 printf("argc = %d\n",argc);
 
 printf(" %s\n",argv[0]);
 
 return 1;
 
}
