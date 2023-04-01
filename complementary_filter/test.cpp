#include <algorithm>
#include<stdio.h>
#include<wchar.h>
#include<stdlib.h>
#include<time.h>
#include<io.h>
#include <string> 
#include <iostream> 
#include <fstream>
#include <time.h> 
using namespace std;

#define MAX_LINE 3000
void f(string inputfile,string outputfile)
{
 
	FILE *fp = NULL;
	char line[MAX_LINE];
 
	
	ofstream outedit(outputfile);
 
	if((fp = fopen(inputfile.c_str(),"r")) != NULL) //string 类为我们提供了一个转换函数 c_str()，
    //该函数能够将 string 字符串转换为C风格的字符串，并返回该字符串的 const 指针（const char*）
	{
		char delims[] = ",";
		int a,b,c;
		while (fgets(line, MAX_LINE, fp))
		{
			
			a= atoi(strtok( line, delims ));		
			b= atoi(strtok( NULL, delims ));
			c= atoi(strtok( NULL, delims ));
			
            outedit<<a+10<<','<<b+100<<','<<c+1000<<endl;
					 
		}
		fclose(fp);
		fp = NULL;
	}
}
 
 
 
int main(int argc, char* argv[])
{
	string inputfile;
	string outputfile;
	inputfile="../data/test1.csv";
	outputfile=inputfile.substr(0,strlen(inputfile.c_str())-4)+"_out.csv";
	f(inputfile,outputfile);
	system("pause");
	return 0;
}