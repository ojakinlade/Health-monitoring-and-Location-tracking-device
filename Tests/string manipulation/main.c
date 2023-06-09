#include <stdio.h>
#include <stdlib.h>

char dataToSend[100] = "25.03,3.90,7.44";
char temp[10];
char lng[10];
char lat[10];
int pos1;
int pos2;

// Function to find the positions of commas in a string
void FindCommaPositions(char* str, int* pos1, int* pos2)
{
    for(int i = 0; i < strlen(str); i++)
    {
        if(str[i] == ',')
        {
            *pos1 = i;
            break;
        }
    }
    for(int j = *pos1 + 1; j < strlen(str); j++)
    {
        if(str[j] == ',')
        {
            *pos2 = j;
            break;
        }
    }
}

/**
* @brief Function to extract and store values between commas in separate arrays
* @param str The input string
* @param temp The array to store the value before the first comma
* @param lng The array to store the value between the first and second commas
* @param lat The array to store the value after the second comma
* @param pos1 The position of the first comma
* @param pos2 The position of the second comma
*/
void ParseData(char* str, char* temp, char* lng, char* lat,int pos1,int pos2)
{
    int len = strlen(str);
    for(int i = 0; i < pos1; i++)
    {
        temp[i] = str[i];
    }
    for(int j = pos1 + 1; j < pos2; j++)
    {
        lng[j-(pos1 + 1)] = str[j];
    }
    for(int k = pos2 + 1; k < len; k++)
    {
        lat[k-(pos2 + 1)] = str[k];
    }
}

int main()
{
    printf("%d\n",strlen(dataToSend));
    FindCommaPositions(dataToSend,&pos1,&pos2);
    ParseData(dataToSend,temp,lng,lat,pos1,pos2);
    printf("Temp:%s\n",temp);
    printf("Lng:%s\n",lng);
    printf("Lat:%s\n",lat);

    return 0;
}


