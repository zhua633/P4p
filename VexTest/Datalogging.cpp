#include <Windows.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string> 
#include <iomanip>

using namespace std;

FILE* myFile;
int RowNum = 0;
float datalog[8] = { 0 };
int columns = 0;
bool header = false;

void datalogClear() {
    //Opens and clears the file
    myFile = fopen("TestFile.txt", "w");
    if (myFile == NULL) {
        cout << "Failed to open file" << endl;
    }
}

void datalogDataGroupStart() {
    //writes the new row index to the file
   // fprintf(myFile, "%-i\t", RowNum);
}

void datalogAddValue(int Col, float data) {
    //writes the data into the column of the current row
   // fprintf(myFile, "%-5.5f\t\t", data);
    datalog[Col] = data;
  //  cout << datalog[Col]<< endl;
    if (Col > columns) {
        columns = Col;
    }
   // cout << "Columns = " << columns << endl;
}

void datalogDataGroupEnd() {
    //end line/row
    if (!header) {
        fprintf(myFile,"Row\t");
        for(int i = 0; i < columns + 1; i++) {
            fprintf(myFile, "Series %d\t", i);
        }
        header = true;
        fprintf(myFile, "\n");
    }
    fprintf(myFile, "%-i\t", RowNum);
    for (int i = 0; i < columns+1; i++) {
        fprintf(myFile, "%-5.5f\t\t", datalog[i]);
       // cout << datalog[i] <<"\t";
    }
    fprintf(myFile, "\n");
   // cout << "\n";
    RowNum++;
}


void datalogClose() {
    if (myFile != NULL) {
        fclose(myFile);
    }
}