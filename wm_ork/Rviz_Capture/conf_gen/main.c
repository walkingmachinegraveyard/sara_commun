/*
 * main.c
 *
 *  Created on: Jun 25, 2016
 *      Author: samuel
 */

#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>

int main (int argc, char **argv){

	/*Lecture du nombre d'image dans le dossier***************/
	int file_count = 0;
	DIR * dirp;
	struct dirent * entry;

	dirp = opendir("/home/samuel/sam_temp/snapshots"); /* There should be error handling after this */
	while ((entry = readdir(dirp)) != NULL) {
	    if (entry->d_type == DT_REG) { /* If the entry is a regular file */
	         file_count++;
	    }
	}
	closedir(dirp);
	printf("\nNumber of images : %d\n",file_count);
	/*********************************************************/

	/*Écriture du config file pour le PDF_Generator***********/
	FILE * fHandle = NULL;
	int i =0;

	fHandle = fopen("/home/samuel/sam_temp/pdf_config.conf","w");
	if(fHandle != NULL){
		printf("File created\n");
	} else {
		printf("Error Creating the file\n");
	}

	fprintf(fHandle,"/home/samuel/sam_temp\n\n");
	fprintf(fHandle,"1 \n\n");
	fprintf(fHandle,"1 \n\n");
	fprintf(fHandle,"The STAGE\n\n");

	if(file_count > 0){
		fprintf(fHandle,"/home/samuel/sam_temp/snapshots/camera.png\n");
		fprintf(fHandle,"Screenshot : 1 \n");
		if(file_count > 1){
			while(i<file_count){
				fprintf(fHandle,"/home/samuel/sam_temp/snapshots/camera-%d.png\n",i);
				fprintf(fHandle,"Screenshot : %d \n",i+1);
				i++;
			}
		}
	} else {
		fprintf(fHandle,"No image recorded\n");
	}

	fclose(fHandle);
	/*********************************************************/
	return 0;
}




/*

# Main title
The Mega Stage!
# Here you can put as many image as needed
# /path/to/image.png or/path/to/image.jpeg
# image title
/home/alexandre/Pictures/ets.png
ETS logo

}
*/
