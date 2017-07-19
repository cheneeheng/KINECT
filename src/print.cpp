/*
 * print.cpp
 *
 *  Created on: Mar 24, 2017
 *      Author: chen
 *      Detail: Prints output.
 */

#include "print.h"

int printer(
		int x_)
{
	if (1)
		switch (x_)
		{
			case 1:
				printf(
						"# Initialization.......................................................SUCCESS\n");
				break;
			case 2:
				printf(
						"# Reading action labels................................................SUCCESS\n");
				break;
			case 3:
				printf(
						CRED "# Reading action labels.................................................FAILED\n" CNOR);
				break;
			case 4:
				printf(
						"# Reading object specific action labels................................SUCCESS\n");
				break;
			case 5:
				printf(
						CRED "# Reading object specific action labels.................................FAILED\n" CNOR);
				break;
			case 6:
				printf(
						"# Reading information about location areas.............................SUCCESS\n");
				break;
			case 7:
				printf(
						CYEL "# No information about location areas is found................................\n" CNOR);
				break;
			case 8:
				printf(
						"# Reading data.........................................................SUCCESS\n");
				break;
			case 9:
				printf(
						"# Parsing data.........................................................SUCCESS\n");
				break;
			case 10:
				printf(
						"# Pre-processing data..................................................SUCCESS\n");
				break;
			case 11:
				printf(
						"# Finding location areas...............................................SUCCESS\n");
				break;
			case 12:
				printf(
						"# Building sector-map..................................................SUCCESS\n");
				break;
			case 13:
				printf(
						"# Clustering of data with DBSCAN.......................................SUCCESS\n");
				break;
			case 14:
				printf(
						"# Combining nearby clusters............................................SUCCESS\n");
				break;
			case 15:
				printf(
						CYEL "# Labeling clusters (location areas)..........................................\n" CNOR);
				break;
			case 16:
				printf(
						"# Labeling clusters (location areas)...................................SUCCESS\n");
				break;
			case 17:
				printf(
						"# Building nodes (location areas)......................................SUCCESS\n");
				break;
			case 18:
				printf(
						"# Fitting curve........................................................SUCCESS\n");
				break;
			case 19:
				printf(
						"# Adjusting sector map.................................................SUCCESS\n");
				break;
			case 20:
				printf(
						"# Fitting points to sector map.........................................SUCCESS\n");
				break;
			case 21:
				printf(
						"# Checking constraint..................................................SUCCESS\n");
				break;
			case 22:
				printf(
						"# Fitting points to initial sector map.................................SUCCESS\n");
				break;
			case 23:
				printf(
						CYEL "# Deleting clusters (location areas)..........................................\n" CNOR);
				break;
			case 24:
				printf(
						CRED "# Data is empty...............................................................\n" CNOR);
				printf(
						CRED "# Reading data..........................................................FAILED\n" CNOR);
				break;
			case 25:
				printf(
						CRED "# Folder with data is missing.................................................\n" CNOR);
				printf(
						CRED "# Reading data folder...................................................FAILED\n" CNOR);
				break;
			case 26:
				printf(
						"# Reading data folder..................................................SUCCESS\n");
				break;
			case 27:
				printf(
						CRED "# Individual folder with data is missing......................................\n" CNOR);
				printf(
						CRED "# Reading individual data folders.......................................FAILED\n" CNOR);
				break;
			case 28:
				printf(
						"# Reading individual data folders......................................SUCCESS\n");
				break;
			case 29:
				printf(
						CRED "# Learning process......................................................FAILED\n" CNOR);
				break;
			case 30:
				printf(
						CGRN "# Learning process.....................................................SUCCESS\n" CNOR);
				break;
			case 31:
				printf(
						CYEL "# Current action is creating a huge sector map, action will be ignored........\n" CNOR);
				break;
			case 32:
				printf(
						CRED "# Subject folder is missing...................................................\n" CNOR);
				printf(
						CRED "# Reading subject folder................................................FAILED\n" CNOR);
				break;
			case 33:
				printf(
						"# Reading subject folder...............................................SUCCESS\n");
				break;
			case 34:
				printf(
						CRED "# Label is missing............................................................\n" CNOR);
				printf(
						CRED "# Reading label.........................................................FAILED\n" CNOR);
				break;
			case 35:
				printf(
						"# Reading label........................................................SUCCESS\n");
				break;
			case 36:
				printf(
						CRED "# Label is empty..............................................................\n" CNOR);
				printf(
						CRED "# Reading label file....................................................FAILED\n" CNOR);
				break;
			case 37:
				printf(
						"# Reading label file...................................................SUCCESS\n");
				break;
			case 38:
				printf(
						CRED "# Surface file is empty.......................................................\n" CNOR);
				printf(
						CRED "# Reading surface file..................................................FAILED\n" CNOR);
				break;
			case 39:
				printf(
						"# Reading surface file.................................................SUCCESS\n");
				break;
			case 40:
				printf(
						"# Surface contact check................................................SUCCESS\n");
				break;
			case 41:
				printf(
						"# Reading saved data...................................................SUCCESS\n");
				break;
			case 42:
				printf(
						CRED "# No saved data found.........................................................\n" CNOR);
				printf(
						CRED "# Reading saved data....................................................FAILED\n" CNOR);
				break;
			case 43:
				printf(
						CRED "# Testing process.......................................................FAILED\n" CNOR);
				break;
			case 44:
				printf(
						CGRN "# Testing process......................................................SUCCESS\n" CNOR);
				break;
			case 45:
				printf(
						CRED "# Surface limit file is empty.................................................\n" CNOR);
				printf(
						CRED "# Reading surface limit file............................................FAILED\n" CNOR);
				break;
			case 46:
				printf(
						"# Reading surface limit file...........................................SUCCESS\n");
				break;
			case 47:
				printf(
						"# Reading reference points for LA......................................SUCCESS\n");
				break;
			case 48:
				printf(
						"# Rewriting data file with label.......................................SUCCESS\n");
				break;
			case 49:
				printf(
						CRED "# Object label file is empty..................................................\n" CNOR);
				break;
			case 50:
				printf(
						"# Reading object label file............................................SUCCESS\n");
				break;
			case 51:
				printf(
						CRED "# Object transition file is empty.............................................\n" CNOR);
				break;
			case 52:
				printf(
						"# Reading object transition file.......................................SUCCESS\n");
				break;
			case 53:
				printf(
						CRED "# Object-LA file is empty.....................................................\n" CNOR);
				break;
			case 54:
				printf(
						"# Reading object-LA file...............................................SUCCESS\n");
				break;
			default:
				printf(
						CRED "# UNKNOWN COMMAND.............................................................\n" CNOR);
				break;

		}
	return EXIT_SUCCESS;
}

