/*

	Destination parser 

	Author: Rivu

*/

#include <vector>
#include <stdio.h>
#include <string>
#include <cstring>

#include <glm/glm.hpp>

bool loadLocation(const char * path, 
	std::vector<glm::vec3> &points
	)
{

	printf("Loading file %s...\n", path);

	FILE * file = fopen(path, "r");
	if( file == NULL ){
		printf("Impossible to open the file !\n");
		getchar();
		return false;
	}

	while (1)
	
	{	

		char lineHeader[128];
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF)
			break; // EOF = End Of File. Quit the loop.


		glm::vec3 vertex;
			fscanf(file, "%f %f %f \n", &vertex.x,&vertex.y,&vertex.z );
			//printf("%f %f %f \n", vertex.x,vertex.y,vertex.z);
			points.push_back(vertex);
			

	}










return true ;

}
