/*
 * Utility.cpp
 *
 *  Created on: Jul 4, 2017
 *      Author: vale
 */

#include "Utility.hpp"



bool readFile(std::string name_file,std::vector< std::vector<double> > & data){
	std::ifstream infile(name_file);
	std::string line;
	double number;
	while (std::getline(infile, line)){
	  std::istringstream iss(line);
	  std::istringstream iss1(line);
	  std::vector<double> myNumbers;

	  int a, b;
	  if (!(iss >> a >> b)){ break; } // error
	  // TODEBUG
	  //std::cout << line<<std::endl;
	  //---
	  while ( iss1 >> number ){
		  myNumbers.push_back( number );
	  }
	  /*app << myNumbers[0],myNumbers[1],myNumbers[2],myNumbers[3],myNumbers[4],
			 myNumbers[5],myNumbers[6],myNumbers[7],myNumbers[8],myNumbers[9],
			 myNumbers[10],myNumbers[11],myNumbers[12],myNumbers[13],myNumbers[14],
			 myNumbers[15],myNumbers[16],myNumbers[17],myNumbers[18],myNumbers[19],
			 myNumbers[20],myNumbers[21],myNumbers[22],0.0;*/

	  // TODEBUG
	  /*for(int ii=0; ii<(int)myNumbers.size();ii++){
		  std::cout << myNumbers[ii] <<" ";
	  }
	  std::cout<<std::endl;
	  std::cout<<std::endl;*/
	  //---
	  data.push_back(myNumbers);

	}
	return true;
}
