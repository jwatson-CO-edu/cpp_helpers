/***********  
Cpp_Helpers.cpp
James Watson , 2017 March
Functions of general use for C++ programming

Template Version: 2017-04-13
***********/

#include "Cpp_Helpers.h"

// == Debug Tools ==

void assert_report( bool assertion , string report ) { // Reporting wrapper for 'assert'
	if ( !assertion ) { cout << endl << report << endl; } // if an assert will trigger, print BEFORE
	assert( assertion ); // crash if necessary
}

void sep_dbg(){ cout << "==================== Debug ====================" << endl; } // Print a separator for debug information

void sep( string title , size_t width , char dingbat ){
	// Print a separating title card for debug 
	cout << string( width , dingbat ) << " " << title << " " << string( width , dingbat ) << endl;
}

void newline(){ cout << endl; } // print a new line

// __ End Debug __


// == Math Tools ==

float rand_float(){ return (float)  rand() / (float)RAND_MAX; }

double rand_dbbl(){ return (double) rand() / (double)RAND_MAX; }

int randrange( int end ){ return (int)( rand() % end ); }
// NOTE: 'randrange' functions do not actually check if range bounds are in the proper order

int randrange( int bgn , int end ){ return bgn + (int)( rand() % ( end - bgn ) ); }

double randrange( double lo , double hi ){ return lo + (double) rand() * ( hi - lo ); }

// __ End Math __


// == File Tools ==

bool file_exists( const string& fName ){ // Return true if the file exists , otherwise return false
	struct stat buf; 
	if( stat( fName.c_str() , &buf ) != -1 ){ return true; } else { return false; }
}

std::vector<string> readlines( string path ){ // Return all the lines of text file as a string vector
	std::vector<string> rtnVec;
	if( file_exists( path ) ){
		ifstream fin( path ); // Open the list file for reading
		string line; // String to store each line
		while ( std::getline( fin , line ) ){ // While we are able to fetch a line
			rtnVec.push_back( line ); // Add the file to the list to read
		}
		fin.close();
	} else { cout << "readlines: Could not open the file " << path << endl; }
	return rtnVec;
}

// __ End File __


// == String Tools ==

void remove_all( string& rawStr , char keyChar ){ // Destructively remove all instances of 'keyChar' from 'rawStr'
	string::size_type i = 0; // http://stackoverflow.com/a/1488798/7186022
	while ( i < rawStr.length() ){ // While the end of the string has not been reached
		i = rawStr.find( keyChar , i ); // Assign the index to the next instance of 'keyChar'
		if ( i == string::npos ){ break; } // If we reached the end of the string , exit
		rawStr.erase( i ); // If an instance of the char has been found , erase it ( 'erase' resizes 'remove' doe not )
	} // If we are outside the loop , we have reached the end of the string
}

// Destructively remove all newlines from 'rawStr'
void strip_newlines( string& rawStr ){ remove_all( rawStr , '\r' ); remove_all( rawStr , '\n' ); }

string strip_after_dot( string fName ){ // Return a copy of 'fName' with the first period and all following characters removed
	int index = 0; string rtnStr = "";
	while( fName[index] != '.' && index < fName.size() ){ // While not period and string remains
		rtnStr += fName[index]; // add the char at index and increment index
		index++; 
	} 
	return rtnStr; // Return the transformed copy of the string
}

// __ End String __


// === Functors ===

// == class Incrementer ==

Incrementer::Incrementer( llin start ){ count = start; } // Create an incrementer with a starting number

llin Incrementer::operator()(){  count++;  return count;  } // Increment the counter and return it

// __ End Incrementer __

// ___ End Functors ___
