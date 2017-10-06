#pragma once

/***********  
Cpp_Helpers.h
James Watson , 2017 March
Shortcuts and Aliases , and Functions of general use for C++ programming

Template Version: 2017-05-26
***********/

#ifndef CPPHELPERS_H
#define CPPHELPERS_H

#include <string> // --- string manipulation
#include <cmath> // ---- abs, min/max, trig, hyperbolic, power, exp, error, rounding
#include <stdlib.h> // - srand, rand 
#include <time.h> // --- time , for getting sys time and seeding random numbers
#include <limits> // --- number limits of data types, limit on 'cin.ignore'
#include <cassert> // -- input/condition verification
//#define NDEBUG // ---- uncomment to disable assert()

#include <vector> // --- standard vector datatype , the friendly array } Data Structures
#include <list> // ----- standard list datatype                       /
#include <map> // ------ dictionaries                                /
#include <algorithm> //- Searching structures ______________________/

#include <iostream> // - standard input and output, istream } Input / Output
#include <fstream> // -- Reading files                     /
#include <sstream> // -- Text streams                     /
#include <sys/stat.h> // File status ____________________/



// == Shortcuts and Aliases ==
// ~ Namespaces ~
using std::cout; // --------------- output to terminal
using std::endl; // --------------- newline
using std::cin; // ---------------- input from terminal
using std::ifstream; // ----------- File Input streams
using std::ofstream; // ----------- File Output streams
using std::vector; // ------------- vectors
using std::list; // --------------- lists
using std::map; // ---------------- dictionaries
using std::string; // ------------- strings!           // Requires C++11
using std::to_string; // ---------- string conversion  // Requires C++11
// ~ Type Aliases ~
using usll = unsigned long long; // big ints           // Requires C++11
// ~ Constants ~
#define EPSILON 1e-8d // ---------- Margin too small to care about
// __ End Shortcuts __

/* These are very short functions, but putting them in a separate CPP anyway because .H files are text substitution and including in 
several places (likely) will define the functions multiple times, which will raise errors. */

// == Debug Tools ==
void assert_report( bool assertion , string report ); // Reporting wrapper for 'assert'
void sep_dbg(); // Print a separator for debug information
void sep( string title = "" , size_t width = 6 , char dingbat = '=' ); // Print a separating title card for debug 
void newline();// print a new line
// == End Debug ==

// == Math Tools ==

float rand_float(); // Note that this is not exactly uniformly distributed

int randrange( int end );

int randrange( int bgn , int end );

double randrange( double lo , double hi );

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool eq( T op1 , T op2 ){ return ( (double) abs( op1 - op2 ) ) < EPSILON; }

// __ End Math __

// == File Tools ==
bool file_exists( const string& fName );  // Return true if the file exists , otherwise return false
std::vector<string> readlines( string path ); // Return all the lines of text file as a string vector
// == End File ==

// == String Tools ==
void remove_all( string& rawStr , char keyChar ); // Destructively remove all instances of 'keyChar' from 'rawStr'
// Destructively remove all newlines from 'rawStr'
void strip_newlines( string& rawStr );
string strip_after_dot( string fName ); // Return a copy of 'fName' with the first period and all following characters removed
// == End String ==

// == Container Tools ==

template<typename T> // NOTE: Templated functions must have their definition in the header file
std::ostream& operator<<( std::ostream& os , const std::vector<T>& vec ) { // ostream '<<' operator for vectors
	// NOTE: This function assumes that the ostream '<<' operator for T has already been defined
	os << "[ ";
	for (size_t i = 0; i < vec.size(); i++) {
		os << vec[i];
		if (i + 1 < vec.size()) { os << ", "; }
	}
	os << " ]";
	return os; // You must return a reference to the stream!
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
vector<T> vec_range( T lo , T hi ){
	T i = 0;
	vector<T> rtnVec;
	if( lo < hi )
		for( i = lo ; i <= hi ; i++ ){ rtnVec.push_back( i ); }
	else
		for( i = hi ; i >= lo ; i-- ){ rtnVec.push_back( i ); }
	return rtnVec;
}


template<typename T> // NOTE: Templated functions must have their definition in the header file
list<T> lst_range( T lo , T hi ){
	T i = 0;
	list<T> rtnVec;
	if( lo < hi )
		for( i = lo ; i <= hi ; i++ ){ rtnVec.push_back( i ); }
	else
		for( i = hi ; i >= lo ; i-- ){ rtnVec.push_back( i ); }
	return rtnVec;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool is_arg_in_list( T arg , list<T> lst ){
	// Return true if 'arg' is in 'lst' , false otherwise
	return find( lst.begin( ) , lst.end() , arg ) != lst.end();
}

// == End Container ==




#endif

/* == Useful Parts =========================================================================================================================

// ~~ cout << operator ~~
std::ostream& operator<<(std::ostream& os, const Gene& vec) {
	std::vector<float> codons = vec.copy_codons();
	os << "[ ";
	for (size_t i = 0; i < codons.size(); i++) {
		os << codons[i];
		if (i + 1 < codons.size()) { os << ", "; }
	}
	os << " ]";
	return os; // You must return a reference to the stream!
}

// ~~ Function Object ~~
struct myclass {
  bool operator() (int i,int j) { return (i<j);}
} myobject; 

  == End Parts ==  */
