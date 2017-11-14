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
#include <cstdio> // --- printf
#include <cmath> // ---- abs, min/max, trig, hyperbolic, power, exp, error, rounding
#include <limits> // --- Infinity
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

// ~ Type Shortcuts ~ // This is only for names that are unlikely to be shadowed
using std::cout; // --------------- output to terminal
using std::endl; // --------------- newline
using std::cin; // ---------------- input from terminal
using std::ifstream; // ----------- File Input streams
using std::ofstream; // ----------- File Output streams
using std::ostream; // ------------ Output streams
using std::string; // ------------- strings!           // Requires C++11
using std::to_string; // ---------- string conversion  // Requires C++11
using std::min; // ---------------- 'min' function
using std::isnan; // -------------- NaN Test
using std::abs; // ---------------- Absolute value
using std::printf; // ------------- Our fave printing function from C

// ~ Type Aliases ~ // Use this for long type names and names that are likley to be shadowed
using usll = unsigned long long; // big ints ( unsigned ) // Requires C++11
using llin = long long int; // ---- big ints              // Requires C++11

// ~ Constants ~
#define EPSILON 1e-8d // ---------- Margin too small to care about
double const INFTY_D = std::numeric_limits<double>::infinity();

// __ End Shortcuts __

/* These are very short functions, but putting them in a separate CPP anyway because .H files are text substitution and including in 
several places (likely) will define the functions multiple times, which will raise errors. */


// == Debug Tools ==

void assert_report( bool assertion , string report ); // Reporting wrapper for 'assert'

void sep_dbg(); // Print a separator for debug information

void sep( string title = "" , size_t width = 6 , char dingbat = '=' ); // Print a separating title card for debug 

void newline(); //  print a new line

// __ End Debug __


// == Math Tools ==

float rand_float(); // Note that this is not exactly uniformly distributed

int randrange( int end );

int randrange( int bgn , int end );

double randrange( double lo , double hi );

template<typename T> // NOTE: Templated functions must have their definition in the header file

bool eq( T op1 , T op2 ){ return ( (double) abs( op1 - op2 ) ) < EPSILON; }

usll tri_num( usll n );
size_t tri_num( size_t n );

double round_zero( double num );

template <typename T> 
int sign( T val ) { return ( T(0) < val ) - ( val < T(0) ); } // Return the sign if the number: -1 for val<0 , 0 for val==0 , 1 for val>0

// __ End Math __


// == File Tools ==

bool file_exists( const string& fName );  // Return true if the file exists , otherwise return false

std::vector<string> readlines( string path ); // Return all the lines of text file as a string vector

// __ End File __


// == String Tools ==

void remove_all( string& rawStr , char keyChar ); // Destructively remove all instances of 'keyChar' from 'rawStr'

// Destructively remove all newlines from 'rawStr'
void strip_newlines( string& rawStr );

string strip_after_dot( string fName ); // Return a copy of 'fName' with the first period and all following characters removed

// __ End String __


// == Container Tools ==

template<typename T> // NOTE: Templated functions must have their definition in the header file
ostream& operator<<( ostream& os , const std::vector<T>& vec ) { // ostream '<<' operator for vectors
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
std::vector<T> vec_range( T lo , T hi ){
	T i = 0;
	std::vector<T> rtnVec;
	if( lo < hi )
		for( i = lo ; i <= hi ; i++ ){ rtnVec.push_back( i ); }
	else
		for( i = hi ; i >= lo ; i-- ){ rtnVec.push_back( i ); }
	return rtnVec;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
T min_num_in_vec( std::vector<T> searchVec ){
	//~ T      least   = std::numeric_limits<T>::infinity(); // THIS IS RETURNING ZERO
	T      least   = std::numeric_limits<T>::max(); // Get the largest representable number
	size_t i       = 0                , 
	       numElem = searchVec.size() ;
	//~ cout << "DEBUG: Starting with infinity: " << least << endl;
	for( i = 0 ; i < numElem ; i++ ){ 
		least = min( searchVec[i] , least ); 
		//~ cout << searchVec[i] << " , ";
	} 
	//~ cout << endl;
	return least;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
std::list<T> lst_range( T lo , T hi ){
	T i = 0;
	std::list<T> rtnVec;
	if( lo < hi )
		for( i = lo ; i <= hi ; i++ ){ rtnVec.push_back( i ); }
	else
		for( i = hi ; i >= lo ; i-- ){ rtnVec.push_back( i ); }
	return rtnVec;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool is_arg_in_list( T arg , std::list<T> lst ){
	// Return true if 'arg' is in 'lst' , false otherwise
	return find( lst.begin() , lst.end() , arg ) != lst.end();
}

// __ End Container __


// === Functors ===

// == class Incrementer ==

class Incrementer{
public:
	Incrementer( llin start = 0 );
	llin operator()();
protected:
	llin count;
};

// __ End Incrementer __

// ___ End Functors ___


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

  __ End Parts __  */
