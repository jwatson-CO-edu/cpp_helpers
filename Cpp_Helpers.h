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
#include <set> // ------ sets                                       /
#include <algorithm> //- Searching structures _____________________/

#include <iostream> // - standard input and output , istream } Input / Output
#include <fstream> // -- Reading files                      /
#include <sstream> // -- Text streams                      /
#include <sys/stat.h> // File status _____________________/


// == Shortcuts and Aliases ==

// ~ Standard Shortcuts ~ // This is only for names that are unlikely to be shadowed
using std::cout; // --------------- output to terminal
using std::endl; // --------------- newline
using std::cin; // ---------------- input from terminal
using std::ifstream; // ----------- File Input streams
using std::ofstream; // ----------- File Output streams
using std::ostream; // ------------ Output streams
using std::string; // ------------- strings!           // Requires C++11
using std::to_string; // ---------- string conversion  // Requires C++11
using std::min; // ---------------- 'min' function
using std::max; // ---------------- 'max' function
using std::isnan; // -------------- NaN Test
using std::abs; // ---------------- Absolute value
using std::printf; // ------------- Our fave printing function from C

// ~ Type Aliases ~ // Use this for long type names and names that are unlikley to be shadowed
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

size_t randrange( size_t end );

int randrange( int bgn , int end );

double randrange( double lo , double hi );

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool eq( T op1 , T op2 ){ return ( (double) abs( op1 - op2 ) ) < EPSILON; }

template < typename T , typename U >
bool eq( T op1 , U op2 ){ return ( abs( (double)op1 - (double)op2 ) ) < EPSILON; }

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
	T      least   = searchVec[0]; 
	size_t i       = 0                , 
	       numElem = searchVec.size() ;
	for( i = 0 ; i < numElem ; i++ ){ 
		least = min( searchVec[i] , least ); 
	} 
	return least;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
T max_num_in_vec( std::vector<T> searchVec ){
	T      most    = searchVec[0]; 
	size_t i       = 0                , 
	       numElem = searchVec.size() ;
	for( i = 0 ; i < numElem ; i++ ){ 
		most = max( searchVec[i] , most ); 
	} 
	return most;
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

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool is_arg_in_set( T arg , std::set<T> st ){
	// Return true if 'arg' is in 'st' , false otherwise
	// URL , C++ cannot recognize a templated typename
	typename std::set<T>::iterator it = st.find( arg );
	return it != st.end();
}

template<typename T>
T rand_choice( std::vector<T> searchVec ){ return searchVec[ randrange( searchVec.size() ) ]; }

std::vector<bool> bool_false_vector( size_t length );

size_t random_false_elem_index( std::vector<bool> vec );

//  Return the 'i'th index of 'iterable', wrapping to index 0 at all integer multiples of 'len' , Wraps forward and backwards , Python Style
llin indexw( llin len , llin i );

std::ostream& operator<<( std::ostream& os , const std::set<int>& elemSet );

template<typename T>
std::vector<T> vec_copy( std::vector<T>& original ){
	size_t len = original.size();
	std::vector<T> rtnVec;
	for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( original[i] );  }
	return rtnVec;
}

template<typename T>
std::vector<T> vec_copy( const std::vector<T>& original ){
	size_t len = original.size();
	std::vector<T> rtnVec;
	for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( original[i] );  }
	return rtnVec;
}

template<typename T>
std::vector<std::vector<T>> vec_vec_copy( std::vector<std::vector<T>>& original ){
	size_t len_i , len_j;
	std::vector<std::vector<T>> rtnVecVec;
	len_i = original.size();
	for( size_t i = 0 ; i < len_i ; i++ ){
		len_j = original[i].size();
		std::vector<T> temp;
		for( size_t j = 0 ; j < len_j ; j++ ){
			temp.push_back( original[i][j] );
		}
		rtnVecVec.push_back( temp );
	}
	return rtnVecVec;
}

template<typename T>
std::vector<std::vector<T>> vec_vec_copy( const std::vector<std::vector<T>>& original ){
	size_t len_i , len_j;
	std::vector<std::vector<T>> rtnVecVec;
	len_i = original.size();
	for( size_t i = 0 ; i < len_i ; i++ ){
		len_j = original[i].size();
		std::vector<T> temp;
		for( size_t j = 0 ; j < len_j ; j++ ){
			temp.push_back( original[i][j] );
		}
		rtnVecVec.push_back( temp );
	}
	return rtnVecVec;
}

template<typename T>
T* elem_i_from_list( std::list<T>& searchList , size_t index ){
	// Access a list like it were a vector
	typename std::list<T>::iterator it; // URL , resolve dependent templated typenames: https://stackoverflow.com/a/11275548
	size_t i     = 0                 ,
		   len   = searchList.size() ;
	for( it = searchList.begin() ; it != searchList.end() ; ++it ){
		if( index == i ){  return &(*it);  }
		i++;
	}
	return nullptr;
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

// ~~ Array Initialization ~~ 
int foo[3][2] = { { 1 , 2 } , { 3 , 4 } , { 5 , 6 } }; // Nested array assignment test

  __ End Parts __________________________________________________________________________________________________________________________ */
  
  
/* == Spare Parts ==========================================================================================================================

template<typename T>
size_t vec_wrap_index( std::vector<T> vec , llin rawIndex ){
	size_t vecLen = vec.size();
	if( rawIndex >= 0 ){
		return rawIndex % vecLen;
	} else {
		
	}
}

 __ End Parts ___________________________________________________________________________________________________________________________ */
