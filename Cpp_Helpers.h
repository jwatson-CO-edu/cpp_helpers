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
#include <algorithm> //- Searching structures                      /
#include <queue> // ---- Priority Queue                           /
#include <utility> // -- Pair , pair get ________________________/

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
using std::isinf; // -------------- Infinity Test
using std::abs; // ---------------- Absolute value
using std::printf; // ------------- Our fave printing function from C

// ~ Type Aliases ~ // Use this for long type names and names that are unlikley to be shadowed
using usll = unsigned long long; // big ints ( unsigned ) // Requires C++11
using llin = long long int; // ---- big ints              // Requires C++11

// ~ Constants ~
#define EPSILON 1e-8d // ---------- Margin too small to care about
double const INFTY_D = std::numeric_limits<double>::infinity();

// __ End Shortcuts __


// === Structs ===

struct IndexSearchResult{ // A container to hold a search result for an index that cannot have a negative value
	bool   result; // Is the result a valid one?
	size_t index; //- If so, which is the index we like best?
};

struct IndexDbblResult{ // A container to hold a search result for an index that cannot have a negative value
	size_t index; // - Which is the index we like best?
	double measure; // How much do we like it?
};

struct IDDbblResult{ // A container to hold a search result for an index that cannot have a negative value
	llin   ID; // ---- Which is the index we like best?
	double measure; // How much do we like it?
};

struct IndexMultiResult{ // A container to hold a search result for an index that cannot have a negative value
	bool /* -------- */ result; // Is the result a valid one?
	std::vector<size_t> indices; //- If so, which is the index we like best?
};

struct SuccessCode{
	bool   success; // Succeed or Fail?
	size_t code; // -- Status code
	string desc; // -- Description of { disposition , failure reason , results , etc }
};

// ___ End Struct ___


// == Struct Helpers ==

IndexSearchResult default_false_result();

// __ End Helpers __


// == Debug Tools ==

void assert_report( bool assertion , string report ); // Reporting wrapper for 'assert'

void sep_dbg(); // Print a separator for debug information

void sep( string title = "" , size_t width = 6 , char dingbat = '=' ); // Print a separating title card for debug 

void newline(); //  print a new line

// __ End Debug __


// == Math Tools ==

float rand_float(); // Note that this is not exactly uniformly distributed

double rand_dbbl();

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

string prepad(  string original , size_t totLen , char padChar = ' ' );
string postpad( string original , size_t totLen , char padChar = ' ' );

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
size_t min_index_in_vec( std::vector<T> searchVec ){
	T      least   = searchVec[0]; 
	size_t minDex  = 0                ,
		   i       = 0                , 
	       numElem = searchVec.size() ;
	for( i = 0 ; i < numElem ; i++ ){ 
		if( searchVec[i] < least ){
			least  = searchVec[i];
			minDex = i;
		}
	} 
	return minDex;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
size_t max_index_in_vec( std::vector<T> searchVec ){
	T      most    = searchVec[0]; 
	size_t maxDex  = 0                ,
		   i       = 0                , 
	       numElem = searchVec.size() ;
	for( i = 0 ; i < numElem ; i++ ){ 
		if( searchVec[i] > most ){
			most   = searchVec[i];
			maxDex = i;
		}
	} 
	return maxDex;
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
bool is_arg_in_list( T arg , std::list<T>& lst ){
	// Return true if 'arg' is in 'lst' , false otherwise
	return find( lst.begin() , lst.end() , arg ) != lst.end();
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool is_arg_in_set( T arg , std::set<T>& st ){
	// Return true if 'arg' is in 'st' , false otherwise
	// URL , C++ cannot recognize a templated typename
	typename std::set<T>::iterator it = st.find( arg );
	return it != st.end();
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool is_arg_in_vector( T arg , std::vector<T>& vec ){
	// Return true if 'arg' is in 'st' , false otherwise
	// URL , resolve dependent templated typenames: https://stackoverflow.com/a/11275548
	typename std::vector<T>::iterator it = find( vec.begin() , vec.end() , arg );
	return it != vec.end();
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
IndexSearchResult search_vec_for_arg( std::vector<T>& vec , T arg ){
	// Search the vector for the specified 'arg' and return the result
	// NOTE: This function assumes that the '==' comparison operator can be used on the vector items
	IndexSearchResult result = default_false_result();
	size_t len = vec.size();
	for( size_t i = 0 ; i < len ; i++ ){
		if( arg == vec[i] ){  
			result.result = true;  
			result.index  = i;  
			return result; // Found a match, shortcut return with the result
		}
	}
	return result; // No match found , Return failed result
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
IndexSearchResult search_vec_for_arg( const std::vector<T>& vec , T arg ){
	// Search the vector for the specified 'arg' and return the result
	// NOTE: This function assumes that the '==' comparison operator can be used on the vector items
	IndexSearchResult result = default_false_result();
	size_t len = vec.size();
	for( size_t i = 0 ; i < len ; i++ ){
		if( arg == vec[i] ){  
			result.result = true;  
			result.index  = i;  
			return result; // Found a match, shortcut return with the result
		}
	}
	return result; // No match found , Return failed result
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool vec2_contains_vec1( std::vector<T>& vec1 , std::vector<T>& vec2 ){
	// Return true if every element of 'vec1' can be found in 'vec2' , Otherwise return false
	size_t len1 = vec1.size();
	IndexSearchResult result;
	for( size_t i = 0 ; i < len1 ; i++ ){ // For each of the elements of 'vec1'
		result = search_vec_for_arg( vec2 , vec1[i] );
		if( !result.result ){ return false; } // Mismatch , Shortcut to false
	}
	return true; // Of we got here, then none of the searches failed
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool vec2_contains_vec1( const std::vector<T>& vec1 , const std::vector<T>& vec2 ){
	// Return true if every element of 'vec1' can be found in 'vec2' , Otherwise return false
	size_t len1 = vec1.size();
	IndexSearchResult result;
	for( size_t i = 0 ; i < len1 ; i++ ){ // For each of the elements of 'vec1'
		result = search_vec_for_arg( vec2 , vec1[i] );
		if( !result.result ){ return false; } // Mismatch , Shortcut to false
	}
	return true; // Of we got here, then none of the searches failed
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool vec_same_contents( std::vector<T>& vec1 , std::vector<T>& vec2 ){
	// Return true if and only if all of the elements of 'vec1' are in 'vec2' and vice-versa
	return vec2_contains_vec1( vec1 , vec2 ) && vec2_contains_vec1( vec2 , vec1 );
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool vec_same_contents( const std::vector<T>& vec1 , const std::vector<T>& vec2 ){
	// Return true if and only if all of the elements of 'vec1' are in 'vec2' and vice-versa
	return vec2_contains_vec1( vec1 , vec2 ) && vec2_contains_vec1( vec2 , vec1 );
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
std::vector<T> vec_copy_without_elem( std::vector<T>& original , T holdout ){
	// NOTE: This function assumes that the equality operator is defined for type T
	size_t len = original.size();
	std::vector<T> rtnVec;
	for( size_t i = 0 ; i < len ; i++ ){  if( original[i] != holdout ){  rtnVec.push_back( original[i] );  }  }
	return rtnVec;
}

template<typename T>
std::vector<T> vec_copy_without_elem( std::vector<T>& original , std::vector<T>& holdout ){
	// NOTE: This function assumes that the equality operator is defined for type T
	size_t len = original.size();
	std::vector<T> rtnVec;
	for( size_t i = 0 ; i < len ; i++ ){  if( !is_arg_in_vector( original[i] , holdout ) ){  rtnVec.push_back( original[i] );  }  }
	return rtnVec;
}

template<typename T>
std::vector<T> vec_copy_without_elem( const std::vector<T>& original , T holdout ){
	// NOTE: This function assumes that the equality operator is defined for type T
	size_t len = original.size();
	std::vector<T> rtnVec;
	for( size_t i = 0 ; i < len ; i++ ){  if( original[i] != holdout ){  rtnVec.push_back( original[i] );  }  }
	return rtnVec;
}

template<typename T>
std::vector< size_t > vec_vec_len( std::vector<std::vector<T>>& vecVec ){
	std::vector<size_t> rtnVec;
	size_t len = vecVec.size();
	for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( vecVec[i].size() );  }
	return rtnVec;
}

template<typename T>
size_t vec_vec_longest_rep( std::vector<std::vector<T>>& vecVec ){
	// Return the length of the longest string representation of an element of a 2D std::vector
	size_t longest = 0;
	size_t len_i , len_j;
	string tempRep;
	len_i = vecVec.size();
	for( size_t i = 0 ; i < len_i ; i++ ){
		len_j = vecVec[i].size();
		for( size_t j = 0 ; j < len_j ; j++ ){
			tempRep = to_string( vecVec[i][j] );
			longest = max( tempRep.size() , longest );
		}
	}
	return longest;
}

template<typename T>
size_t vec_vec_longest_rep( const std::vector<std::vector<T>>& vecVec ){
	// Return the length of the longest string representation of an element of a 2D std::vector
	size_t longest = 0;
	size_t len_i , len_j;
	string tempRep;
	len_i = vecVec.size();
	for( size_t i = 0 ; i < len_i ; i++ ){
		len_j = vecVec[i].size();
		for( size_t j = 0 ; j < len_j ; j++ ){
			tempRep = to_string( vecVec[i][j] );
			longest = max( tempRep.size() , longest );
		}
	}
	return longest;
}

template<typename T>
void print_vec_vec( std::vector<std::vector<T>>& vecVec , size_t padLen = 1 ){
	// Return the length of the longest string representation of an element of a 2D std::vector
	size_t longest = vec_vec_longest_rep( vecVec );
	size_t len_i , len_j;
	string tempRep;
	len_i = vecVec.size();
	for( size_t i = 0 ; i < len_i ; i++ ){
		len_j = vecVec[i].size();
		for( size_t j = 0 ; j < len_j ; j++ ){
			tempRep = prepad( to_string( vecVec[i][j] ) , longest );
			cout << tempRep << string( padLen , ' ' );
		}
		cout << endl;
	}
}

template<typename T>
void print_vec_vec( const std::vector<std::vector<T>>& vecVec , size_t padLen = 1 ){
	// Return the length of the longest string representation of an element of a 2D std::vector
	size_t longest = vec_vec_longest_rep( vecVec );
	size_t len_i , len_j;
	string tempRep;
	len_i = vecVec.size();
	for( size_t i = 0 ; i < len_i ; i++ ){
		len_j = vecVec[i].size();
		for( size_t j = 0 ; j < len_j ; j++ ){
			tempRep = prepad( to_string( vecVec[i][j] ) , longest );
			cout << tempRep << string( padLen , ' ' );
		}
		cout << endl;
	}
}

template<typename T>
size_t vec_vec_any_empty( std::vector<std::vector<T>>& vecVec ){
	// Return true if any of the sub-vectors are empty, otherwise return false
	size_t len = vecVec.size();
	for( size_t i = 0 ; i < len ; i++ ){  if( vecVec[i].size() == 0 ){  return true;  }  }
	return false;
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
void vec_vec_bottom_clear( std::vector<std::vector<T>>& vecVec ){
	// Clear the bottom-level vectors
	size_t len = vecVec.size();
	for( size_t i = 0 ; i < len ; i++ ){  vecVec[i].clear();  }
}

template<typename T>
void vec_vec_top_populate( std::vector<std::vector<T>>& vecVec , size_t N ){
	// Populate the top level with empty vectors
	vecVec.clear();
	for( size_t i = 0 ; i < N ; i++ ){
		std::vector<T> temp;
		vecVec.push_back( temp );
	}
}

template<typename T>
T* elem_i_from_list( std::list<T>& searchList , size_t index ){
	// Access a list like it were a vector
	// NOTE: Retuning a pointer so that the same type can be used as both a return value and an error code
	typename std::list<T>::iterator it; // URL , resolve dependent templated typenames: https://stackoverflow.com/a/11275548
	size_t i     = 0                 ,
		   len   = searchList.size() ;
	for( it = searchList.begin() ; it != searchList.end() ; ++it ){
		if( index == i ){  return &(*it);  }
		i++;
	}
	return nullptr; // ERROR , No such index!
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
IndexMultiResult first_in_common_btn_lists_index( std::list<T>& lst1 , std::list<T>& lst2 ){
	size_t i = 0 ,
		   j = 0 ;
	typename std::list<T>::iterator it_i , // URL , resolve dependent templated typenames: https://stackoverflow.com/a/11275548
									it_j ;  
	IndexMultiResult result;
	result.result = false;
	for( it_i = lst1.begin() ; it_i != lst1.end() ; ++it_i ){
		for( it_j = lst2.begin() ; it_j != lst2.end() ; ++it_j ){
			result.result = *it_i == *it_j;
			if( result.result ){  break;  }
			j++;
		}
		if( result.result ){  break;  }
		i++;
	}
	result.indices.push_back( i );  result.indices.push_back( j );  // These indices point to the commonality, or the last indices
	return result;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
IndexMultiResult first_in_common_btn_vec_index( std::vector<T>& lst1 , std::vector<T>& lst2 ){
	size_t i     = 0           ,
		   j     = 0           ,
		   len_i = lst1.size() , 
		   len_j = lst2.size() ;
	
	IndexMultiResult result;
	for( i = 0 ; i < len_i ; i++ ){
		for( j = 0 ; j < len_j ; j++ ){
			result.result = lst1[i] == lst2[j];
			if( result.result ){  break;  }
		}
		if( result.result ){  break;  }
	}
	result.indices.push_back( i );  result.indices.push_back( j );  // These indices point to the commonality, or the last indices
	return result;
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

// ~ Function Pointer ~
void (*foo)(int) // Declare function pointer 'foo' to a function that takes one int and returns void

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
