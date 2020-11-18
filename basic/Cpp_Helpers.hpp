#pragma once

/***********  
Cpp_Helpers.h
James Watson , 2017 March
Shortcuts and Aliases , and Functions of general use for C++ programming

Template Version: 2017-05-26
***********/

#ifndef CPPHELPERS_H
#define CPPHELPERS_H

#if defined(unix) || defined(__unix__) || defined(__unix)
    #define IS_LINUX
#endif

#include <string> // --- string manipulation
#include <cstdio> // --- printf , getchar
#include <cmath> // ---- abs, min/max, trig, hyperbolic, power, exp, error, rounding
#include <limits> // --- Infinity
#include <stdlib.h> // - srand , rand , atof , strtof
#include <time.h> // --- time , for getting sys time and seeding random numbers
#include <ctime> // ---- time , for date formatting
#include <time.h> /* for clock_gettime */
#include <chrono> // more different time 
#include <stdint.h> /* for uint64 definition */
#include <limits> // --- number limits of data types, limit on 'cin.ignore'
#include <ctype.h> // -- type tests
#include <cassert> // -- input/condition verification
//#define NDEBUG // ---- uncomment to disable assert()
#include <stdexcept> //- std errors

#include <vector> // --------- standard vector datatype , the friendly array } Data Structures
#include <list> // ----------- standard list datatype                       /
#include <map> // ------------ dictionaries                                /
#include <set> // ------------ sets                                       /
#include <algorithm> // ------ Searching structures , sort               /
#include <queue> // ---------- Priority Queue                           /
#include <utility> // -------- Pair , pair get , swap                  /
#include <initializer_list> // Pass array literals to funcs __________/

#include <iostream> // - standard input and output , istream } Input / Output
#include <fstream> // -- File I/O                           /
#include <sstream> // -- Text streams                      /
#include <sys/stat.h> // File status _____________________/
#include <filesystem>

namespace fs = std::filesystem;

typedef  std::chrono::steady_clock::time_point  StdTime;
typedef  std::chrono::steady_clock              StdClock;

// == Shortcuts and Aliases ==

// ~ Standard Shortcuts ~ // This is only for names that are unlikely to be shadowed
using std::cout; // ------- output to terminal
using std::cerr; // ------- Unbuffered terminal output , std::cerr automatically flushes all output as soon as it is written?
using std::clog; // ------- Unbuffered terminal output
using std::endl; // ------- newline
using std::cin; // -------- input from terminal
using std::ifstream; // --- File Input streams
using std::ofstream; // --- File Output streams
using std::ostream; // ---- Output streams
using std::stringstream; // String streams
using std::string; // ----- strings!           // Requires C++11
using std::to_string; // -- string conversion  // Requires C++11
using std::min; // -------- 'min' function
using std::max; // -------- 'max' function
using std::abs; // -------- Absolute value
using std::pow; // -------- Exponents
using std::ceil; // ------- Ceiling
using std::round; // ------ To nearest integer
using std::isnan; // ------ NaN Test
using std::isinf; // ------ Infinity Test
using std::printf; // ----- Our fave printing function from C
using std::swap; // ------- Swap 2 values , per CS 100
using std::sort; // ------- Get it sorted
using std::vector;
using std::chrono::duration_cast; 
using std::chrono::microseconds;

// ~ Type Aliases ~ // Use this for long type names and names that are unlikley to be shadowed
using usll = unsigned long long; // big ints ( unsigned ) // Requires C++11
using llin = long long int; // ---- big ints              // Requires C++11
using uint = unsigned int; // ----- unsigned ints

// ~ Constants ~
#define EPSLNDB 1e-8 // ----------- Margin too small to care about , double
#define EPSLNFL 1e-4f // ---------- Margin too small to care about , float
#define BILLION 1000000000L // ---- ONE BILLION
double const INFTY_D   = std::numeric_limits<double>::infinity();
double const BILLION_D = 1e9;

// __ End Shortcuts __

// == Debug Tools ==

double time_elapsed( StdTime& clok ); // Get the elapsed time

void print_args( int argc, char *argv[] );

void assert_report( bool assertion , string report ); // Reporting wrapper for 'assert'

string yesno( bool condition );

void waitkey();
void waitkey( bool condition );
void waitkey( bool condition , string message );

void sep_dbg(); // Print a separator for debug information

void sep( string title = "" , size_t width = 6 , char dingbat = '~' ); // Print a separating title card for debug 

// __ End Debug __


// == Logic / Control Tools ==

void toggle( bool& bit );

// __ End Logic / Ctrl __


// == Math Tools ==

void random_seed_RUN_ONCE_MAIN(); // init random

double rand_01(); // Return a random number on [0,1)

void fill_array_rand( double arr[] , size_t len ); // Populate an array with random doubles on [0,1)

bool dice_roll( double prob );

int randrange( int end );

size_t randrange( size_t end );

int randrange( int bgn , int end );

template<typename FLT>
FLT randrange( FLT lo , FLT hi ){ return (FLT) min( lo , hi ) + (FLT) rand_01() * abs( hi - lo ); }

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool eq( T op1 , T op2 ){ return ( (double) abs( op1 - op2 ) ) < EPSLNDB; }

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool eqf( T op1 , T op2 ){ return ( (float) abs( op1 - op2 ) ) < EPSLNFL; }

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool eq( T op1 , T op2 , T eps ){ return ( abs( op1 - op2 ) ) < eps; }

template < typename T , typename U >
bool eq( T op1 , U op2 ){ return ( abs( (double)op1 - (double)op2 ) ) < EPSLNDB; }

usll tri_num( usll n );
size_t tri_num( size_t n );

double round_zero( double num );

template <typename T> 
int sign( T val ) { return ( T(0) < val ) - ( val < T(0) ); } // Return the sign if the number: -1 for val<0 , 0 for val==0 , 1 for val>0

template <typename T> 
bool is_even( T number ){  return ( number % (T)2 ) == 0;  }

template <typename T> 
bool is_odd( T number ){  return ( number % (T)2 ) != 0;  }

template <typename T> 
T clamp_val( T val , T minm , T maxm ){
    // NOTE: std::clamp is available since C++17 , https://en.cppreference.com/w/cpp/algorithm/clamp
    if( val < minm ) return minm;
    if( val > maxm ) return maxm;
    return val;
}

// __ End Math __


// == File Tools ==

bool check_exist( string path , string check );

vector<string> list_path( string path );

vector<string> readlines( string path ); // Return all the lines of text file as a string vector

void printlines( const vector<string>& lines ); // Print all the lines read from a file

string timestamp();

string get_file_string( string path ); // Get the contents of the file at `path` as a string

// __ End File __


// == String Tools ==

// print the binary representation of a uint
void print_binary_int( unsigned int arg ); 

// Print the hexadecimal representation of a uint
void print_hex_int( unsigned int a );

// print the binary representation of a uchar
void print_binary_char( unsigned char arg );

void remove_all( string& rawStr , char keyChar ); // Destructively remove all instances of 'keyChar' from 'rawStr'

// Destructively remove all newlines from 'rawStr'
void strip_newlines( string& rawStr );

string strip_after_dot( string fName ); // Return a copy of 'fName' with the first period and all following characters removed

string prepad(  string original , size_t totLen , char padChar = ' ' );
string postpad( string original , size_t totLen , char padChar = ' ' );

bool str_has_sub( string bigStr , string subStr ); // Return true if 'bigStr' contains 'subStr' , Otherwise return false

bool isnewline( char queryChar ); // Return true if the char is a newline , Otherwise return false

vector<double> tokenize_to_dbbl_w_separator( string rawStr , char separator ); // Return a vector of doubles between 'separator'
vector<float> tokenize_to_float_w_separator( string rawStr , char separator ); // Return a vector of floats between 'separator'

vector<string> split( string s , char sep );



bool has_substr( string& superStr , string& subStr );

// __ End String __


// == Timing ==

class StopWatch{
    
public:
    
    StopWatch();
    StopWatch( double intervalSec );
    bool has_interval_elapsed();
    double seconds_elapsed();
    void mark();
        
protected:

    double interval;
    struct timespec tNow;
    struct timespec tMrk;
    
};

// __ End Timing __


// == Container Tools ==

vector<size_t> vec_index_zeros( size_t len );

vector<double> vec_dbbl_zeros( size_t len );

vector<vector<double>> vec_vec_dbbl_zeros( size_t len ); // Return a square vector of zeros

vector<bool> bool_false_vector( size_t length );

vector<vector<bool>> bool_false_vec_vec( size_t length );

size_t random_false_elem_index( vector<bool> vec );

bool   all_elem_true( const vector<bool>& bulVec ); // - Return true if all elements true , otherwise return false
bool   any_elem_true( const vector<bool>& bulVec ); // - Return true if any elements true , otherwise return false
size_t count_elem_true( const vector<bool>& bulVec ); // Return the number of elements that are true

//  Return the 'i'th index of 'iterable', wrapping to index 0 at all integer multiples of 'len' , Wraps forward and backwards , Python Style
llin indexw( llin len , llin i );

size_t wrap_index_for_len( size_t index , size_t len );

std::ostream& operator<<( std::ostream& os , const std::set<int>& elemSet );

template<typename T>
vector<T> vec_copy( const vector<T>& original ){
    size_t len = original.size();
    vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( original[i] );  }
    return rtnVec;
}



vector<double> err_vec( size_t len );

bool is_err( const vector<double>& vec ); // Return true if all of the elements are NaN, Otherwise return false

vector<vector<size_t>> enumerate_in_base( size_t digits , size_t base );

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


// === Memory Utils ===

string pointer_info_str( void* generalPointer );

template<typename T>
void delif( T*& ptr ){  
    // Delete the pointer if it contains data and set it to NULL
    if( ptr ){  
        delete ptr;  
        ptr = nullptr;
    }  
} 

template<typename T>
void clearif( vector<T*>& vec ){
    // Delete a vector of pointers, Then clear the vector
    size_t len = vec.size();
    for( size_t i = 0 ; i < len ; i++ ){  delif( vec[i] );  }
    vec.clear();
}

template<typename T>
void clearif( std::list<T*>& lst ){
    // Delete a list of pointers, Then clear the list
    typename std::list<T*>::iterator it;
    for( it = lst.begin() ; it != lst.end() ; ++it ){  delif( *it );  }
    lst.clear();
}

// ___ End Memory ___

#endif

/* == Useful Parts =========================================================================================================================

// ~~ Debug by Print ~~
bool SHOWDEBUG = true; // if( SHOWDEBUG ){  cout << "" << endl;  }

// ~~ cout << operator ~~
std::ostream& operator<<(std::ostream& os, const Gene& vec) {
    vector<float> codons = vec.copy_codons();
    os << "[ ";
    for (size_t i = 0; i < codons.size(); i++) {
        os << codons[i];
        if (i + 1 < codons.size()) { os << ", "; }
    }
    os << " ]";
    return os; // You must return a reference to the stream!
}


// ~~ Function Object (functor) ~~
struct myclass {
  bool operator() (int i,int j) { return (i<j);}
} myobject;


// ~~ Array Initialization ~~ 
int foo[3][2] = { { 1 , 2 } , { 3 , 4 } , { 5 , 6 } }; // Nested array assignment test


// ~ Function Pointer ~
void (*foo)(int) // Declare function pointer 'foo' to a function that takes one int and returns void


// ~ Lambda (C++11) ~
https://www.cprogramming.com/c++11/c++11-lambda-closures.html

// LAMBDA: [&] Capture all named variables by reference
// Type defaults to void if there is no return statement
auto checkAndUpdateBest = [&]( Eigen::Vector3d direction ){
    result = query_NDBG( NDBG , movedParts , referenceParts , direction );
    if( result.result ){
        currAng = angle_between( direction , avrgDir );
        if( currAng < bestAng ){
            bestDir = direction;
            bestAng = currAng;
        }
    }
}; // NOTE: Lambda expression must end with a semicolon

bool SHOWDEBUG = true  , // if( SHOWDEBUG ){  cout << "" << endl;  }
     BREAKPNTS = false ; // if( BREAKPNTS ){  waitkey( CONDITION , "MSG" );  }

  __ End Parts __________________________________________________________________________________________________________________________ */
  
  
/* == Spare Parts ==========================================================================================================================

template<typename T>
size_t vec_wrap_index( vector<T> vec , llin rawIndex ){
    size_t vecLen = vec.size();
    if( rawIndex >= 0 ){
        return rawIndex % vecLen;
    } else {
        
    }
}

 __ End Parts ___________________________________________________________________________________________________________________________ */
