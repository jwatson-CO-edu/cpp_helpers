/***********  
Cpp_Helpers.cpp
James Watson , 2017 March
Functions of general use for C++ programming

Template Version: 2017-04-13
***********/

#include "Cpp_Helpers.h"

// == Struct Helpers ==

IndexSearchResult default_false_result(){  return IndexSearchResult{ false , 0 };  } // Return a failed search result , index 0

// __ End Helpers __

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

int    randrange( int end ){    return (int)( rand() % end );    }
size_t randrange( size_t end ){ return (size_t)( rand() % end ); }

// NOTE: 'randrange' functions do not actually check if range bounds are in the proper order

int randrange( int bgn , int end ){ return bgn + (int)( rand() % ( end - bgn ) ); }

double randrange( double lo , double hi ){ return lo + (double) rand() * ( hi - lo ); }

usll tri_num( usll n ){ return n * ( n + 1 ) / 2; } // --- Return the nth triangular number
size_t tri_num( size_t n ){ return n * ( n + 1 ) / 2; } // Return the nth triangular number

double round_zero( double num ){ if( abs( num ) < EPSILON ){ return 0.0d; }else{ return num; } }

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

string timestamp(){
	// Return a timestamp suitable for filenames
	// URL: http://www.cplusplus.com/forum/beginner/32329/#msg174653
	time_t t = time(NULL);
	tm* timePtr = localtime(&t);
	
	return to_string( 1900 + timePtr->tm_year ) + "-" + prepad( to_string( timePtr->tm_mon ) , 2 , '0' ) + "-"
		 + prepad( to_string( timePtr->tm_mday ) , 2 , '0' ) + "_" + prepad( to_string( timePtr->tm_hour ) , 2 , '0' )
		 + "-" + prepad( to_string( timePtr->tm_min ) , 2 , '0' ) + "-" + prepad( to_string( timePtr->tm_sec ) , 2 , '0' );
}

// __ End File __


// == String Tools ==

void remove_all( string& rawStr , char keyChar ){ // Destructively remove all instances of 'keyChar' from 'rawStr'
	string::size_type i = 0; // http://stackoverflow.com/a/1488798/7186022
	while ( i < rawStr.length() ){ // While the end of the string has not been reached
		i = rawStr.find( keyChar , i ); // Assign the index to the next instance of 'keyChar'
		if ( i == string::npos ){ break; } // If we reached the end of the string , exit
		rawStr.erase( i ); // If an instance of the char has been found , erase it ( 'erase' resizes , 'remove' does not )
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

string prepad(  string original , size_t totLen , char padChar ){
	size_t origLen = original.size();
	if( origLen < totLen ){  return string( totLen - origLen , padChar ) + original;  }
	else{  return original;  }
}

string postpad( string original , size_t totLen , char padChar ){
	size_t origLen = original.size();
	if( origLen < totLen ){  return original + string( totLen - origLen , padChar );  }
	else{  return original;  }
}

// __ End String __


// == Container Tools ==

std::vector<bool> bool_false_vector( size_t length ){
	std::vector<bool> rtnVec;
	for( size_t i = 0 ; i < length ; i++ ){ rtnVec.push_back( false ); }
	return rtnVec;
}

std::vector<std::vector<bool>> bool_false_vec_vec( size_t length ){
	std::vector<std::vector<bool>> rtnVecVec;
	for( size_t i = 0 ; i < length ; i++ ){ 
		std::vector<bool> temp;
		for( size_t j = 0 ; j < length ; j++ ){  temp.push_back( false );  }
		rtnVecVec.push_back( temp ); 
	}
	return rtnVecVec;
}

size_t random_false_elem_index( std::vector<bool> vec ){
	// Return a random index of an element that has value 'false'
	std::vector<size_t> availableIndices;
	size_t vecLen = vec.size();
	// Build a vector of available 'false' indices so that we are guaranteed to make the right choice
	for( size_t i = 0 ; i < vecLen ; i++ ){ if( vec[i] == false ){ availableIndices.push_back( i ); } }
	return rand_choice( availableIndices );
}

bool all_elem_true( std::vector<bool>& bulVec ){ 
	// Return true if all elements true , otherwise return false
	size_t len = bulVec.size();
	for( size_t i = 0 ; i < len ; i++ ){  if( !bulVec[i] ){  return false;  }  }
	return true;
}

llin indexw( llin len , llin i ){
    //  Return the 'i'th index of 'iterable', wrapping to index 0 at all integer multiples of 'len' , Wraps forward and backwards , Python Style
	llin revDex = 0;
    if( i >= 0 ){
        return i % ( len );
    } else {
        revDex = abs( i ) % ( len );
        if( revDex == 0 ){ return 0; }
        return len - revDex;
	}
}

std::ostream& operator<<( std::ostream& os , const std::set<int>& elemSet ) {
	size_t count = 0              ,
		   len   = elemSet.size() ;
	os << "(set){ ";
	std::set<int>::iterator it;
	for( it = elemSet.begin() ; it != elemSet.end() ; ++it ){
		os << *it;
		if( count + 1 < len ){ os << ", "; }
		count++;
	}
	os << " }";
	return os; // You must return a reference to the stream!
}

std::vector<size_t> vec_index_zeros( size_t len ){
	std::vector<size_t> rntVec;
	for( size_t i = 0 ; i < len ; i++ ){  rntVec.push_back( 0 );  }
	return rntVec;
}

// __ End Container __

// === Functors ===

// == class Incrementer ==

Incrementer::Incrementer( llin start ){ count = start; } // Create an incrementer with a starting number

llin Incrementer::operator()(){ llin temp = count;  count++;  return temp;  } // Increment the counter and return it

// __ End Incrementer __

// ___ End Functors ___
