/***********  
Cpp_Helpers.cpp
James Watson , 2017 March
Functions of general use for C++ programming

Template Version: 2017-04-13
***********/

#include "Cpp_Helpers.hpp"

// == Debug Tools ==

void assert_report( bool assertion , string report ) { // Reporting wrapper for 'assert'
    if ( !assertion ) { cerr << endl << report << endl; } // if an assert will trigger, print BEFORE
    assert( assertion ); // crash if necessary
}

void sep_dbg(){ cout << "==================== Debug ====================" << endl; } // Print a separator for debug information

void sep( string title , size_t width , char dingbat ){
    // Print a separating title card for debug 
    cout << string( width , dingbat ) << " " << title << " " << string( width , dingbat ) << endl;
}

string yesno( bool condition ){  return ( condition ? "YES" : "NO" );  }

void waitkey(){  
    cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    cout << "BREAKPOINT: Press [Enter] to continue...\n";  
    getchar();  
}

void waitkey( bool condition ){
    if( condition ){
        cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
        cout << "BREAKPOINT: Press [Enter] to continue...\n";  
        getchar();
    }
}

void waitkey( bool condition , string message ){
    if( condition ){
        cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
        cout << "BREAKPOINT: " << message << " , Press [Enter] to continue...\n";  
        getchar();
    }
}

// __ End Debug __


// == Logic / Control Tools ==

void toggle( bool& bit ){  bit = !bit;  }

// __ End Logic / Ctrl __


// == Math Tools ==

bool dice_roll( double prob ){  return rand_01() < prob;  }

int    randrange( int end ){    return (int)( rand() % end );    }
size_t randrange( size_t end ){ return (size_t)( rand() % end ); }

// NOTE: 'randrange' functions do not actually check if range bounds are in the proper order

int randrange( int bgn , int end ){ return bgn + (int)( rand() % ( end - bgn ) ); }

usll tri_num( usll n ){ return n * ( n + 1 ) / 2; } // --- Return the nth triangular number
size_t tri_num( size_t n ){ return n * ( n + 1 ) / 2; } // Return the nth triangular number

double round_zero( double num ){ if( abs( num ) < EPSLNDB ){ return 0.0; }else{ return num; } }

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

void printlines( const std::vector<string>& lines ){
    // Print all the lines read from a file
    size_t len = lines.size();
    for( size_t i = 0 ; i < len ; i++ ){  cerr << lines[i] << endl;  } // unbuffered dump
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
    size_t index = 0; string rtnStr = "";
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

// Return true if 'bigStr' contains 'subStr' , Otherwise return false
bool str_has_sub( string bigStr , string subStr ){  return bigStr.find( subStr ) != string::npos;  }

// Return true if the char is a newline , Otherwise return false
bool isnewline( char queryChar ){  return ( queryChar == '\r' ) || ( queryChar == '\n' );  }

std::vector<double> tokenize_to_dbbl_w_separator( string rawStr , char separator ){ 
    // Return a vector of doubles between 'separator'
    size_t len = rawStr.length();
    //~ tokens = [] # 
    std::vector<double> tokens; // list of tokens to return
    //~ currToken = '' # 
    string currToken = ""; // the current token, built a character at a time
    char currChr;
    //~ for char in rawStr: # for each character of the input string
    for( size_t i = 0 ; i < len ; i++ ){
        currChr = rawStr[i];
        //~ if not char.isspace(): # 
        if( !isspace( currChr ) && !isnewline( currChr ) ){ // if the current char is not whitespace, process
            //~ if not char == separator: # if the character is not a separator, then
            if( !( currChr == separator ) ){
                //~ currToken += char # accumulate the char onto the current token
                currToken += currChr;
            }else{ // else the character is a separator, process the previous token
            //~ else: # 
                //~ tokens.append( evalFunc( currToken ) ) # 
                tokens.push_back( atof( currToken.c_str() ) ); // transform token and append to the token list
                //~ currToken = '' # 
                currToken = ""; // reset the current token
            }
        } //~ # else is whitespace, ignore
    }
    //~ if currToken: # If there is data in 'currToken', process it
    if( currToken.length() > 0 )
        //~ tokens.append( evalFunc( currToken ) ) # transform token and append to the token list
        tokens.push_back( atof( currToken.c_str() ) ); // transform token and append to the token list
    //~ return tokens
    return tokens;
}

vector<float> tokenize_to_float_w_separator( string rawStr , char separator ){ 
    // Return a vector of doubles between 'separator'
    size_t len = rawStr.length();
    //~ tokens = [] # 
    std::vector<float> tokens; // list of tokens to return
    //~ currToken = '' # 
    string currToken = ""; // the current token, built a character at a time
    char currChr;
    //~ for char in rawStr: # for each character of the input string
    for( size_t i = 0 ; i < len ; i++ ){
        currChr = rawStr[i];
        //~ if not char.isspace(): # 
        if( !isspace( currChr ) && !isnewline( currChr ) ){ // if the current char is not whitespace, process
            //~ if not char == separator: # if the character is not a separator, then
            if( !( currChr == separator ) ){
                //~ currToken += char # accumulate the char onto the current token
                currToken += currChr;
            }else{ // else the character is a separator, process the previous token
            //~ else: # 
                //~ tokens.append( evalFunc( currToken ) ) # 
                tokens.push_back( strtof( currToken.c_str() , NULL ) ); // transform token and append to the token list
                //~ currToken = '' # 
                currToken = ""; // reset the current token
            }
        } //~ # else is whitespace, ignore
    }
    //~ if currToken: # If there is data in 'currToken', process it
    if( currToken.length() > 0 )
        //~ tokens.append( evalFunc( currToken ) ) # transform token and append to the token list
        tokens.push_back( atof( currToken.c_str() ) ); // transform token and append to the token list
    //~ return tokens
    return tokens;
}

// __ End String __


// == Timing ==

// = class StopWatch =

void StopWatch::mark(){
    // Mark the present time for comparison later
    clock_gettime( CLOCK_MONOTONIC , &tMrk );
}

StopWatch::StopWatch(){  mark();  } // New StopWatch with the time of it's instantiation marked

StopWatch::StopWatch( double intervalSec ){
    // New StopWatch with a monitored interval specified
    mark();
    interval = intervalSec;
}

double StopWatch::seconds_elapsed(){
    // Get the number of seconds that have passed since the last mark
    // 1. Get current time
    clock_gettime( CLOCK_MONOTONIC , &tNow );
    // 2. Get elapsed time
    return (double) ( tNow.tv_sec - tMrk.tv_sec ) + (double) ( tNow.tv_nsec - tMrk.tv_nsec ) / BILLION;
}

bool StopWatch::has_interval_elapsed(){
    // If the interval has passed since the last mark, return true and reset mark, Otherwise return false
    bool answer = false;
    // 1. Get elapsed time
    double diff = seconds_elapsed();
    // 2. If the interval has elapsed, set mark
    if( diff >= interval ){
        answer = true;
        mark();
    }
    return answer;
}



// _ End StopWatch _

// __ End Timing __


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

bool all_elem_true( const std::vector<bool>& bulVec ){ 
    // Return true if all elements true , otherwise return false
    size_t len = bulVec.size();
    for( size_t i = 0 ; i < len ; i++ ){  if( !bulVec[i] ){  return false;  }  }
    return true;
}

bool any_elem_true( const std::vector<bool>& bulVec ){
    // Return true if any elements true , otherwise return false
    size_t len = bulVec.size();
    for( size_t i = 0 ; i < len ; i++ ){  if( bulVec[i] ){  return true;  }  }
    return false;
}

size_t count_elem_true( const std::vector<bool>& bulVec ){
    // Return the number of elements that are true
    size_t len       = bulVec.size() , 
           totalTrue = 0             ;
    for( size_t i = 0 ; i < len ; i++ ){  if( bulVec[i] ){  totalTrue++;  }  }
    return totalTrue;
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

size_t wrap_index_for_len( size_t index , size_t len ){  return index % len;  }

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

std::vector<double> vec_dbbl_zeros( size_t len ){
    std::vector<double> rntVec;
    for( size_t i = 0 ; i < len ; i++ ){  rntVec.push_back( 0.0 );  }
    return rntVec;
}

std::vector<std::vector<double>> vec_vec_dbbl_zeros( size_t len ){
    // Return a square vector of zeros
    std::vector<std::vector<double>> rtnVecVec;
    for( size_t i = 0 ; i < len ; i++ ){
        std::vector<double> temp;
        for( size_t j = 0 ; j < len ; j++ ){  temp.push_back( 0 );  }
        rtnVecVec.push_back( temp );
    }
    return rtnVecVec;
}

std::vector<double> err_vec( size_t len ){
    std::vector<double> rtnErr;
    for( size_t i = 0 ; i < len ; i++ ){  rtnErr.push_back( nan("") );  }
    return rtnErr;
}

bool is_err( const std::vector<double>& vec ){
    // Return true if all of the elements are NaN, Otherwise return false
    size_t len = vec.size();
    for( size_t i = 0 ; i < len ; i++ ){  if( !isnan( vec[i] ) ){ return false; }  }
    return true;
}

// __ End Container __

// === Functors ===

// == class Incrementer ==

Incrementer::Incrementer( llin start ){ count = start; } // Create an incrementer with a starting number

llin Incrementer::operator()(){ llin temp = count;  count++;  return temp;  } // Increment the counter and return it

// __ End Incrementer __

// ___ End Functors ___


// === Memory Utils ===

string pointer_info_str( void* generalPointer ){
    return string( "Pointer Null?: " ) 
         + string( generalPointer == nullptr ? "YES" : "NO" ) 
         + string( " , Data: " )
         + string( (char*)generalPointer );
}

// ___ End Memory ___



void print_binary_int( unsigned int arg ){
    // print the binary representation of a uint
    printf( "0b " );
    for( int i  = 0 ; i < 32 ; i++ ){
        printf( "%i" , ( arg >> ( 31 - i ) ) & 1 );
        if( (i+1)%8 == 0 )  printf( " " );
    }
}

void print_hex_int( unsigned int a ){
	// Print the hexadecimal representation of a uint
	unsigned int byte;
	printf( "0x " );
	for( int i = 3 ; i > -1 ; i-- ){ 
		byte = ( a >> ( i*8 ) ) & 0xFF;
		printf( "%02hhX " , byte );
	}
}

void print_binary_char( unsigned char arg ){
    // print the binary representation of a uchar
    printf( "0b " );
    for( int i  = 0 ; i < 8 ; i++ ){
        printf( "%i" , ( arg >> ( 7 - i ) ) & 1 );
    }
}

void random_seed_RUN_ONCE_MAIN(){
    srand( time(NULL) ); // init random
}

double rand_01(){ // NOTE: Not the best way for uniform rand, but is EASIEST
    // Return a random number on [0,1)
    // NOTE: This function assumes that an appropriate random seed was set in `main`
    return rand() / ( RAND_MAX + 1.0 );
}

double time_elapsed( StdTime& clok ){
    StdTime t = StdClock::now();  // 1. Get the time now
    // 2. Compute span and convert microseconds to seconds
    double elapsed = 
        duration_cast<microseconds>(t - clok).count()
        / (double) 1e6;
    // 3. Reset the time marker var        
    clok = t;
    // 4. Return
    return elapsed;
}

void fill_array_rand( double arr[] , size_t len ){
    // Populate an array with random doubles on [0,1)
    for( size_t i = 0 ; i < len ; i++ ){
        arr[i] = rand_01();
    }
}

string yes_no( bool expr ){
    // Return an affirmative string for true -or- a negative string for false
    return ( expr ? "YES" : "NO" );
}

ostream& operator<<( ostream& os , const std::vector<u_char>& vec ) { // ostream '<<' operator for vectors
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "[ ";
    for (size_t i = 0; i < vec.size(); i++) {
        os << (long long int) vec[i];
        if (i + 1 < vec.size()) { os << ", "; }
    }
    os << " ]";
    return os; // You must return a reference to the stream!
}

bool check_exist( string path , string check ){
    // Use `stat` to check if something exists
    struct stat buffer;   
    bool /*- */ result;
    stat( path.c_str() , &buffer );

    if( check == "f" || check == "file" || check == "F" || check == "FILE" )
        result = S_ISREG( buffer.st_mode );
    else if( check == "d" || check == "directory" || check == "D" || check == "DIRECTORY" )
        result =  S_ISDIR( buffer.st_mode );
    else
        result = false;

    return result;
}

vector<string> split( string s , char sep ){
    vector<string> rtnVec;
    string currWord; // ------------- Current word between separators
    s.append( string( 1 , sep ) ); // Delimiter termination hack
    int len = s.length(); // -------- Length of input string
    // For each char in the string
    for( int i = 0 ; i < len ; i++ ){
        if( s[i] != sep ){ // If not separator, accumulate char to `currWord`
            currWord += s[i];
        }else{ // else is separator, 
            // add word if we accumulated one
            if( currWord.length() > 0 ){  rtnVec.push_back( currWord );  } // Assign word to array
            currWord = "";
        }
    }
    return rtnVec;
}

vector<string> list_path( string path ){
    // List all the files in a directory
    // https://stackoverflow.com/a/37494654
    vector<string> rtnVec;
    for( const auto& entry : fs::directory_iterator( path ) ){  rtnVec.push_back( entry.path().string() );  }
    return rtnVec;
}

bool has_substr( string& superStr , string& subStr ){
    // Return true if `superStr` containst `subStr`, otherwise return false
    std::size_t found = superStr.find( subStr );
    return found != string::npos;
}

void print_args( int c, char *v[] ){
    cout << "print_args! " << c << endl;
    for( int i = 0 ; i < c ; i++ ){  
        cout << *(v[i]) << ( i+1 >= c ? "\n" : ", " );  
    }
}

string get_file_string( string path ){
    // Get the contents of the file at `path` as a string
    std::ifstream f( path );
    string rtnStr;
    bool _DEBUG = false;
    if( f ){
        std::stringstream buf;
        buf << f.rdbuf();
        f.close();
        rtnStr = buf.str();
    }else{
        cout << "File " << path << " could NOT be opened!" << endl;
    }
    int strLen = rtnStr.length();
    if( strLen == 0 )
        cout << "WARN: " << path << " yielded an EMPTY string!" << endl;
    else if( _DEBUG )
        cout << path << " yielded " << strLen << " characters.""" << endl;
    return rtnStr;
}
