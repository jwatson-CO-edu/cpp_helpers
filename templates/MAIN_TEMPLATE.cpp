/*
MAIN_TEMPLATE.cpp
James Watson , YYYY MONTHNAME
A ONE-LINE DESCRIPTION OF THE FILE

Dependencies:
Template Version: 2017-09-23
*/

/* MS Visual Studio Checklist (Delete if not MSVS)
1. [ ] Copy/Create header and source folders under the directory that contains the main CPP
2. [ ] Copy/Create header and source files in their respective folders
3. [ ] (Right Click) --> Add the header files and the source files to their respective folders in the project
4. [ ] Project --> Properties --> C++ --> General : Add the above header folder to the list of include directories 
5. [ ] Project --> Properties --> C++ --> Preprocessor --> Definitions : Add "_CRT_SECURE_NO_WARNINGS" ( Voxelyze only ) 
6. [ ] Project --> Properties --> C++ : Reduce warning level to 2 ( Voxelyze only ) */

/* C++ Checklist 
1. [ ] Appropriate #include */

// === Init ================================================================================================================================

// ~~~ Includes ~~~
// ~~ Standard ( Common includes not already in Cpp_Helpers.h ) ~~ 
//#include <iomanip> // - Output precision and printing
//#include <exception> // error handling
//#include <stdexcept> // library provided error types like 'std::range_error'
//#include <tuple> // --- standard tuple datatype
//#include <cstddef> // - alias types names:  size_t, ptrdiff_t
//#include <cstring> // - C Strings: strcomp 
//#include <algorithm> // sort, search, min, max, sequence operations

// ~~ Special ~~

// ~~ Local ~~ 
#include "Cpp_Helpers.h" // Shortcuts and Aliases , and Functions of general use for C++ programming

// ___ End Init ____________________________________________________________________________________________________________________________


// === Program Functions & Classes ===


// ___ End Functions & Classes ___


// === Program Vars ===

// ~~ Stack Vars ~~

// ~~ Heap Structs ~~

// ___ End Vars ___


// === main ================================================================================================================================

int main( int argc , char** argv ){ // Main takes the terminal command and flags that called it as arguments
	
	srand( time( 0 ) ); // Random seed based on the current clock time
	sep( "I EXIST!" ); // Silly output test
	
	
	// DO STUFF HERE
	
	return 0; // I guess everything turned out alright at the end!
}

// ___ End main ____________________________________________________________________________________________________________________________


/* === Spare Parts =========================================================================================================================



   ___ End Spare ___________________________________________________________________________________________________________________________
*/
