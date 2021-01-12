#ifndef CONTAINERHELPERS_H
#define CONTAINERHELPERS_H

#include <vector>
#include <iostream>
#include <list> // ----------- standard list datatype

#include "Cpp_Helpers.hpp"

using std::vector;
using std::ostream;
using std::list;
using std::endl;

// a28102be8c2d19c8c5281d38689703ab9f14ef61
struct IndexSearchResult{ // A container to hold a search result for an index that cannot have a negative value
    bool   result; // Is the result a valid one?
    size_t index; //- If so, which is the index we like best?
};

size_t random_false_elem_index( std::vector<bool> vec );
std::vector<std::vector<size_t>> enumerate_in_base( size_t digits , size_t base );
ostream& operator<<( ostream& os , const std::vector<u_char>& vec );

template<typename FLT>
vector<FLT> randrange_vec( FLT lo , FLT hi , size_t len ){
    std::vector<FLT> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( randrange( lo , hi ) );  }
    return rtnVec;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
ostream& operator<<( ostream& os , const vector<T>& vec ) { // ostream '<<' operator for vectors
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
ostream& operator<<( ostream& os , const vector<vector<T>>& vecVec ) { // ostream '<<' operator for vectors
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "[ ";
    for (size_t i = 0; i < vecVec.size(); i++) {
        os << "  " << vecVec[i];
        if (i + 1 < vecVec.size()) { os << ", " << endl; }
    }
    os << " ]";
    return os; // You must return a reference to the stream!
}

template<typename T1 , typename T2> // NOTE: Templated functions must have their definition in the header file
ostream& operator<<( ostream& os , const vector<std::pair<T1,T2>>& vec ) { // ostream '<<' operator for pair vectors
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "[ ";
    for (size_t i = 0; i < vec.size(); i++) {
        os << "(" << std::get<0>( vec[i] ) << " , " << std::get<1>( vec[i] ) << ")" ;
        if (i + 1 < vec.size()) { os << ", "; }
    }
    os << " ]";
    return os; // You must return a reference to the stream!
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
ostream& operator<<( ostream& os , const vector<std::pair<T,T>>& vec ) { // ostream '<<' operator for pair vectors
    // NOTE: This function assumes that the ostream '<<' operator for T has already been defined
    os << "[ ";
    for (size_t i = 0; i < vec.size(); i++) {
        os << "(" << std::get<0>( vec[i] ) << " , " << std::get<1>( vec[i] ) << ")" ;
        if (i + 1 < vec.size()) { os << ", "; }
    }
    os << " ]";
    return os; // You must return a reference to the stream!
}

template<typename T>
size_t last_index( const vector<T>& vec ){  return vec.size() - 1;  }

template<typename T>
T last_elem( const vector<T>& vec ){  return vec[ vec.size() - 1 ];  } // NOTE: This function assumes there is at least one element

template<typename T>
vector<T> vec_copy( vector<T>& original ){ // NOTE: There is a const arg version as well
    size_t len = original.size();
    vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( original[i] );  }
    return rtnVec;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
void extend_vec_with( vector<T>& original , const vector<T>& extension ){
    size_t xtLen = extension.size();
    for( size_t i = 0 ; i < xtLen ; i++ ){  original.push_back( extension[i] );  } // Using this so that you can extend vector with itself
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
vector<T> vec_join( const vector<T>& vec1 , const vector<T>& vec2 ){
    // Return the vector that is the concatenation of 'vec1' and 'vec2'
    vector<T> rtnVec;
    extend_vec_with( rtnVec , vec1 );
    extend_vec_with( rtnVec , vec2 );
    return rtnVec;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
vector<T> vec_plus_one( vector<T>& original , T plusOne ){
    vector<T> rtnVec;
    rtnVec = vec_copy( original );
    rtnVec.push_back( plusOne );
    return rtnVec;
}	

template<typename T> // NOTE: Templated functions must have their definition in the header file
void extend_vec_vec_with( vector<vector<T>>& original , vector<vector<T>>& extension ){
    size_t xtLen = extension.size();
    for( size_t i = 0 ; i < xtLen ; i++ ){  
        vector<T> temp = vec_copy( extension[i] );
        original.push_back( temp );  
    } // Using this so that you can extend vector with itself
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
vector<T> vec_range( T lo , T hi ){
    T i = 0;
    vector<T> rtnVec;
    if( lo == hi )
        rtnVec = { lo };
    else if( lo < hi )
        for( i = lo ; i <= hi ; i++ ){ rtnVec.push_back( i ); }
    else
        for( i = hi ; i >= lo ; i-- ){ rtnVec.push_back( i ); }
    return rtnVec;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
T min_num_in_vec( vector<T> searchVec ){
    T      least   = searchVec[0]; 
    size_t i       = 0                , 
           numElem = searchVec.size() ;
    for( i = 0 ; i < numElem ; i++ ){ 
        least = min( searchVec[i] , least ); 
    } 
    return least;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
T max_num_in_vec( const vector<T>& searchVec ){ // DEPRECATED , THOUGH USED IN ASP
    // NOTE: This function assumes that 'searchVec' has at least one element
    T      most    = searchVec[0]; 
    size_t i       = 0                , 
           numElem = searchVec.size() ;
    for( i = 1 ; i < numElem ; i++ ){ 
        most = max( searchVec[i] , most ); 
    } 
    return most;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
T max( const vector<T>& searchVec ){ 
    // NOTE: This function assumes that 'searchVec' has at least one element
    size_t i       = 0                , 
           numElem = searchVec.size() ;
    T      most    = searchVec[0]; 
    for( i = 1 ; i < numElem ; i++ ){ 
        most = max( searchVec[i] , most ); 
    } 
    return most;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
size_t min_index_in_vec( vector<T> searchVec ){
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
size_t max_index_in_vec( vector<T> searchVec ){
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
    if( lo == hi ){
        rtnVec.push_back( lo );
    }else if( lo < hi )
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
bool is_arg_in_vector( T arg , const vector<T>& vec ){
    // Return true if 'arg' is in 'st' , false otherwise
    // URL , resolve dependent templated typenames: https://stackoverflow.com/a/11275548
    // URL , const_iterator: https://stackoverflow.com/a/309589
    typename vector<T>::const_iterator it = find( vec.begin() , vec.end() , arg ); 
    return it != vec.end();
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
vector<T> vec_intersection( const vector<T>& vec1 , const vector<T>& vec2 ){
    // Return a vector of all the common elements of 'vec1' and 'vec2'
    vector<T> rtnVec;
    size_t len = vec1.size();
    for( size_t i = 0 ; i < len ; i++ ){  if( is_arg_in_vector( vec1[i] , vec2 ) ){  rtnVec.push_back( vec1[i] );  }  }
    return rtnVec;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
size_t count_lessThan_in_vec( T query , vector<T>& searchVec ){
    // Return the number of elements of 'searchVec' that are less than 'query'
    size_t count = 0                , 
           len   = searchVec.size() ;
    for( size_t i = 0 ; i < len ; i++ ){  if( searchVec[i] < query ){  count++;  }  }
    return count;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
size_t count_grtrThan_in_vec( T query , vector<T>& searchVec ){
    // Return the number of elements of 'searchVec' that are less than 'query'
    size_t count = 0                , 
           len   = searchVec.size() ;
    for( size_t i = 0 ; i < len ; i++ ){  if( searchVec[i] > query ){  count++;  }  }
    return count;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
size_t count_elems_also_in( vector<T>& searchVec , vector<T>& compareVec ){
    // Return the number of elements of 'searchVec' that can be found in 'compareVec' (Repeats counted)
    size_t count = 0                , 
           len   = searchVec.size() ;
    for( size_t i = 0 ; i < len ; i++ ){  if( is_arg_in_vector( searchVec[i] , compareVec ) ){  count++;  }  }
    return count;
}

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool vec2_contains_vec1( const vector<T>& vec1 , const vector<T>& vec2 ){
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
bool vec_same_contents( const vector<T>& vec1 , const vector<T>& vec2 ){
    // Return true if and only if all of the elements of 'vec1' are in 'vec2' and vice-versa
    return vec2_contains_vec1( vec1 , vec2 ) && vec2_contains_vec1( vec2 , vec1 );
}

template<typename T>
T rand_choice( vector<T> searchVec ){ return searchVec[ randrange( searchVec.size() ) ]; }

template<typename T>
size_t wrap_index_for_vec( size_t index , const vector<T>& vec ){  return index % vec.size();  }

template<typename T>
vector<T> vec_copy_without_elem( vector<T>& original , T holdout ){ // non-const params
    // NOTE: This function assumes that the equality operator is defined for type T
    size_t len = original.size();
    vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  if( original[i] != holdout ){  rtnVec.push_back( original[i] );  }  }
    return rtnVec;
}

template<typename T>
vector<T> vec_copy_without_elem( const vector<T>& original , T holdout ){ // const params
    // NOTE: This function assumes that the equality operator is defined for type T
    size_t len = original.size();
    vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  if( original[i] != holdout ){  rtnVec.push_back( original[i] );  }  }
    return rtnVec;
}

template<typename T>
vector<T> vec_copy_without_elem( const vector<T>& original , const vector<T>& holdout ){
    // NOTE: This function assumes that the equality operator is defined for type T
    size_t len = original.size();
    vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  if( !is_arg_in_vector( original[i] , holdout ) ){  rtnVec.push_back( original[i] );  }  }
    return rtnVec;
}

template<typename T>
vector<T> vec_copy_without_elem( const vector<T>& original , T holdElem , const vector<T>& holdVec ){
    // NOTE: This function assumes that the equality operator is defined for type T
    size_t len = original.size();
    vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  
        if(   ( !is_arg_in_vector( original[i] , holdVec ) )  &&  ( original[i] != holdElem )  ){  rtnVec.push_back( original[i] );  }  
    }
    return rtnVec;
}

template<typename T>
vector<T> vec_copy_without_elem( const vector<T>& original , const vector<vector<T>>& holdVec ){
    // vec_vec version // Would this be faster with a set?
    vector<T> rtnVec = vec_copy( original );
    size_t lenI = holdVec.size();
    // Successively remove elements based on the contents of each vec in 'holdVec'
    for( size_t i = 0 ; i < lenI ; i++ ){  rtnVec = vec_copy_without_elem( rtnVec , holdVec[i] );  }
    return rtnVec;
}

template<typename T>
vector<T> vec_copy_one_index( const vector<T>& original , size_t index ){
    vector<T> rtnVec;
    if( index < original.size() ){  rtnVec.push_back( original[ index ] );  }
    return rtnVec;
}

template<typename T>
vector<T> vec_copy_shuffled( const vector<T>& original ){
    vector<T> rtnVec = vec_copy( original );
    std::random_shuffle( rtnVec.begin() , rtnVec.end() );
    return rtnVec;
}

template<typename T>
void vec_assign_all_same( vector<T>& original , T sameVal ){
    // Assign each element to be the same value
    size_t len = original.size();
    for( size_t i = 0 ; i < len ; i++ ){  original[i] = sameVal;  }
}

template<typename T>
vector< size_t > vec_vec_len( vector<vector<T>>& vecVec ){
    vector<size_t> rtnVec;
    size_t len = vecVec.size();
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( vecVec[i].size() );  }
    return rtnVec;
}

template<typename T>
size_t vec_vec_longest_rep( vector<vector<T>>& vecVec ){
    // Return the length of the longest string representation of an element of a 2D vector
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
size_t vec_vec_longest_rep( const vector<vector<T>>& vecVec ){
    // Return the length of the longest string representation of an element of a 2D vector
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
void print_vec_vec( vector<vector<T>>& vecVec , size_t padLen = 1 ){
    // Return the length of the longest string representation of an element of a 2D vector
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
void print_vec_vec( const vector<vector<T>>& vecVec , size_t padLen = 1 ){
    // Return the length of the longest string representation of an element of a 2D vector
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
size_t vec_vec_any_empty( vector<vector<T>>& vecVec ){
    // Return true if any of the sub-vectors are empty, otherwise return false
    size_t len = vecVec.size();
    for( size_t i = 0 ; i < len ; i++ ){  if( vecVec[i].size() == 0 ){  return true;  }  }
    return false;
}

template<typename T>
vector<vector<T>> vec_vec_copy_nonempty( vector<vector<T>>& vecVec ){
    // Return a copy of 'vecVec' that contains only the non-empty sub-vectors
    size_t len    = vecVec.size() , 
           subLen = 0             ;
    vector<vector<T>> rtnVecVec;
    for( size_t i = 0 ; i < len ; i++ ){  
        if( vecVec[i].size() > 0 ){
            vector<T> temp = vec_copy( vecVec[i] );
            rtnVecVec.push_back( temp );
        }  
    }
    return rtnVecVec;
}

template<typename T>
vector<vector<T>> vec_vec_copy( vector<vector<T>>& original ){
    size_t len_i , len_j;
    vector<vector<T>> rtnVecVec;
    len_i = original.size();
    for( size_t i = 0 ; i < len_i ; i++ ){
        len_j = original[i].size();
        vector<T> temp;
        for( size_t j = 0 ; j < len_j ; j++ ){
            temp.push_back( original[i][j] );
        }
        rtnVecVec.push_back( temp );
    }
    return rtnVecVec;
}

template<typename T>
vector<vector<T>> vec_vec_copy( const vector<vector<T>>& original ){
    size_t len_i , len_j;
    vector<vector<T>> rtnVecVec;
    len_i = original.size();
    for( size_t i = 0 ; i < len_i ; i++ ){
        len_j = original[i].size();
        vector<T> temp;
        for( size_t j = 0 ; j < len_j ; j++ ){
            temp.push_back( original[i][j] );
        }
        rtnVecVec.push_back( temp );
    }
    return rtnVecVec;
}

template<typename T>
void vec_vec_bottom_clear( vector<vector<T>>& vecVec ){
    // Clear the bottom-level vectors
    size_t len = vecVec.size();
    for( size_t i = 0 ; i < len ; i++ ){  vecVec[i].clear();  }
}

template<typename T>
void vec_vec_top_populate( vector<vector<T>>& vecVec , size_t N ){
    // Populate the top level with empty vectors
    vecVec.clear();
    for( size_t i = 0 ; i < N ; i++ ){
        vector<T> temp;
        vecVec.push_back( temp );
    }
}

template<typename T>
vector<vector<T>> row_vec_to_col_vec_vec( vector<T>& original ){
    // Convert a 1D standard vector into a standard 2D nested vector in which each vector has one element of the 'original'
    size_t len = original.size();
    vector<vector<T>> rtnVecVec;
    for( size_t i = 0 ; i < len ; i++ ){
        vector<T> temp;
        temp.push_back( original[i] );
        rtnVecVec.push_back( temp );
    }
    return rtnVecVec;
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
vector<T> list_to_vec( std::list<T>& inputList ){
    // Return a list composed of all of the elements of 'inputList' , In order
    vector<T> rtnVec;
    typename std::list<T>::iterator it; // URL , resolve dependent templated typenames: https://stackoverflow.com/a/11275548
    for( it = inputList.begin() ; it != inputList.end() ; ++it ){
        rtnVec.push_back( *it );
    }
    return rtnVec;
}

template<typename T>
vector<T> linspace( T a , T b , size_t N ){
    // Original by Lorenzo Riano , URL: https://gist.github.com/lorenzoriano/5414671
    // NOTE: If N = 1 , A vector with only 'a' will be returned
    // NOTE: If N = 0 , An empty vector is returned
    T h = ( b - a ) / static_cast<T>( N - 1 );
    vector<T> xs( N );
    typename vector<T>::iterator x;
    T val;
    // This loop structure is unusual , will look at later
    for ( x = xs.begin() , val = a ; x != xs.end() ; ++x , val += h ){  *x = val;  }
    return xs;
}

template<typename T>
vector<T> operator*( const vector<T>& opVec , T opScl ){
    // Scale all of the elements of 'opVec' by 'opScl'
    vector<T> rtnVec;
    size_t len = opVec.size();
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( opVec[i] * opScl );  }
    return rtnVec;
}

template<typename TFLT1 , typename TFLT2>
vector<TFLT1> operator*( const vector<TFLT1>& opVec , TFLT2 opScl ){  return opVec * (TFLT1)opScl;  }

template<typename T>
vector<T> operator-( const vector<T>& op1 , const vector<T>& op2 ){
    // Element-wise subtraction of op1 - op2
    vector<T> rtnVec;
    size_t len = op1.size();
    if( len == op2.size() ){
        for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( op1[i] - op2[i] );  }
        return rtnVec;
    }else{
        throw std::out_of_range ( "operator- vector<T> , Vector length mismatch: " + to_string( len ) + " and " + to_string( op2.size() ) );
    }
}

template<typename T>
vector<T> operator/( const vector<T>& op1 , const vector<T>& op2 ){
    // Element-wise division of op1 / op2
    vector<T> rtnVec;
    size_t len = op1.size();
    if( len == op2.size() ){
        for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( op1[i] / op2[i] );  }
        return rtnVec;
    }else{
        throw std::out_of_range ( "operator/ vector<T> , Vector length mismatch: " + to_string( len ) + " and " + to_string( op2.size() ) );
    }
}

template<typename T>
vector<T>& operator+=( vector<T>& opLeft , T opRght ){
    size_t len = opLeft.size();
    for( size_t i = 0 ; i < len ; i++ ){  opLeft[i] += opRght;  }
    return opLeft;
}

template<typename T>
vector<T>& operator+=( vector<T>& opLeft , vector<T>& opRght ){
    // Element-wise add-assign 'opLeft'
    // NOTE: Add-assign operations are only carried out through shared number of elements
    size_t len = min( opLeft.size() , opRght.size() );
    for( size_t i = 0 ; i < len ; i++ ){  opLeft[i] += opRght[i];  }
    return opLeft;
}

template<typename T>
vector<T> clamp_vec( vector<T>& raw , T lower , T upper ){
    // Clamp each value in 'raw' to within 'lower' and 'upper'
    stdvec<T> rtnVec;
    size_t    len = raw.size();
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( clamp_val( raw[i] , lower , upper ) );  }
    return rtnVec;
}

template<typename T>
vector<T> abs( const vector<T>& op1 ){
    // Element-wise absolute value: abs( op1[i] ) 
    vector<T> rtnVec;
    size_t len = op1.size();
    for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( abs( op1[i] ) );  }
    return rtnVec;
}

template< typename T , typename INT >
vector<T> subvec_of_from_to( const vector<T>& superVec , INT bgn , INT end ){
    size_t len = superVec.size();
    if(  ( bgn < len )  &&  ( end < len )  ){
        if( bgn > end ){  size_t swap = bgn;  bgn = end;  end = swap;  }
        // URL , Fetch sub-vector: https://stackoverflow.com/a/421615
        typename vector<T>::const_iterator it_bgn = superVec.begin() + bgn;
        typename vector<T>::const_iterator it_end = superVec.begin() + end + 1; // I do not prefer the Python slicing indices
        vector<T> rtnVec( it_bgn , it_end );
        return rtnVec;
    }else{
        throw std::out_of_range ( "subvec_of_from_to , An index was out of range , bgn: " + to_string( bgn ) + " , end: " + to_string( end )
                                + " out of " + to_string( len ) );
    }
}

template<typename T>
T most_numerous_value_in( const vector<T>& vec ){
    // Return the value that appears the most number of times in 
    // NOTE: If each value appears only once, then the first element will be returned
    size_t len           = vec.size() ,
           greatestCount = 0 , 
           elemCount     = 0 ;
    T      mostVal       = vec[0];
    for( size_t i = 0 ; i < len ; i++ ){
        elemCount = std::count( vec.begin() , vec.end() , vec[i] );
        if( elemCount > greatestCount ){
            greatestCount = elemCount;
            mostVal       = vec[i];
        }
    }
    return mostVal;
}

template<typename T>
vector<size_t> all_indices_equal_to_val( const vector<T>& vec , T val ){
    size_t len = vec.size();
    vector<size_t> matchDices;
    for( size_t i = 0 ; i < len ; i++ ){  if( vec[i] == val ){  matchDices.push_back( i );  }  }
    return matchDices;
}

// = Maps =

template < typename T1 , typename T2 >
vector<T1> map_keys_vec( std::map< T1 , T2 >& dict ){
    vector<T1>  rtnVec;
    typename std::map< T1 , T2 >::iterator it;
    for( it = dict.begin() ; it != dict.end() ; ++it ){  rtnVec.push_back( it->first );  }
    return rtnVec;
}

template < typename T1 , typename T2 >
vector<T1> map_has_key( const std::map< T1 , T2 >& dict , T1 key ){
    typename std::map< T1 , T2 >::iterator it = dict.find( key );
    return it != dict.end();
}

// _ End Maps _


// = Sets =

template<typename T> // NOTE: Templated functions must have their definition in the header file
bool is_arg_in_set( T arg , const std::set<T>& st ){
    // Return true if 'arg' is in 'st' , false otherwise
    // URL , C++ cannot recognize a templated typename
    typename std::set<T>::iterator it = st.find( arg );
    return it != st.end();
}

template<typename T> 
void set_insert_vec( std::set<T>& st , const vector<T>& vec ){
    // Insert all of the elements of 'vec' into 'st'.  'st' will automatically reject repeats
    size_t len = vec.size();
    for( size_t i = 0 ; i < len ; i++ ){  st.insert( vec[i] );  }
}

template<typename T> 
vector<T> set_to_vec( const std::set<T>& st ){
    // Return a vector that contains all of the elements in the set
    vector<T> rtnVec;
    for( typename std::set<T>::iterator it = st.begin() ; it != st.end() ; ++it ){  rtnVec.push_back( *it );  }
    return rtnVec;
}

template<typename T> 
vector<T> vec_minus_set( const vector<T>& vec , const std::set<T>& st ){ // This is an alias of the below , sorry
    // Return a vector of all the elements of 'vec' NOT in 'st'
    vector<T> rtnVec;
    size_t len = vec.size();
    for( size_t i = 0 ; i < len ; i++ ){  if( !is_arg_in_set( vec[i] , st ) ){  rtnVec.push_back( vec[i] );  }  }
    return rtnVec;
}

template<typename T>
vector<T> vec_copy_without_elem( const vector<T>& original , const std::set<T>& holdout ){ // This is an alias of the above , sorry
    // NOTE: This function assumes that the equality operator is defined for type T
    size_t len = original.size();
    vector<T> rtnVec;
    for( size_t i = 0 ; i < len ; i++ ){  if( !is_arg_in_set( original[i] , holdout ) ){  rtnVec.push_back( original[i] );  }  }
    return rtnVec;
}

template<typename T> 
vector<T> vec_unique_only( const vector<T>& vec ){
    // Return a copy of 'vec' with only unique elements
    size_t len = vec.size();
    std::set<T> st;
    for( size_t i = 0 ; i < len ; i++ ){  st.insert( vec[i] );  }
    return set_to_vec( st );
}

// _ End Sets _

// = Queues =

// Empty the queue and discard all the values
template<typename T> // NOTE: Templated functions must have their definition in the header file
void erase_queue( std::queue<T>& Q ){    while( Q.size() > 0 ){  Q.pop();  }    }

template<typename T> // NOTE: Templated functions must have their definition in the header file
T queue_get_pop( std::queue<T>& Q ){ 
    // Get the front item, then pop it
    T temp = Q.front();
    Q.pop();
    return temp;
}

template<typename T> 
void enqueue_vec( std::queue<T>& Q , const vector<T>& additions ){
    size_t len = additions.size();
    for( size_t i = 0 ; i < len ; i++ ){  Q.push( additions[i] );  }
}

template<typename T> 
void enqueue_vec_not_in_set( std::queue<T>& Q , const vector<T>& additions , const std::set<T>& exclusions ){
    size_t len = additions.size();
    for( size_t i = 0 ; i < len ; i++ ){  
        if( !is_arg_in_set( additions[i] , exclusions ) ){  Q.push( additions[i] );  }
    }
}

template<typename T> 
void enqueue_vec_not_in_either_set( std::queue<T>& Q , const vector<T>& additions , const std::set<T>& ex1 , const std::set<T>& ex2 ){
    size_t len = additions.size();
    for( size_t i = 0 ; i < len ; i++ ){  
        if( 
            ( !is_arg_in_set( additions[i] , ex1 ) )
                &&
            ( !is_arg_in_set( additions[i] , ex2 ) )
        ){  Q.push( additions[i] );  }
    }
}

template< typename T , typename F > // NOTE: Templated functions must have their definition in the header file
T queue_get_pop( std::priority_queue< T , vector<T> , F >& Q ){ 
    // Get the front item, then pop it
    T temp = Q.top();
    Q.pop();
    return temp;
}

template< typename T , typename F > // NOTE: Templated functions must have their definition in the header file
void erase_queue( std::priority_queue< T , vector<T> , F >& Q ){    while( Q.size() > 0 ){  Q.pop();  }    };

// _ End Queue _

#endif