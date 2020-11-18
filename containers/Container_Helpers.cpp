#include "Container_Helpers.hpp"

size_t random_false_elem_index( std::vector<bool> vec ){
    // Return a random index of an element that has value 'false'
    std::vector<size_t> availableIndices;
    size_t vecLen = vec.size();
    // Build a vector of available 'false' indices so that we are guaranteed to make the right choice
    for( size_t i = 0 ; i < vecLen ; i++ ){ if( vec[i] == false ){ availableIndices.push_back( i ); } }
    return rand_choice( availableIndices );
}

std::vector<std::vector<size_t>> enumerate_in_base( size_t digits , size_t base ){
    // Return all possible combinations of digits in 'base' that have 'digits'
    std::vector<std::vector<size_t>> rtnVecVec;
    // Basement Case: User asked for no digits
    if( digits == 0 ){  return rtnVecVec;  }
    // Base Case: There is only one digit , Count from 0 to 'base'-1
    if( digits == 1 ){
        for( size_t i = 0 ; i < base ; i++ ){
            std::vector<size_t> temp = { i };
            rtnVecVec.push_back( temp );
        }
        return rtnVecVec;
    }
    // Recursive Case: There is more than 1 digit , Generate leading digits and append trailing digits
    // 1. Fetch trailing digits
    std::vector<std::vector<size_t>> trailingDigits = enumerate_in_base( digits-1 , base );
    size_t /* ------------------- */ trLen /* -- */ = trailingDigits.size();
    std::vector<size_t> /* ------ */ leadingDigit;
    // 2. Generate leading digits and append trailing digits
    for( size_t i = 0 ; i < base ; i++ ){
        leadingDigit = { i };
        for( size_t j = 0 ; j < trLen ; j++ ){  rtnVecVec.push_back(  vec_join( leadingDigit , trailingDigits[j] )  );  }
    }
    return rtnVecVec;
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