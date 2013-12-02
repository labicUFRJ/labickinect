/*
 * reconstruction_exception.h
 *
 *  Created on: Dec 2, 2013
 *      Author: macecchi
 */

#ifndef RECONSTRUCTION_EXCEPTION_H_
#define RECONSTRUCTION_EXCEPTION_H_

#include <exception>

namespace labic {

class ReconstructionException: public std::exception {
public:
	ReconstructionException(std::string msg): msg(msg) {}
	~ReconstructionException() throw() {}
	virtual const char* what() const throw() { return msg.c_str(); }
private:
	std::string msg;
};

}



#endif /* RECONSTRUCTION_EXCEPTION_H_ */
