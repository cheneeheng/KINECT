/*
 * Preprocessing.h
 *
 *  Created on: Jul 14, 2017
 *      Author: chen
 */

#ifndef PREPROCESSING_H_
#define PREPROCESSING_H_

#include "ReadFile.h"
#include "WriteFile.h"
#include <memory>

#include <Eigen/Eigen>

class Preprocessing
{
public:
	Preprocessing();
	virtual ~Preprocessing();

	void Transitions(
			const std::string &kb_path_,
			const std::vector<std::string> &objs_,
			std::map<std::string, std::vector<int> > idxs_);
	void Surfaces(
			const std::string &src_,
			const std::string &dst_);
private:
	template<class T>
	void Output(
			std::vector<std::vector<T> > x_);

};

#endif /* PREPROCESSING_H_ */
