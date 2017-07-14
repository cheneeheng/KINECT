/*
 * COS.h
 *
 *  Created on: May 22, 2017
 *      Author: chen
 */

#ifndef COS_H_
#define COS_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>

class COS
{
private:
	using mSmSI_t = std::map<std::string, std::map<std::string, int> >;
	using mSvvD_t = std::map<std::string, std::vector<std::vector<double> > >;

	int label_os;
	std::vector<std::string> label_list_os; // list of object state labels
	mSmSI_t la_os; // list of object state labels
	mSvvD_t transition_os; // transition between the object state
	mSvvD_t transition_os_la;
	mSvvD_t transition_la_os;

public:
	COS();
	virtual ~COS();

	/*
	 * current os label
	 */
	virtual int OSLabel() const
	{
		return label_os;
	}
	virtual void OSLabel(
			int x_)
	{
		label_os = x_;
	}

	/*
	 * list of object state labels
	 */
	virtual std::vector<std::string> OSLabelList() const
	{
		return label_list_os;
	}
	virtual void OSLabelList(
			std::vector<std::string> x_)
	{
		label_list_os = x_;
	}

	/*
	 * list of LA object state correspondence
	 */
	virtual mSmSI_t LAOSMap() const
	{
		return la_os;
	}
	virtual void LAOSMap(
			mSmSI_t x_)
	{
		la_os = x_;
	}

	/*
	 * P(O_t|LA_t)
	 */
	virtual mSvvD_t TransitionLAOS() const
	{
		return transition_la_os;
	}
	virtual void TransitionLAOS(
			mSvvD_t x_)
	{
		transition_la_os = x_;
	}

	/*
	 * P(LA_t|O_t)
	 */
	virtual mSvvD_t TransitionOSLA() const
	{
		return transition_os_la;
	}
	virtual void TransitionOSLA(
			mSvvD_t x_)
	{
		transition_os_la = x_;
	}

	/*
	 * P(O_t|O_t-1)
	 */
	virtual mSvvD_t TransitionOS() const
	{
		return transition_os;
	}
	virtual void TransitionOS(
			mSvvD_t x_)
	{
		transition_os = x_;
	}
};

#endif /* COS_H_ */
