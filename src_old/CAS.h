/*******************************************************************************
 * CAS.h
 *
 *  Created on: May 20, 2017
 *      Author: chen
 *      Detail: ActionState container.
 ******************************************************************************/

#ifndef CAS_H_
#define CAS_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>

class CAS
{

	/*
	 * Information needed for action state
	 */

public:
	CAS();
	virtual ~CAS();

	virtual int Grasp() const	{ return grasp; }
	virtual void Grasp(int x_) 	{ grasp = x_; }

	virtual int Label1() const	{ return label1; }
	virtual void Label1(int x_) { label1 = x_; }

	virtual int Label2() const	{ return label2; }
	virtual void Label2(int x_) { label2 = x_; }

	virtual int OS() const	{ return obj; }
	virtual void OS(int x_) { obj = x_; }

	virtual double Velocity() const	 { return vel; }
	virtual void Velocity(double x_) { vel = x_; }

	virtual double Probability() const	{ return pct_err; }
	virtual void Probability(double x_) { pct_err = x_; }

	virtual int SurfaceFlag() const  { return surface_flag; }
	virtual void SurfaceFlag(int x_) { surface_flag = x_; }

	virtual double SurfaceDistance() const	{ return surface_dist; }
	virtual void SurfaceDistance(double x_) { surface_dist = x_; }

	virtual std::string SurfaceName() const	{ return surface_name; }
	virtual void SurfaceName(std::string x_) { surface_name = x_; }

	virtual std::map<std::string,double> Goal() const	{ return goal; }
	virtual void Goal(std::map<std::string,double> x_) { goal = x_; }

	virtual std::map<std::string,double> Window() const	{ return window; }
	virtual void Window(std::map<std::string,double> x_) { window = x_; }

private:
	//typedef std::map<std::string,double> ms;
	using mS = std::map<std::string,double>;

	int grasp;					// if it is grasped
	int label1;					// last node
	int label2;					// next node
	int obj;					// object state
	double vel;					// velocity
	double pct_err;				// highest probability
	int surface_flag;			// which surface
	std::string surface_name;	// which surface
	double surface_dist;		// surface distance
	mS goal;	// probabilities
	mS window;	// window radius
};

#endif /* CAS_H_ */
