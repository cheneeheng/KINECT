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

/**
 * Grasp states.
 *
 * @key RELEASE Released state.
 * @key GRABBED Grasped state.
 * @key RELEASED_CLOSE Trigger for release.
 * @key GRABBED_CLOSE Trigger for grasp.
 */
enum class GRAB
{
	RELEASE, GRABBED, RELEASED_CLOSE, GRABBED_CLOSE
};

/**
 * Container for action states.
 * Member functions are used to retrieve and modify action states.
 */
class CAS
{
private:
	//typedef std::map<std::string,double> ms;
	using msD_t = std::map<std::string,double>;

	// Grasp state
	GRAB grasp;

	// Start node
	int label1;

	// Goal node
	int label2;

	// Object state, one hot vector based on dictionary.
	int obj;

	// Velocity
	double vel;

	// Highest goal probability
	double pct_err;

	// Which known surface that is currently being evaluated
	int surface_flag;

	// Surface label
	std::string surface_name;

	// Distance to surface
	double surface_dist;

	// Goal probabilities
	msD_t goal;

	// Maximum variances.
	msD_t window;

public:
	CAS();
	virtual ~CAS();

	virtual GRAB Grasp() const
	{
		return grasp;
	}
	virtual void Grasp(
			GRAB x_)
	{
		grasp = x_;
	}

	virtual int Label1() const
	{
		return label1;
	}
	virtual void Label1(
			int x_)
	{
		label1 = x_;
	}

	virtual int Label2() const
	{
		return label2;
	}
	virtual void Label2(
			int x_)
	{
		label2 = x_;
	}

	virtual int OS() const
	{
		return obj;
	}
	virtual void OS(
			int x_)
	{
		obj = x_;
	}

	virtual double Velocity() const
	{
		return vel;
	}
	virtual void Velocity(
			double x_)
	{
		vel = x_;
	}

	virtual double Probability() const
	{
		return pct_err;
	}
	virtual void Probability(
			double x_)
	{
		pct_err = x_;
	}

	virtual int SurfaceFlag() const
	{
		return surface_flag;
	}
	virtual void SurfaceFlag(
			int x_)
	{
		surface_flag = x_;
	}

	virtual double SurfaceDistance() const
	{
		return surface_dist;
	}
	virtual void SurfaceDistance(
			double x_)
	{
		surface_dist = x_;
	}

	virtual std::string SurfaceName() const
	{
		return surface_name;
	}
	virtual void SurfaceName(
			std::string x_)
	{
		surface_name = x_;
	}

	virtual msD_t Goal() const
	{
		return goal;
	}
	virtual void Goal(
			msD_t x_)
	{
		goal = x_;
	}

	virtual msD_t Window() const
	{
		return window;
	}
	virtual void Window(
			msD_t x_)
	{
		window = x_;
	}
};

#endif /* CAS_H_ */
