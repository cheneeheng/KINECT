/*
 * CKB.h
 *
 *  Created on: May 20, 2017
 *      Author: chen
 *      Detail: Container for knowledge-base.
 */

#ifndef CKB_H_
#define CKB_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <memory>

#include <Eigen/Eigen>

/*
 * Container for information stored in knowledge-base.
 * Member functions are used to retrieve or modify data.
 */
class CKB
{

private:
	using msvvD_t = std::map<std::string, std::vector<std::vector<double> > >;
	using msmss_t = std::map<std::string, std::map<std::string, std::string> >;
	using mspII_t = std::map<std::string, std::pair<int, int> >;

	// Surface equation
	std::vector<Eigen::Vector4d> surface_eq;

	// Surface plane midpoint
	std::vector<Eigen::Vector3d> surface_mid;

 	// Surface plane minimum point
	std::vector<Eigen::Vector3d> surface_min;

 	// Surface plane maximum point
	std::vector<Eigen::Vector3d> surface_max;

   // Surface rotation matrix to align to up vector
	std::vector<Eigen::Matrix3d> surface_rot;

 	// Surface distance limit
	std::vector<double> surface_lim;

	// Action category
	mspII_t ac; 			
			
	// Action labels	
	std::vector<std::string> al; 	
	
	// Object labels	
	msmss_t ol;

	// Action transitions
	msvvD_t transition_action;

public:

	/**
	 * Constructor for CKB.
	 */
	CKB();

	/**
	 * Destructor for CKB.
	 */
	virtual ~CKB();

	virtual std::vector<Eigen::Vector4d> SurfaceEquation() const
	{
		return surface_eq;
	}
	virtual void SurfaceEquation(
			std::vector<Eigen::Vector4d> x_)
	{
		surface_eq = x_;
	}

	virtual std::vector<Eigen::Vector3d> SurfaceMidPoint() const
	{
		return surface_mid;
	}
	virtual void SurfaceMidPoint(
			std::vector<Eigen::Vector3d> x_)
	{
		surface_mid = x_;
	}

	virtual std::vector<Eigen::Vector3d> SurfaceMinPoint() const
	{
		return surface_min;
	}
	virtual void SurfaceMinPoint(
			std::vector<Eigen::Vector3d> x_)
	{
		surface_min = x_;
	}

	virtual std::vector<Eigen::Vector3d> SurfaceMaxPoint() const
	{
		return surface_max;
	}
	virtual void SurfaceMaxPoint(
			std::vector<Eigen::Vector3d> x_)
	{
		surface_max = x_;
	}

	virtual std::vector<Eigen::Matrix3d> SurfaceRotation() const
	{
		return surface_rot;
	}
	virtual void SurfaceRotation(
			std::vector<Eigen::Matrix3d> x_)
	{
		surface_rot = x_;
	}

	virtual std::vector<double> SurfaceLimit() const
	{
		return surface_lim;
	}
	virtual void SurfaceLimit(
			std::vector<double> x_)
	{
		surface_lim = x_;
	}

	virtual mspII_t AC() const
	{
		return ac;
	}
	virtual void AC(
			mspII_t x_)
	{
		ac = x_;
	}

	virtual std::vector<std::string> AL() const
	{
		return al;
	}
	virtual void AL(
			std::vector<std::string> x_)
	{
		al = x_;
	}

	virtual msmss_t OL() const
	{
		return ol;
	}
	virtual void OL(
			msmss_t x_)
	{
		ol = x_;
	}

	virtual msvvD_t TransitionLA() const
	{
		return transition_action;
	}
	virtual void TransitionLA(
			msvvD_t x_)
	{
		transition_action = x_;
	}

};

#endif /* CKB_H_ */
