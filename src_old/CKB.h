/*
 * CKB.h
 *
 *  Created on: May 20, 2017
 *      Author: chen
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

class CKB
{

/*
 * Information stored in knowledge-base
 */

public:
	CKB();
	virtual ~CKB();

	virtual std::vector<Eigen::Vector4d> SurfaceEquation() const	{ return surface_eq; }
	virtual void SurfaceEquation(std::vector<Eigen::Vector4d> x_) 	{ surface_eq = x_; }

	virtual std::vector<Eigen::Vector3d> SurfaceMidPoint() const	{ return surface_mid; }
	virtual void SurfaceMidPoint(std::vector<Eigen::Vector3d> x_) 	{ surface_mid = x_; }

	virtual std::vector<Eigen::Vector3d> SurfaceMinPoint() const	{ return surface_min; }
	virtual void SurfaceMinPoint(std::vector<Eigen::Vector3d> x_) 	{ surface_min = x_; }

	virtual std::vector<Eigen::Vector3d> SurfaceMaxPoint() const	{ return surface_max; }
	virtual void SurfaceMaxPoint(std::vector<Eigen::Vector3d> x_) 	{ surface_max = x_; }

	virtual std::vector<Eigen::Matrix3d> SurfaceRotation() const	{ return surface_rot; }
	virtual void SurfaceRotation(std::vector<Eigen::Matrix3d> x_) 	{ surface_rot = x_; }

	virtual std::vector<double> SurfaceLimit() const	{ return surface_lim; }
	virtual void SurfaceLimit(std::vector<double> x_) 	{ surface_lim = x_; }

	virtual std::map<int,std::vector<std::string> > Label() const	{ return label; }
	virtual void Label(std::map<int,std::vector<std::string> > x_)	{ label = x_; }

	virtual std::map<std::string,std::pair<int,int> > AC() const	{ return ac; }
	virtual void AC(std::map<std::string,std::pair<int,int> > x_)	{ ac = x_; }

	virtual std::vector<std::string> AL() const		{ return al; }
	virtual void AL(std::vector<std::string> x_)	{ al = x_; }

	virtual std::map<std::string,std::map<std::string,std::string> > OL() const	{ return ol; }
	virtual void OL(std::map<std::string,std::map<std::string,std::string> > x_)	{ ol = x_; }

	virtual std::map<std::string,std::vector<std::vector<double> > > TransitionLA() const	{ return transition_action; }
	virtual void TransitionLA(std::map<std::string,std::vector<std::vector<double> > > x_)	{ transition_action = x_; }

private:
	std::vector<Eigen::Vector4d> surface_eq;
	std::vector<Eigen::Vector3d> surface_mid;
	std::vector<Eigen::Vector3d> surface_min; // from mid
	std::vector<Eigen::Vector3d> surface_max; // from mid
	std::vector<Eigen::Matrix3d> surface_rot;
	std::vector<double>   		 surface_lim;
	std::map<int,std::vector<std::string> > label;
	std::map<std::string,std::pair<int,int> > ac;
	std::vector<std::string> al;
	std::map<std::string,std::map<std::string,std::string> > ol;
	std::map<std::string,std::vector<std::vector<double> > > transition_action;
};

#endif /* CKB_H_ */
