/*
 * prediction.h
 *
 *  Created on: Mar 14, 2017
 *      Author: chen
 */

#ifndef PREDICTION_H_
#define PREDICTION_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "Graph.h"
#include "labeling.h"

// ============================================================================
// Prediction
// ============================================================================

void triggerContact(
	point_t &p_,
	Graph Graph_);

void checkMotion(
	point_t pos_,
	point_t vel_,
	vector<vector<double> > surface_,
	limit_t limit,
	label_t &LABEL_);

void checkSector(
	pred_t &prediction_,
	vector<double> &t_val_,
	vector<int> &last_loc_,
	point_t pos_,
	Graph &Graph_,
	int label1_);

void motionPrediction(
	vector<int> &prediction_,
	vector<double> &t_val_,
	bool &flag_predict_,
	bool &flag_predict_last_,
	vector<double> &predict_in_,
	vector<double> &predict_err_,
	vector<double> &predict_in_last_,
	double &pow_dec_,
	Graph Graph_);

void locationPrediction(
	int location_num_,
	point_t pos_,
	point_t vel_,
	Graph Graph_,
	double surface_limit_,
	double angle_limit_,
	double vel_limit_,
	label_t &LABEL_);

void predictionEdge(
	pred_t &prediction_,
	Graph &Graph_,
	point_t pos_,
	point_t vel_,
	int label1_,
	vector<int> &last_loc_,
	limit_t limit,
	label_t &label_,
	bool &flag_predict_,
	bool &flag_predict_last_,
	double &pow_dec_);

void predictionNode(
	vector<vector<point_t> > pva_avg,
	point_t pos_,
	point_t vel_,
	int label1_,
	int label2_,
	Graph &Graph_,
	limit_t limit_,
	label_t &label_,
	bool flag_motion_,
	bool learn_);



#endif /* PREDICTION_H_ */
