/*
 * DataParser.h
 *
 *  Created on: Apr 18, 2017
 *      Author: chen
 *      Detail: Parses data from a file.
 */

#ifndef DATAPARSER_H_
#define DATAPARSER_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>

#include <Eigen/Eigen>

class DataParser
{
public:

	/**
	 * Constructor for DataParser class.
	 */
	DataParser();

	/**
	 * Destructor for DataParser class.
	 */
	virtual ~DataParser();

	/**
	 * Clears parser.
	 */
	void ClearParser();

	/**
	 * Parse data without known label.
	 */
	virtual int ParseDataNoLabel();

	/**
	 * Parse data with known label.
	 */
	virtual int ParseData();

	/**
	 * Resets parser.
	 *
	 * @param data_ Data used by parser.
	 */
	virtual void SetDataParser(
			const std::vector<std::vector<std::string> > &data_);

	virtual Eigen::Vector4d GetFaceParser() const
	{
		return face_parser;
	}
	virtual std::vector<int> GetFrameParser() const
	{
		return frames_parser;
	}
	virtual std::vector<int> GetContactParser() const
	{
		return contact_parser;
	}
	virtual std::vector<std::string> GetLabelParser() const
	{
		return labels_parser;
	}
	virtual std::vector<Eigen::Vector4d> GetPointParser() const
	{
		return points_parser;
	}

protected:
	Eigen::Vector4d face_parser;				// Parsed face position
	std::vector<int> frames_parser;				// Parsed frame number
	std::vector<int> contact_parser;			// Parsed contact flag
	std::vector<std::string> labels_parser;		// Parsed labels
	std::vector<Eigen::Vector4d> points_parser;	// Parsed trajectory point

private:
	std::vector<std::vector<std::string> > data_parser;

};

#endif /* DATAPARSER_H_ */
