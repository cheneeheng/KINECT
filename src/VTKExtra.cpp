/*******************************************************************************
 * VTKExtra.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 *      Detail: For viewing purposes.
 ******************************************************************************/

//#define SAVING_IMAGE
#define DOC_DIR "/home/chen/HMAR/HMAR_Location/doc/"

#include "VTKExtra.h"

VTKExtra::VTKExtra(
		int loc_int_,
		int sec_int_)
		: LOC_INT(loc_int_),
				SEC_INT(sec_int_)
{
}

VTKExtra::~VTKExtra()
{
}

/*******************************************************************************
 * KEYBOARD
 ******************************************************************************/
// Define interaction style
class KeyPressInteractorStyle: public vtkInteractorStyleTrackballCamera
{
public:
	static KeyPressInteractorStyle* New();vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera)
	;

	void SetWin(
			vtkSmartPointer<vtkRenderWindow> x)
	{
		renwin = x;
	}
	void SetWinInter(
			vtkSmartPointer<vtkRenderWindowInteractor> x)
	{
		renwinintr = x;
	}

	virtual void OnKeyPress()
	{
		// Get the key press
		vtkRenderWindowInteractor *rwi = this->Interactor;
		std::string key = rwi->GetKeySym();

		// Handle an arrow key
		if (key == "l")
		{
			vtkSmartPointer<vtkGL2PSExporter> exp;
			exp = vtkSmartPointer<vtkGL2PSExporter>::New();
			exp->SetRenderWindow(renwin);
			exp->SetFileFormatToEPS();
			exp->CompressOff();
			exp->SetSortToSimple();
			exp->TextAsPathOn();
			exp->DrawBackgroundOn();
			exp->SetFilePrefix((std::string(DOC_DIR) + "LA_drink").c_str());
			exp->Write();
		}

		if (key == "m")
		{
			vtkSmartPointer<vtkGL2PSExporter> exp;
			exp = vtkSmartPointer<vtkGL2PSExporter>::New();
			exp->SetRenderWindow(renwin);
			exp->SetFileFormatToEPS();
			exp->CompressOff();
			exp->SetSortToSimple();
			exp->TextAsPathOn();
			exp->DrawBackgroundOn();
			exp->SetFilePrefix((std::string(DOC_DIR) + "SM_drink").c_str());
			exp->Write();
		}

//      if(key == "q")
//      {
//    	  renwinintr->GetRenderWindow()->Finalize();
//    	  renwinintr->TerminateApp();
//      }

// Forward events
		vtkInteractorStyleTrackballCamera::OnKeyPress();
	}

private:
	vtkSmartPointer<vtkRenderWindow> renwin;
	vtkSmartPointer<vtkRenderWindowInteractor> renwinintr;

};
vtkStandardNewMacro(KeyPressInteractorStyle);

/*******************************************************************************
 * MOUSE
 ******************************************************************************/

class customMouseInteractorStyle: public vtkInteractorStyleTrackballCamera
{
public:
	static customMouseInteractorStyle* New();vtkTypeMacro(customMouseInteractorStyle, vtkInteractorStyleTrackballCamera)
	;

	void SetLeftButton(
			int option_)
	{
		left_click_ = option_;
	}

	void SetColors(
			std::vector<std::vector<unsigned char> > color)
	{
		color_ = color;
	}

	void SetLabels(
			std::vector<std::string> label_)
	{
		label = label_;
	}

	void SetRefLabels(
			std::vector<std::string> label_)
	{
		label_ref = label_;
	}

	void SetLocations(
			std::vector<int> x)
	{
		location = x;
	}

	std::vector<std::string> GetLabels()
	{
		return label;
	}

	std::vector<int> GetLocations()
	{
		return location;
	}

	void SetNumberOfLabels(
			int x)
	{
		num_locations = x;
	}

	virtual void OnLeftButtonDown();

//    virtual void OnMiddleButtonDown()
//    {
//      std::cout << "Pressed middle mouse button." << std::endl;
//      // Forward events
//      vtkInteractorStyleTrackballCamera::OnMiddleButtonDown();
//    }
//
//    virtual void OnRightButtonDown()
//    {
//      std::cout << "Pressed right mouse button." << std::endl;
//      // Forward events
//      vtkInteractorStyleTrackballCamera::OnRightButtonDown();
//    }

private:
	int num_locations;
	int left_click_;
	std::vector<int> location;
	std::vector<std::string> label;
	std::vector<std::string> label_ref;
	std::vector<std::vector<unsigned char> > color_;

	void writeText(
			const char* text,
			double *rgb,
			int x,
			int y);
};

void customMouseInteractorStyle::OnLeftButtonDown()
{
	switch (left_click_)
	{
		case CLICK_LABEL:
		{
			int* clickPos = this->GetInteractor()->GetEventPosition();

			// Pick from this location.
			vtkSmartPointer<vtkPointPicker> picker = vtkSmartPointer<
					vtkPointPicker>::New();
			picker->Pick(clickPos[0], clickPos[1], 0,
					this->GetDefaultRenderer());

			double rgb[3];
			if (picker->GetActor() != 0)
			{
				double* pos = picker->GetPickPosition();
				picker->GetActor()->GetMapper()->GetInput()->GetPointData()->GetScalars()->GetTuple(
						picker->GetPointId(), rgb);
			}

			for (int i = 0; i < num_locations; i++)
			{
				if (rgb[0] == color_[i + 1][0] && rgb[1] == color_[i + 1][1]
						&& rgb[2] == color_[i + 1][2])
				{
					for (int ii = 0; ii < label_ref.size(); ii++)
					{
						printf(">>>>> %d %s\n", ii, label_ref[ii].c_str());
					}
					printf(
							">>>>> Select label from list by entering the number : ");
					std::string mystr;
					getline(cin, mystr);
					if (atoi(mystr.c_str()) < 0
							|| atoi(mystr.c_str()) > label_ref.size() - 1)
					{
						printf(
								VTKCYEL ">>>>> [WARNING] : Please choose only from the listed numbers.\n" VTKCNOR);
						printf(">>>>> Pick a location with color.");
						break;
					}
					else
					{
						printf(">>>>> %s selected.\n",
								label_ref[atoi(mystr.c_str())].c_str());
					}

					if (!label[i].empty())
					{
						printf(
								VTKCYEL ">>>>> [WARNING] : Label has been given. Do you wish to overwrite? [Y/N]\n" VTKCNOR);
						printf(">>>>> ");
						while (1)
						{
							std::string mystr2;
							getline(cin, mystr2);
							if (!strcmp(mystr2.c_str(), "Y"))
							{
								printf(
										VTKCYEL ">>>>> [WARNING] : Label has been overwritten. New label : %s\n" VTKCNOR,
										label_ref[atoi(mystr.c_str())].c_str());
								label[i] = label_ref[atoi(mystr.c_str())];
								writeText(
										label_ref[atoi(mystr.c_str())].c_str(),
										rgb, 10, 470 - 10 * (i + 1));
								break;
							}
							if (!strcmp(mystr2.c_str(), "N"))
							{
								printf(
										VTKCYEL ">>>>> [WARNING] : Label has not been overwritten.\n" VTKCNOR);
								break;
							}
						}
					}
					else
					{
						label[i] = label_ref[atoi(mystr.c_str())];
						writeText(label_ref[atoi(mystr.c_str())].c_str(), rgb,
								10, 470 - 10 * (i + 1));
						break;
					}
				}
			}

			bool flag = true;
			for (int i = 0; i < num_locations; i++)
			{
				// when there is no labeling at all
				if (label.empty())
					break;
				// check for completeness of labelling
				if (label[i].empty())
					break;
				// else
				if (i == num_locations - 1)
				{
					flag = false;
					printf(
							VTKCYEL ">>>>> [WARNING] : Label has been fully labeled. Proceed? [Y/N]\n" VTKCNOR);
					printf(">>>>> ");
					std::string mystr3;
					getline(cin, mystr3);
					if (!strcmp(mystr3.c_str(), "Y"))
					{
						this->GetInteractor()->TerminateApp();
					}
					else
					{
						printf(">>>>> Pick a location with color.\n");
					}
				}
			}
			if (flag)
			{
				printf(">>>>> Pick a location with color.\n");
			}

			// Forward events, camera manipulation
			vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
			break;
		}
		case CLICK_DELETE:
		{
			//std::cout << "Pressed left mouse button." << std::endl;
			int* clickPos = this->GetInteractor()->GetEventPosition();

			// Pick from this location.
			vtkSmartPointer<vtkPointPicker> picker = vtkSmartPointer<
					vtkPointPicker>::New();
			picker->Pick(clickPos[0], clickPos[1], 0,
					this->GetDefaultRenderer());

			double rgb[3];
			if (picker->GetActor() != 0)
			{
				picker->GetActor()->GetMapper()->GetInput()->GetPointData()->GetScalars()->GetTuple(
						picker->GetPointId(), rgb);
			}

			for (int i = 0; i < num_locations; i++)
			{
				if (rgb[0] == color_[i + 1][0] && rgb[1] == color_[i + 1][1]
						&& rgb[2] == color_[i + 1][2])
				{
					printf(">>>>> Delete label? [Y/N]\n");
					printf(">>>>> ");
					std::string mystr;
					getline(cin, mystr);
					while (1)
					{
						if (!strcmp(mystr.c_str(), "Y"))
						{
							printf(
									">>>>> [WARNING] : Label has not been deleted.\n");
							printf(
									">>>>> Press q to quit or click another location with color to delete.\n");
							location[i] = -1;
							break;
						}
						if (!strcmp(mystr.c_str(), "N"))
						{
							printf(
									">>>>> [WARNING] : Label has not been deleted.\n");
							printf(
									">>>>> Press q to quit or click another location with color to delete.\n");
							break;
						}
					}
				}
			}
			vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
			break;
		}
		default:
			vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
			break;
	}
}

void customMouseInteractorStyle::writeText(
		const char* text,
		double *rgb,
		int x,
		int y)
{
	// Setup the text and add it to the renderer
	vtkSmartPointer<vtkTextActor> textActor;
	textActor = vtkSmartPointer<vtkTextActor>::New();
	textActor->SetInput(text);
	textActor->SetPosition(x, y);
	textActor->GetTextProperty()->SetFontSize(10);
	textActor->GetTextProperty()->SetColor(rgb[0] / 255, rgb[1] / 255,
			rgb[2] / 255);
	this->GetDefaultRenderer()->AddActor2D(textActor);
	this->GetInteractor()->Render();
}

vtkStandardNewMacro(customMouseInteractorStyle);

/*******************************************************************************
 * ColorCode
 ******************************************************************************/

void VTKExtra::ColorCode(
		std::vector<std::vector<unsigned char> > &container_)
{
	int N = 1;
	container_.clear();
	container_.resize(N * 12);
	// Setup colors
	unsigned char cw[] =
	{ 255, 255, 255 };
	unsigned char cy[] =
	{ 255, 255, 0 };
	unsigned char co[] =
	{ 255, 127, 0 };
	unsigned char cr[] =
	{ 255, 0, 0 };
	unsigned char clg[] =
	{ 127, 255, 0 };
	unsigned char cg[] =
	{ 0, 255, 0 };
	unsigned char cgb[] =
	{ 0, 255, 127 };
	unsigned char cc[] =
	{ 0, 255, 255 };
	unsigned char clb[] =
	{ 0, 127, 255 };
	unsigned char cb[] =
	{ 0, 0, 255 };
	unsigned char cpb[] =
	{ 127, 0, 255 };
	unsigned char cpr[] =
	{ 255, 0, 127 };
	for (int i = 0; i < N; i++)
	{
		this->arrayTovector(cw, 3, container_[12 * i + 0]);
		this->arrayTovector(cy, 3, container_[12 * i + 1]);
		this->arrayTovector(co, 3, container_[12 * i + 2]);
		this->arrayTovector(cr, 3, container_[12 * i + 3]);
		this->arrayTovector(cc, 3, container_[12 * i + 4]);
		this->arrayTovector(cpb, 3, container_[12 * i + 5]);
		this->arrayTovector(cgb, 3, container_[12 * i + 6]);
		this->arrayTovector(clb, 3, container_[12 * i + 7]);
		this->arrayTovector(cb, 3, container_[12 * i + 8]);
		this->arrayTovector(cg, 3, container_[12 * i + 9]);
		this->arrayTovector(clg, 3, container_[12 * i + 10]);
		this->arrayTovector(cpr, 3, container_[12 * i + 11]);
	}
}

/*******************************************************************************
 * this->DataPoints
 ******************************************************************************/

vtkSmartPointer<vtkPolyDataMapper> VTKExtra::DataPoints(
		std::vector<Eigen::Vector4d> points_,
		int num_locations_,
		std::vector<std::vector<unsigned char> > color_,
		bool cluster_)
{
	int num_locations = num_locations_;

	// Create the geometry of a point (the coordinate)
	// add point to polydata to create the vertices for glyph
	// creating the vertices with a small cube glyph
	// add points and vertices to polydata
	// giving the points color
	// Create a mapper and actor
	// Create a renderer, render window, and interactor
	// custom mouse
	// Setup the text and add it to the renderer
	vtkSmartPointer<vtkPoints> points;
	vtkSmartPointer<vtkPolyData> pointsPolydata;
	vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter;
	vtkSmartPointer<vtkPolyData> polydata;
	vtkSmartPointer<vtkUnsignedCharArray> colors;
	vtkSmartPointer<vtkPolyDataMapper> mapper;
//	vtkSmartPointer<vtkActor> 					actor;
//	vtkSmartPointer<vtkRenderer> 				renderer;
//	vtkSmartPointer<vtkRenderWindow> 			renWin;
//	vtkSmartPointer<vtkRenderWindowInteractor> 	renWinInter;
//	vtkSmartPointer<customMouseInteractorStyle> style;
//	vtkSmartPointer<vtkTextActor> 				textActor;

	points = vtkSmartPointer<vtkPoints>::New();
	pointsPolydata = vtkSmartPointer<vtkPolyData>::New();
	vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	polydata = vtkSmartPointer<vtkPolyData>::New();
	colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

	for (int i = 0; i < points_.size(); i++)
		points->InsertNextPoint((double) points_[i](0), (double) points_[i](1),
				(double) points_[i](2));

	pointsPolydata->SetPoints(points);

#if VTK_MAJOR_VERSION <= 5
	vertexFilter->SetInputConnection(pointsPolydata->GetProducerPort());
	vertexFilter->Update();
#else
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();
#endif

	polydata->ShallowCopy(vertexFilter->GetOutput());

	if (cluster_)
	{
		colors->SetNumberOfComponents(3);
		colors->SetName("Colors");
		std::vector<unsigned char*> color_tmp;
		color_tmp.resize(color_.size());
		for (int i = 0; i < color_.size(); i++)
		{
			color_tmp[i] = new unsigned char[3];
			vectorToArray(color_[i], color_tmp[i]);
		}
		for (int i = 0; i < points_.size(); i++)
		{
			if (points_[i][3] < num_locations && points_[i][3] >= 0)
			{
				colors->InsertNextTypedTuple(
						(unsigned char*) color_tmp[points_[i][3] + 1]);
			}
			else
			{
				colors->InsertNextTypedTuple(color_tmp[0]);
			}
		}
		polydata->GetPointData()->SetScalars(colors);
	}

#if VTK_MAJOR_VERSION <= 5
	mapper->SetInput(polydata);
#else
	mapper->SetInputData(polydata);
#endif

	return mapper;
}

void VTKExtra::ShowData(
		std::vector<Eigen::Vector4d> points_,
		std::vector<std::string> &labels_,
		std::vector<std::string> labels_ref_,
		std::vector<int> &loc_idx_,
		std::vector<std::vector<unsigned char> > color_,
		bool cluster_,
		bool labeling_,
		bool deleting_)
{
	int num_locations = labels_.size();

	vtkSmartPointer<vtkActor> actor;
	vtkSmartPointer<vtkRenderer> renderer;
	vtkSmartPointer<vtkRenderWindow> renWin;
	vtkSmartPointer<vtkRenderWindowInteractor> renWinInter;
	vtkSmartPointer<customMouseInteractorStyle> style;
	vtkSmartPointer<vtkTextActor> textActor;
	vtkSmartPointer<vtkCamera> camera;

	actor = vtkSmartPointer<vtkActor>::New();
	renderer = vtkSmartPointer<vtkRenderer>::New();
	renWin = vtkSmartPointer<vtkRenderWindow>::New();
	renWinInter = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	style = vtkSmartPointer<customMouseInteractorStyle>::New();
//	textActor 		= vtkSmartPointer<vtkTextActor>::New();
	camera = vtkSmartPointer<vtkCamera>::New();

	actor->SetMapper(
			this->DataPoints(points_, num_locations, color_, cluster_));
	actor->GetProperty()->SetPointSize(3);
//	actor->GetProperty()->SetColor(1.0, 0.0, 0.0);

	renWin->PointSmoothingOn();
	renWin->SetSize(WIN_WIDTH, WIN_HEIGHT); //(width, height)
	renWin->AddRenderer(renderer);
	renWinInter->SetRenderWindow(renWin);

	if (labeling_)
	{
		style->SetLeftButton(CLICK_LABEL);
		textActor = vtkSmartPointer<vtkTextActor>::New();
		printf(">>>>> Pick a location with color.\n");
		if (!labels_.empty())
			textActor->SetInput(labels_[0].c_str());
		textActor->SetPosition(10, (WIN_HEIGHT - FONT_SIZE - 10));
		textActor->GetTextProperty()->SetFontSize( FONT_SIZE);
		textActor->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
		renderer->AddActor2D(textActor);
	}
	else
	{
		for (int i = 0; i < num_locations; i++)
		{
			if (labels_.empty())
				continue;
			if (labels_[i].empty())
				continue;
			textActor = vtkSmartPointer<vtkTextActor>::New();
			textActor->SetInput(labels_[i].c_str());
			textActor->SetPosition(10,
					(WIN_HEIGHT - FONT_SIZE - 10) - i * FONT_SIZE);
			textActor->GetTextProperty()->SetFontSize(FONT_SIZE);
			textActor->GetTextProperty()->SetColor(
					(double) color_[i + 1][0] / 255,
					(double) color_[i + 1][1] / 255,
					(double) color_[i + 1][2] / 255);
			renderer->AddActor2D(textActor);
		}

		if (deleting_)
		{
			printf(">>>>> Pick a location with color to delete.\n");
			style->SetLeftButton(CLICK_DELETE);
		}
		else
		{
			style->SetLeftButton(CLICK_EMPTY);
		}
	}

	renderer->AddActor(actor);
	//renderer->SetBackground(nc->GetColor3d("MidnightBlue").GetData());
	style->SetDefaultRenderer(renderer);
	style->SetNumberOfLabels(num_locations);
	style->SetLabels(labels_);
	style->SetRefLabels(labels_ref_);
	style->SetLocations(loc_idx_);
	style->SetColors(color_);
	renWinInter->SetInteractorStyle(style);

#ifdef SAVING_IMAGE
	vtkSmartPointer<KeyPressInteractorStyle> style2
	= vtkSmartPointer<KeyPressInteractorStyle>::New();
	style2->SetWin(renWin);
	style2->SetCurrentRenderer(renderer);
	renWinInter->SetInteractorStyle( style2 );
#endif

	renWin->Render();
	renWinInter->Start();

	labels_ = style->GetLabels();
	loc_idx_ = style->GetLocations();
}

/*******************************************************************************
 * Plots
 ******************************************************************************/

void VTKExtra::ShowConnection(
		CGraph *Graph_,
		std::vector<Eigen::Vector4d> points_,
		std::vector<std::string> &labels_,
		std::vector<std::vector<unsigned char> > color_,
		bool Show_points)
{
	// [VARIABLES]*************************************************************
	std::vector<CGraph::node_t> nodes = Graph_->GetNodeList();
	std::vector<std::vector<std::vector<CGraph::edge_t> > > edges =
			Graph_->GetListOfEdges();

	int num_locations = nodes.size();

	std::vector<Eigen::Vector3d> locations;
	locations.resize(num_locations);
	for (int i = 0; i < num_locations; i++)
	{
		locations[i] = V4d3d(nodes[i].centroid);
	}

	vtkSmartPointer<vtkLineSource> lineSource;
	vtkSmartPointer<vtkPolyDataMapper> lineMapper;
	vtkSmartPointer<vtkActor> lineActor;
	vtkSmartPointer<vtkPoints> points;
	vtkSmartPointer<vtkCellArray> lines;
	vtkSmartPointer<vtkPolyData> polyData;
	vtkSmartPointer<vtkDoubleArray> tubeRadius;
	vtkSmartPointer<vtkTubeFilter> tubeFilter;
	vtkSmartPointer<vtkPolyDataMapper> tubeMapper;
	vtkSmartPointer<vtkActor> tubeActor;
	vtkSmartPointer<vtkUnsignedCharArray> colors;
	vtkSmartPointer<vtkRenderer> renderer;
	vtkSmartPointer<vtkRenderWindow> renWin;
	vtkSmartPointer<vtkRenderWindowInteractor> renWinInter;
	vtkSmartPointer<customMouseInteractorStyle> style;
	// *************************************************************[VARIABLES]

	style = vtkSmartPointer<customMouseInteractorStyle>::New();
	renderer = vtkSmartPointer<vtkRenderer>::New();
	renWin = vtkSmartPointer<vtkRenderWindow>::New();
	renWinInter = vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renWin->SetSize(1280, 800); //(width, height)
	renWin->AddRenderer(renderer);
	renWinInter->SetRenderWindow(renWin);

	vtkSmartPointer<vtkPoints> pointspoly;
	vtkSmartPointer<vtkPolygon> polygon;
	vtkSmartPointer<vtkCellArray> polygons;
	vtkSmartPointer<vtkPolyData> polygonPolyData;
	vtkSmartPointer<vtkPolyDataMapper> mapperpoly;
	vtkSmartPointer<vtkActor> actorpoly;

	for (int i = 0; i < Sqr(num_locations); i++)
	{
		if (i / num_locations == i % num_locations)
			continue;
		if (edges[i / num_locations][i % num_locations][0].counter == 0)
			continue;

		// [SECTOR POLYGON]****************************************************
		pointspoly = vtkSmartPointer<vtkPoints>::New();
		polygons = vtkSmartPointer<vtkCellArray>::New();
		polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
		mapperpoly = vtkSmartPointer<vtkPolyDataMapper>::New();
		actorpoly = vtkSmartPointer<vtkActor>::New();

		int loc = LOC_INT;
		int sec = SEC_INT;
		int n1 = i / num_locations;
		int n2 = i % num_locations;

		Eigen::AngleAxisd aa;
		Eigen::Vector3d Ntmp[2], Nmax[2], init, beg, end;
		init = Eigen::Vector3d::Zero();
//		init = sector[i][0].loc_start[0];

		vtkIdType ID[4];

		for (int l = 0; l < loc; l++)
		{
			aa.angle() = (double) (2 * M_PI * (0) / sec);
			aa.axis() = edges[n1][n2][0].tan[l];
			Ntmp[0] = rodriguezVec(aa, edges[n1][n2][0].nor[l]);
			Nmax[0] = Ntmp[0] * edges[n1][n2][0].sector_map[l * sec + 0];

			beg = V4d3d(
					edges[n1][n2][0].loc_mid[l]
							* (double) edges[n1][n2][0].loc_mid[l][3])
					- (edges[n1][n2][0].tan[l]
							* (edges[n1][n2][0].loc_len[l] / 2))
					+ locations[n1];
			end = V4d3d(
					edges[n1][n2][0].loc_mid[l]
							* (double) edges[n1][n2][0].loc_mid[l][3])
					+ (edges[n1][n2][0].tan[l]
							* (edges[n1][n2][0].loc_len[l] / 2))
					+ locations[n1];
			for (int s = 0; s < sec; s++)
			{
				int s_tmp = (s + 1) % sec;
				aa.angle() = (double) (2 * M_PI * (s_tmp) / sec);
				Ntmp[s_tmp % 2] = rodriguezVec(aa, edges[n1][n2][0].nor[l]);
				Nmax[s_tmp % 2] = Ntmp[s_tmp % 2]
						* edges[n1][n2][0].sector_map[l * sec + s];
//				if(edges[n1][n2][0].sector_map[l*sec+s]>0)
//				{
//					cout << edges[n1][n2][0].nor[l] << endl;
//					cout << Ntmp[s_tmp%2] << endl;
//					cout << Nmax[s_tmp%2] << endl;
//					cout << edges[n1][n2][0].sector_map[l*sec+s] << endl;
//				}
				pointspoly->InsertPoint((l * sec + s) * 8 + 0,
						(double) (beg + Nmax[s % 2] - init)(0),
						(double) (beg + Nmax[s % 2] - init)(1),
						(double) (beg + Nmax[s % 2] - init)(2));
				pointspoly->InsertPoint((l * sec + s) * 8 + 1,
						(double) (end + Nmax[s % 2] - init)(0),
						(double) (end + Nmax[s % 2] - init)(1),
						(double) (end + Nmax[s % 2] - init)(2));
				pointspoly->InsertPoint((l * sec + s) * 8 + 2,
						(double) (beg + Nmax[s_tmp % 2] - init)(0),
						(double) (beg + Nmax[s_tmp % 2] - init)(1),
						(double) (beg + Nmax[s_tmp % 2] - init)(2));
				pointspoly->InsertPoint((l * sec + s) * 8 + 3,
						(double) (end + Nmax[s_tmp % 2] - init)(0),
						(double) (end + Nmax[s_tmp % 2] - init)(1),
						(double) (end + Nmax[s_tmp % 2] - init)(2));
				ID[0] = (l * sec + s) * 8 + 0;
				ID[1] = (l * sec + s) * 8 + 1;
				ID[2] = (l * sec + s) * 8 + 3;
				ID[3] = (l * sec + s) * 8 + 2;
				polygons->InsertNextCell(4, ID);

				pointspoly->InsertPoint((l * sec + s) * 8 + 4,
						(double) (beg + Nmax[s_tmp % 2] - init)(0),
						(double) (beg + Nmax[s_tmp % 2] - init)(1),
						(double) (beg + Nmax[s_tmp % 2] - init)(2));
				pointspoly->InsertPoint((l * sec + s) * 8 + 5,
						(double) (end + Nmax[s_tmp % 2] - init)(0),
						(double) (end + Nmax[s_tmp % 2] - init)(1),
						(double) (end + Nmax[s_tmp % 2] - init)(2));
				Nmax[s_tmp % 2] = Ntmp[s_tmp % 2]
						* edges[n1][n2][0].sector_map[l * sec + s_tmp];
				pointspoly->InsertPoint((l * sec + s) * 8 + 6,
						(double) (beg + Nmax[s_tmp % 2] - init)(0),
						(double) (beg + Nmax[s_tmp % 2] - init)(1),
						(double) (beg + Nmax[s_tmp % 2] - init)(2));
				pointspoly->InsertPoint((l * sec + s) * 8 + 7,
						(double) (end + Nmax[s_tmp % 2] - init)(0),
						(double) (end + Nmax[s_tmp % 2] - init)(1),
						(double) (end + Nmax[s_tmp % 2] - init)(2));
				ID[0] = (l * sec + s) * 8 + 4;
				ID[1] = (l * sec + s) * 8 + 5;
				ID[2] = (l * sec + s) * 8 + 7;
				ID[3] = (l * sec + s) * 8 + 6;
				polygons->InsertNextCell(4, ID);
			}
		}

		polygonPolyData->SetPoints(pointspoly);
		polygonPolyData->SetPolys(polygons);

#if VTK_MAJOR_VERSION <= 5
		mapperpoly->SetInput(polygonPolyData);
#else
		mapperpoly->SetInputData(polygonPolyData);
#endif

		actorpoly->SetMapper(mapperpoly);
		actorpoly->GetProperty()->SetColor(0.0, 1.0, 0.0); //color
		renderer->AddActor(actorpoly);
		// ****************************************************[SECTOR POLYGON]
	}

	// [ADDING DATA]***********************************************************
	if (Show_points)
	{
		vtkSmartPointer<vtkTextActor> textActor;
		vtkSmartPointer<vtkActor> actor;
		actor = vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(
				this->DataPoints(points_, num_locations, color_, true));
		actor->GetProperty()->SetPointSize(5);
		renderer->AddActor(actor);
		style->SetNumberOfLabels(num_locations);
		style->SetLabels(labels_);
		style->SetColors(color_);
		for (int i = 0; i < num_locations; i++)
		{
			textActor = vtkSmartPointer<vtkTextActor>::New();
			if (labels_.empty())
				continue;
			if (labels_[i].empty())
				continue;
			textActor->SetInput(labels_[i].c_str());
			textActor->SetPosition(10,
					(WIN_HEIGHT - FONT_SIZE - 10) - i * FONT_SIZE);
			textActor->GetTextProperty()->SetFontSize(FONT_SIZE);
			textActor->GetTextProperty()->SetColor(
					(double) color_[i + 1][0] / 255,
					(double) color_[i + 1][1] / 255,
					(double) color_[i + 1][2] / 255);
			renderer->AddActor2D(textActor);
		}
	}
	// ***********************************************************[ADDING DATA]

	style->SetDefaultRenderer(renderer);
	style->SetLeftButton(CLICK_EMPTY);
	renderer->SetBackground(0.0, 0.0, 0.0);
	renWinInter->SetInteractorStyle(style);

#ifdef SAVING_IMAGE
	vtkSmartPointer<KeyPressInteractorStyle> style2
	= vtkSmartPointer<KeyPressInteractorStyle>::New();
	style2->SetWin(renWin);
	style2->SetWinInter(renWinInter);
	style2->SetCurrentRenderer(renderer);
	renWinInter->SetInteractorStyle( style2 );
#endif

	renWin->Render();
	renWinInter->Start();
}

void VTKExtra::ShowConnectionTest(
		CGraph *Graph_,
		std::vector<Eigen::Vector4d> points_,
		std::vector<std::string> &labels_,
		std::vector<std::vector<unsigned char> > color_,
		bool Show_points)
{
	// [VARIABLES]*************************************************************
	std::vector<CGraph::node_t> nodes = Graph_->GetNodeList();
	std::vector<std::vector<std::vector<CGraph::edge_t> > > edges =
			Graph_->GetListOfEdges();

	int num_locations = nodes.size();

	std::vector<Eigen::Vector3d> locations;
	locations.resize(num_locations);
	for (int i = 0; i < num_locations; i++)
	{
		locations[i] = V4d3d(nodes[i].centroid);
	}

	vtkSmartPointer<vtkSphereSource> sphereSource;
	vtkSmartPointer<vtkUnsignedCharArray> colors;
	vtkSmartPointer<vtkRenderer> renderer;
	vtkSmartPointer<vtkRenderWindow> renWin;
	vtkSmartPointer<vtkRenderWindowInteractor> renWinInter;
	vtkSmartPointer<customMouseInteractorStyle> style;
	vtkSmartPointer<vtkPolyDataMapper> mapper;
	vtkSmartPointer<vtkActor> actor;
	// *************************************************************[VARIABLES]

	style = vtkSmartPointer<customMouseInteractorStyle>::New();
	renderer = vtkSmartPointer<vtkRenderer>::New();
	renWin = vtkSmartPointer<vtkRenderWindow>::New();
	renWinInter = vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renWin->SetSize(1280, 800); //(width, height)
	renWin->AddRenderer(renderer);
	renWinInter->SetRenderWindow(renWin);

	vtkSmartPointer<vtkPoints> pointspoly;
	vtkSmartPointer<vtkPolygon> polygon;
	vtkSmartPointer<vtkCellArray> polygons;
	vtkSmartPointer<vtkPolyData> polygonPolyData;
	vtkSmartPointer<vtkPolyDataMapper> mapperpoly;
	vtkSmartPointer<vtkActor> actorpoly;

	for (int i = 0; i < Sqr(num_locations); i++)
	{
		if (i / num_locations == i % num_locations)
			continue;
		if (edges[i / num_locations][i % num_locations][0].counter == 0)
			continue;

		// [SECTOR POLYGON]****************************************************
		pointspoly = vtkSmartPointer<vtkPoints>::New();
		polygons = vtkSmartPointer<vtkCellArray>::New();
		polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
		mapperpoly = vtkSmartPointer<vtkPolyDataMapper>::New();
		actorpoly = vtkSmartPointer<vtkActor>::New();

		int loc = LOC_INT;
		int sec = SEC_INT;
		int n1 = i / num_locations;
		int n2 = i % num_locations;

		Eigen::AngleAxisd aa;
		Eigen::Vector3d Ntmp[2], Nmax[2], init, beg, end;
		init = Eigen::Vector3d::Zero();
//		init = sector[i][0].loc_start[0];

		vtkIdType ID[4];

		for (int l = 0; l < loc; l++)
		{
			aa.angle() = (double) (2 * M_PI * (0) / sec);
			aa.axis() = edges[n1][n2][0].tan[l];
			Ntmp[0] = rodriguezVec(aa, edges[n1][n2][0].nor[l]);
			Nmax[0] = Ntmp[0] * edges[n1][n2][0].sector_map[l * sec + 0];

			beg = V4d3d(
					edges[n1][n2][0].loc_mid[l]
							* (double) edges[n1][n2][0].loc_mid[l][3])
					- (edges[n1][n2][0].tan[l]
							* (edges[n1][n2][0].loc_len[l] / 2))
					+ locations[n1];
			end = V4d3d(
					edges[n1][n2][0].loc_mid[l]
							* (double) edges[n1][n2][0].loc_mid[l][3])
					+ (edges[n1][n2][0].tan[l]
							* (edges[n1][n2][0].loc_len[l] / 2))
					+ locations[n1];
			for (int s = 0; s < sec; s++)
			{
				int s_tmp = (s + 1) % sec;
				aa.angle() = (double) (2 * M_PI * (s_tmp) / sec);
				aa.axis() = edges[n1][n2][0].tan[l];
				Ntmp[s_tmp % 2] = rodriguezVec(aa, edges[n1][n2][0].nor[l]);
				Nmax[s_tmp % 2] = Ntmp[s_tmp % 2]
						* edges[n1][n2][0].sector_map[l * sec + s];
				pointspoly->InsertPoint((l * sec + s) * 8 + 0,
						(double) (beg + Nmax[s % 2] - init)(0),
						(double) (beg + Nmax[s % 2] - init)(1),
						(double) (beg + Nmax[s % 2] - init)(2));
				pointspoly->InsertPoint((l * sec + s) * 8 + 1,
						(double) (end + Nmax[s % 2] - init)(0),
						(double) (end + Nmax[s % 2] - init)(1),
						(double) (end + Nmax[s % 2] - init)(2));
				pointspoly->InsertPoint((l * sec + s) * 8 + 2,
						(double) (beg + Nmax[s_tmp % 2] - init)(0),
						(double) (beg + Nmax[s_tmp % 2] - init)(1),
						(double) (beg + Nmax[s_tmp % 2] - init)(2));
				pointspoly->InsertPoint((l * sec + s) * 8 + 3,
						(double) (end + Nmax[s_tmp % 2] - init)(0),
						(double) (end + Nmax[s_tmp % 2] - init)(1),
						(double) (end + Nmax[s_tmp % 2] - init)(2));
				ID[0] = (l * sec + s) * 8 + 0;
				ID[1] = (l * sec + s) * 8 + 1;
				ID[2] = (l * sec + s) * 8 + 3;
				ID[3] = (l * sec + s) * 8 + 2;
				polygons->InsertNextCell(4, ID);

				pointspoly->InsertPoint((l * sec + s) * 8 + 4,
						(double) (beg + Nmax[s_tmp % 2] - init)(0),
						(double) (beg + Nmax[s_tmp % 2] - init)(1),
						(double) (beg + Nmax[s_tmp % 2] - init)(2));
				pointspoly->InsertPoint((l * sec + s) * 8 + 5,
						(double) (end + Nmax[s_tmp % 2] - init)(0),
						(double) (end + Nmax[s_tmp % 2] - init)(1),
						(double) (end + Nmax[s_tmp % 2] - init)(2));
				Nmax[s_tmp % 2] = Ntmp[s_tmp % 2]
						* edges[n1][n2][0].sector_map[l * sec + s_tmp];
				pointspoly->InsertPoint((l * sec + s) * 8 + 6,
						(double) (beg + Nmax[s_tmp % 2] - init)(0),
						(double) (beg + Nmax[s_tmp % 2] - init)(1),
						(double) (beg + Nmax[s_tmp % 2] - init)(2));
				pointspoly->InsertPoint((l * sec + s) * 8 + 7,
						(double) (end + Nmax[s_tmp % 2] - init)(0),
						(double) (end + Nmax[s_tmp % 2] - init)(1),
						(double) (end + Nmax[s_tmp % 2] - init)(2));
				ID[0] = (l * sec + s) * 8 + 4;
				ID[1] = (l * sec + s) * 8 + 5;
				ID[2] = (l * sec + s) * 8 + 7;
				ID[3] = (l * sec + s) * 8 + 6;
				polygons->InsertNextCell(4, ID);
			}
		}

		polygonPolyData->SetPoints(pointspoly);
		polygonPolyData->SetPolys(polygons);

#if VTK_MAJOR_VERSION <= 5
		mapperpoly->SetInput(polygonPolyData);
#else
		mapperpoly->SetInputData(polygonPolyData);
#endif

		actorpoly->SetMapper(mapperpoly);
		actorpoly->GetProperty()->SetColor(0.0, 1.0, 0.0);
		renderer->AddActor(actorpoly);
		// ****************************************************[SECTOR POLYGON]
	}

	// [ADDING DATA]***********************************************************
	if (Show_points)
	{
		vtkSmartPointer<vtkTextActor> textActor;
		vtkSmartPointer<vtkActor> actor;
		actor = vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(
				this->DataPoints(points_, num_locations, color_, true));
		actor->GetProperty()->SetPointSize(5);
		renderer->AddActor(actor);
		style->SetNumberOfLabels(num_locations);
		style->SetLabels(labels_);
		style->SetColors(color_);
		for (int i = 0; i < num_locations; i++)
		{
			textActor = vtkSmartPointer<vtkTextActor>::New();
			if (labels_.empty())
				continue;
			if (labels_[i].empty())
				continue;
			textActor->SetInput(labels_[i].c_str());
			textActor->SetPosition(10,
					(WIN_HEIGHT - FONT_SIZE - 10) - i * FONT_SIZE);
			textActor->GetTextProperty()->SetFontSize(FONT_SIZE);
			textActor->GetTextProperty()->SetColor(
					(double) color_[i + 1][0] / 255,
					(double) color_[i + 1][1] / 255,
					(double) color_[i + 1][2] / 255);
			renderer->AddActor2D(textActor);
		}
	}
	// ***********************************************************[ADDING DATA]

	for (int i = 0; i < nodes.size(); i++)
	{

		sphereSource = vtkSmartPointer<vtkSphereSource>::New();
		mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		actor = vtkSmartPointer<vtkActor>::New();

		sphereSource->SetCenter((double) nodes[i].centroid(0),
				(double) nodes[i].centroid(1), (double) nodes[i].centroid(2));
		sphereSource->SetRadius(0.05);
		mapper->SetInputConnection(sphereSource->GetOutputPort());
		actor->SetMapper(mapper);
		actor->GetProperty()->SetColor((double) color_[i + 1][0] / 255,
				(double) color_[i + 1][1] / 255,
				(double) color_[i + 1][2] / 255);
		renderer->AddActor(actor);
	}

	style->SetDefaultRenderer(renderer);
	style->SetLeftButton(CLICK_EMPTY);
	renderer->SetBackground(0.0, 0.0, 0.0);
	renWinInter->SetInteractorStyle(style);

#ifdef SAVING_IMAGE
	vtkSmartPointer<KeyPressInteractorStyle> style2
	= vtkSmartPointer<KeyPressInteractorStyle>::New();
	style2->SetWin(renWin);
	style2->SetCurrentRenderer(renderer);
	renWinInter->SetInteractorStyle( style2 );
#endif

	renWin->Render();
	renWinInter->Start();
}

void VTKExtra::PlotData(
		std::vector<double> x,
		std::vector<double> y)
{
	vtkSmartPointer<vtkTable> table;
	vtkSmartPointer<vtkFloatArray> arrX;
	vtkSmartPointer<vtkFloatArray> arrY;
	vtkSmartPointer<vtkContextView> view;
	vtkSmartPointer<vtkChartXY> chart;

	table = vtkSmartPointer<vtkTable>::New();
	arrX = vtkSmartPointer<vtkFloatArray>::New();
	arrY = vtkSmartPointer<vtkFloatArray>::New();
	view = vtkSmartPointer<vtkContextView>::New();
	chart = vtkSmartPointer<vtkChartXY>::New();

	// Create a table with some points in it
	arrX->SetName("X Axis");
	arrY->SetName("Y Axis");
	table->AddColumn(arrX);
	table->AddColumn(arrY);

	// Fill in the table with values
	int numPoints = x.size();
	table->SetNumberOfRows(numPoints);
	for (int i = 0; i < numPoints; ++i)
	{
		table->SetValue(i, 0, x[i]);
		table->SetValue(i, 1, y[i]);
	}

	// Set up the view
	view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
	view->GetRenderer()->GetRenderWindow()->SetSize(1280, 800); //(width, height)

	// Add multiple line Plots, setting the colors etc
	view->GetScene()->AddItem(chart);

	vtkPlot *line;
	line = chart->AddPlot(vtkChart::LINE);
#if VTK_MAJOR_VERSION <= 5
	line->SetInput(table, 0, 1);
#else
	line->SetInputData(table, 0, 1);
#endif
	line->SetColor(255, 0, 0, 255);
	line->SetWidth(2.0);

//  // For dotted line, the line type can be from 2 to 5 for different dash/dot
//  // patterns (see enum in vtkPen containing DASH_LINE, value 2):
//#ifndef WIN32
//  line->GetPen()->SetLineType(vtkPen::DASH_LINE);
//#endif
//  // (ifdef-ed out on Windows because DASH_LINE does not work on Windows
//  //  machines with built-in Intel HD graphics card...)
//
//  //view->GetRenderWindow()->SetMultiSamples(0);

	// Start interactor
	view->GetInteractor()->Initialize();
	view->GetInteractor()->Start();
}

void VTKExtra::PlotData(
		std::vector<double> x,
		std::vector<double> y,
		std::vector<double> x2,
		std::vector<double> y2)
{
	vtkSmartPointer<vtkTable> table;
	vtkSmartPointer<vtkFloatArray> arrX;
	vtkSmartPointer<vtkFloatArray> arrY;
	vtkSmartPointer<vtkFloatArray> arrY2;
	vtkSmartPointer<vtkContextView> view;
	vtkSmartPointer<vtkChartXY> chart;

	table = vtkSmartPointer<vtkTable>::New();
	arrX = vtkSmartPointer<vtkFloatArray>::New();
	arrY = vtkSmartPointer<vtkFloatArray>::New();
	arrY2 = vtkSmartPointer<vtkFloatArray>::New();
	view = vtkSmartPointer<vtkContextView>::New();
	chart = vtkSmartPointer<vtkChartXY>::New();

	// Create a table with some points in it
	arrX->SetName("X Axis");
	arrY->SetName("Y Axis");
	arrY2->SetName("Y2 Axis");
	table->AddColumn(arrX);
	table->AddColumn(arrY);
	table->AddColumn(arrY2);

	// Fill in the table with values
	int numPoints = x.size();
	table->SetNumberOfRows(numPoints);
	for (int i = 0; i < numPoints; ++i)
	{
		table->SetValue(i, 0, x[i]);
		table->SetValue(i, 1, y[i]);
		table->SetValue(i, 2, y2[i]);
	}

	// Set up the view
	view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
	view->GetRenderer()->GetRenderWindow()->SetSize(1280, 800); //(width, height)

	// Add multiple line Plots, setting the colors etc
	view->GetScene()->AddItem(chart);

	vtkPlot *line;
	line = chart->AddPlot(vtkChart::LINE);
#if VTK_MAJOR_VERSION <= 5
	line->SetInput(table, 0, 1);
#else
	line->SetInputData(table, 0, 1);
#endif
	line->SetColor(255, 0, 0, 255);
	line->SetWidth(2.0);
	line = chart->AddPlot(vtkChart::LINE);
#if VTK_MAJOR_VERSION <= 5
	line->SetInput(table, 0, 2);
#else
	line->SetInputData(table, 0, 2);
#endif
	line->SetColor(0, 255, 0, 255);
	line->SetWidth(12.0);

//  // For dotted line, the line type can be from 2 to 5 for different dash/dot
//  // patterns (see enum in vtkPen containing DASH_LINE, value 2):
//#ifndef WIN32
//  line->GetPen()->SetLineType(vtkPen::DASH_LINE);
//#endif
//  // (ifdef-ed out on Windows because DASH_LINE does not work on Windows
//  //  machines with built-in Intel HD graphics card...)
//
//  //view->GetRenderWindow()->SetMultiSamples(0);

	// Start interactor
	view->GetInteractor()->Initialize();
	view->GetInteractor()->Start();
}

void VTKExtra::PlotDatas(
		std::vector<std::string> title,
		std::vector<double> x,
		std::vector<std::vector<std::vector<double> > > y)
{
	vtkSmartPointer<vtkTable> table;
	vtkSmartPointer<vtkFloatArray> arrX;
	vtkSmartPointer<vtkFloatArray> arrY;
	vtkSmartPointer<vtkContextView> view;
	vtkSmartPointer<vtkChartXY> chart;

	view = vtkSmartPointer<vtkContextView>::New();

	// Set up the view
	view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
	view->GetRenderer()->GetRenderWindow()->SetSize(1600, 800); //(width, height)

	std::vector<std::vector<unsigned char> > cc;
	this->ColorCode(cc);

	for (int i = 0; i < title.size(); i++)
	{
		vtkRectf tmp((i % 4) * 400, (((title.size() - 1) / 4) - (i / 4)) * 400,
				400.0, 400.0);

		chart = vtkSmartPointer<vtkChartXY>::New();
		chart->SetTitle(title[i]);
		chart->SetSize(tmp);
//		chart->SetAutoSize(false);

		// Create a table with some points in it
		arrX = vtkSmartPointer<vtkFloatArray>::New();
		arrX->SetName("X Axis");
		table = vtkSmartPointer<vtkTable>::New();
		table->AddColumn(arrX);

		for (int ii = 0; ii < y[i].size(); ii++)
		{
			arrY = vtkSmartPointer<vtkFloatArray>::New();
			arrY->SetName(
					std::string("Y" + std::to_string(ii + 1) + " Axis").c_str()); //std::to_std::string is c++11
			table->AddColumn(arrY);
		}

		table->SetNumberOfRows(x.size());

		for (int ii = 0; ii < x.size(); ii++)
		{
			table->SetValue(ii, 0, x[ii]);
		}

		for (int ii = 0; ii < y[i].size(); ii++)
		{
			// Fill in the table with values
			for (int iii = 0; iii < y[i][ii].size(); ++iii)
			{
				table->SetValue(iii, ii + 1, y[i][ii][iii]);
			}
		}

		// Add multiple line Plots, setting the colors etc
		view->GetScene()->AddItem(chart);

		vtkPlot *line;
		for (int ii = 0; ii < y[i].size(); ii++)
		{
			line = chart->AddPlot(vtkChart::LINE);
#if VTK_MAJOR_VERSION <= 5
			line->SetInput(table, 0, ii+1);
#else
			line->SetInputData(table, 0, ii + 1);
#endif
			line->SetColor(cc[ii + 1][0], cc[ii + 1][1], cc[ii + 1][2], 255);
			line->SetWidth(2.0);
		}

		// 0 for y, 1 for x
		chart->GetAxis(0)->SetBehavior(vtkAxis::FIXED);
		chart->GetAxis(0)->SetMinimum(0.0);
		chart->GetAxis(0)->SetMaximum(1.0);
	}

	// Start interactor
	view->GetInteractor()->Initialize();
	view->GetInteractor()->Start();
}

void VTKExtra::PlotDatasGeo(
		std::vector<std::string> title,
		std::vector<double> x,
		std::vector<std::vector<std::vector<double> > > y)
{
	vtkSmartPointer<vtkTable> table;
	vtkSmartPointer<vtkFloatArray> arrX;
	vtkSmartPointer<vtkFloatArray> arrY;
	vtkSmartPointer<vtkContextView> view;
	vtkSmartPointer<vtkChartXY> chart;

	view = vtkSmartPointer<vtkContextView>::New();

	// Set up the view
	view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
	view->GetRenderer()->GetRenderWindow()->SetSize(1600, 800); //(width, height)

	std::vector<std::vector<unsigned char> > cc;
	this->ColorCode(cc);

	for (int i = 0; i < title.size(); i++)
	{
		vtkRectf tmp((i % 4) * 400, (((title.size() - 1) / 4) - (i / 4)) * 400,
				400.0, 400.0);

		chart = vtkSmartPointer<vtkChartXY>::New();
		chart->SetTitle(title[i]);
		chart->SetSize(tmp);
		chart->SetAutoSize(false);

		// Create a table with some points in it
		arrX = vtkSmartPointer<vtkFloatArray>::New();
		arrX->SetName("X Axis");
		table = vtkSmartPointer<vtkTable>::New();
		table->AddColumn(arrX);

		for (int ii = 0; ii < y[i].size(); ii++)
		{
			if (ii != i)
				continue;
			arrY = vtkSmartPointer<vtkFloatArray>::New();
			arrY->SetName(
					std::string("Y" + std::to_string(ii + 1) + " Axis").c_str()); //std::to_std::string is c++11
			table->AddColumn(arrY);
		}

		table->SetNumberOfRows(x.size());

		for (int ii = 0; ii < x.size(); ii++)
		{
			table->SetValue(ii, 0, x[ii]);
		}

		for (int ii = 0; ii < y[i].size(); ii++)
		{
			if (ii != i)
				continue;
			// Fill in the table with values
			for (int iii = 0; iii < y[i][ii].size(); ++iii)
			{
				table->SetValue(iii, 1, y[i][ii][iii]);
			}
		}

		// Add multiple line Plots, setting the colors etc
		view->GetScene()->AddItem(chart);

		vtkPlot *line;
		for (int ii = 0; ii < y[i].size(); ii++)
		{
			if (ii != i)
				continue;
			line = chart->AddPlot(vtkChart::LINE);
#if VTK_MAJOR_VERSION <= 5
			line->SetInput(table, 0, ii+1);
#else
			line->SetInputData(table, 0, 1);
#endif
			line->SetColor(cc[ii + 1][0], cc[ii + 1][1], cc[ii + 1][2], 255);
			line->SetWidth(2.0);
		}
	}

	// Start interactor
	view->GetInteractor()->Initialize();
	view->GetInteractor()->Start();
}
