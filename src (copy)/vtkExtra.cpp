/*
 * vtkExtra.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#include "vtkExtra.h"

// For backward compatibility with new VTK generic data arrays
#define InsertNextTypedTuple InsertNextTupleValue
//=============================================================================

void colorCode(vector<unsigned char*> &container)
{
	// Setup colors
	unsigned char cw[]   = {255, 255, 255};
	unsigned char cy[]   = {255, 255, 0};
	unsigned char co[]   = {255, 127, 0};
	unsigned char cr[]   = {255, 0, 0};
	unsigned char clg[]  = {127, 255, 0};
	unsigned char cg[]   = {0, 255, 0};
	unsigned char cgb[]  = {0, 255, 127};
	unsigned char cc[]   = {0, 255, 255};
	unsigned char clb[]  = {0, 127, 255};
	unsigned char cb[]   = {0, 0, 255};
	unsigned char cpb[]  = {127, 0, 255};
	unsigned char cpr[]  = {255, 0, 127};
	copy(cw, 	cw+3, 	container[0]);
	copy(cy, 	cy+3, 	container[1]);
	copy(co, 	co+3, 	container[2]);
	copy(cr, 	cr+3, 	container[3]);
	copy(clg, 	clg+3,	container[4]);
	copy(cg, 	cg+3, 	container[5]);
	copy(cgb, 	cgb+3, 	container[6]);
	copy(cc, 	cc+3, 	container[7]);
	copy(clb, 	clb+3, 	container[8]);
	copy(cb, 	cb+3, 	container[9]);
	copy(cpb, 	cpb+3, 	container[10]);
	copy(cpr, 	cpr+3, 	container[11]);
}

// Define interaction style
class customMouseInteractorStyle : public vtkInteractorStyleTrackballCamera
{
	public:
		static customMouseInteractorStyle* New();
		vtkTypeMacro(customMouseInteractorStyle,
				     vtkInteractorStyleTrackballCamera);

		void setLeftButton(bool pick){pick_ = pick;}

		void setColors(vector<unsigned char*> color){color_ = color;}

		void setLabels(vector<string> label){LABEL = label;}

		vector<string> getLabels(){return LABEL;}

		void setNumberOfLabels(int x){num_locations = x;}

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
		bool pick_;
		vector<string> LABEL;
		vector<unsigned char*> color_;
		void writeText(const char* text, double *rgb, int x, int y);
};

// Define interaction style
void customMouseInteractorStyle::OnLeftButtonDown()
{
	if(pick_)
	{
		//std::cout << "Pressed left mouse button." << std::endl;
		int* clickPos = this->GetInteractor()->GetEventPosition();

		// Pick from this location.
		vtkSmartPointer<vtkPointPicker>  picker = vtkSmartPointer<vtkPointPicker>::New();
		picker->Pick(clickPos[0], clickPos[1], 0, this->GetDefaultRenderer());

		double rgb[3];
		if (picker->GetActor()!=0)
		{
			double* pos = picker->GetPickPosition();
			std::cout << ">>>>> Pick position (world coordinates) is: "
					  << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;

			picker->GetActor()
				  ->GetMapper()->GetInput()
				  ->GetPointData()->GetScalars()
				  ->GetTuple(picker->GetPointId(),rgb);

			//std::cout << "Pick position (color) is: "
			//		  << rgb[0] << " " << rgb[1] << " " << rgb[2] << std::endl;
			//std::cout << "Picked actor: " << picker->GetActor() << std::endl;
		}

		for(int i = 0;i<num_locations;i++)
			if (rgb[0]==color_[i+1][0] &&
				rgb[1]==color_[i+1][1] &&
				rgb[2]==color_[i+1][2])
			{
				cout << ">>>>> Enter label : ";
				string mystr; getline (cin, mystr);
				printf(">>>>> Label %02d : %s\n", i+1, mystr.c_str());

				if (!LABEL[i+1].empty())
				{
					cout << ">>>>> [WARNING] : Label has been given. Do you wish to overwrite? [Y/N]" << endl;
					while(1)
					{
						string mystr2; getline (cin, mystr2);
						if(!strcmp(mystr2.c_str(),"Y"))
						{
							cout << ">>>>> [WARNING] : Label has been overwritten. New label : " << mystr << endl;
							LABEL[i+1] = mystr;
							writeText(mystr.c_str(), rgb, 10, 470-10*(i+1));
							printf(">>>>> Pick a location...\n");
							break;
						}
						if(!strcmp(mystr2.c_str(),"N"))
						{
							cout << ">>>>> [WARNING] : Label has not been overwritten." << endl;
							printf(">>>>> Pick a location...\n");
							break;
						}
					}
				}
				else
				{
					LABEL[i+1] = mystr;
					writeText(mystr.c_str(), rgb, 10, 470-10*(i+1));
					printf(">>>>> Pick a location...\n");
					break;
				}
			}

		for(int i = 1;i<num_locations+1;i++)
		{
			// when there is no labeling at all
			if (LABEL.empty()) break;
			// check for completeness of labelling
			if (LABEL[i].empty()) break;
			// else
			if (i==num_locations)
			{
				cout << ">>>>> [WARNING] : Label has been fully labeled. Proceed? [Y/N]" << endl;
				string mystr3; getline (cin, mystr3);
				if(!strcmp(mystr3.c_str(),"Y"))
					this->GetInteractor()->TerminateApp();
				else
					printf(">>>>> Pick a location...\n");
			}
		}
		// Forward events, camera manipulation
		vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
	}
	else
		vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

void customMouseInteractorStyle::writeText(const char* text, double *rgb, int x, int y)
{
	// Setup the text and add it to the renderer
	vtkSmartPointer<vtkTextActor> textActor;
	textActor = vtkSmartPointer<vtkTextActor>::New();
	textActor->SetInput ( text );
	textActor->SetPosition ( x, y );
	textActor->GetTextProperty()->SetFontSize ( 10 );
	textActor->GetTextProperty()->SetColor ( rgb[0]/255, rgb[1]/255, rgb[2]/255 );
	this->GetDefaultRenderer()->AddActor2D ( textActor );
	this->GetInteractor()->Render();
}

vtkStandardNewMacro(customMouseInteractorStyle);

void showData(
	vector<point_t> p,
	vector<string> &label,
	vector<unsigned char*> color_,
	bool cluster,
	bool labeling)
{
	int line_ = 0;
	int num_locations = 0;
	while(line_<p.size())
	{
		num_locations = max(p[line_].cluster_id,num_locations);
		++line_;
	}
	++num_locations;
	//cout << "Number of locations : " << num_locations << endl;

	// Create the geometry of a point (the coordinate)
	// add point to polydata to create the vertices for glyph
	// creating the vertices with a small cube glyph
	// add points and vertices to polydata
	// giving the points color
	// Create a mapper and actor
	// Create a renderer, render window, and interactor
	// custom mouse
	// Setup the text and add it to the renderer
	vtkSmartPointer<vtkPoints> 					points;
	vtkSmartPointer<vtkPolyData> 				pointsPolydata;
	vtkSmartPointer<vtkVertexGlyphFilter>		vertexFilter;
	vtkSmartPointer<vtkPolyData> 				polydata;
	vtkSmartPointer<vtkUnsignedCharArray> 		colors;
	vtkSmartPointer<vtkPolyDataMapper>			mapper;
	vtkSmartPointer<vtkActor> 					actor;
	vtkSmartPointer<vtkRenderer> 				renderer;
	vtkSmartPointer<vtkRenderWindow> 			renderWindow;
	vtkSmartPointer<vtkRenderWindowInteractor> 	renderWindowInteractor;
	vtkSmartPointer<customMouseInteractorStyle> style;
	vtkSmartPointer<vtkTextActor> 				textActor;

	points 					= vtkSmartPointer<vtkPoints>::New();
	pointsPolydata 			= vtkSmartPointer<vtkPolyData>::New();
	vertexFilter			= vtkSmartPointer<vtkVertexGlyphFilter>::New();
	polydata 				= vtkSmartPointer<vtkPolyData>::New();
	colors 					= vtkSmartPointer<vtkUnsignedCharArray>::New();
	mapper 					= vtkSmartPointer<vtkPolyDataMapper>::New();
	actor 					= vtkSmartPointer<vtkActor>::New();
	renderer 				= vtkSmartPointer<vtkRenderer>::New();
	renderWindow 			= vtkSmartPointer<vtkRenderWindow>::New();
	renderWindowInteractor 	= vtkSmartPointer<vtkRenderWindowInteractor>::New();
	style 					= vtkSmartPointer<customMouseInteractorStyle>::New();
//	textActor 				= vtkSmartPointer<vtkTextActor>::New();

	for(int i=0;i<p.size();i++)
		points->InsertNextPoint(p[i].x, p[i].y, p[i].z);

	pointsPolydata->SetPoints(points);

	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();

	polydata->ShallowCopy(vertexFilter->GetOutput());

//#if VTK_MAJOR_VERSION <= 5
//	vertexFilter->SetInputConnection(pointsPolydata->GetProducerPort());
//#else
//	vertexFilter->SetInputData(pointsPolydata);
//#endif

	if(cluster)
	{
		colors->SetNumberOfComponents(3);
		colors->SetName ("Colors");
		for(int i=0;i<p.size();i++)
		{
			if(p[i].cluster_id < num_locations && p[i].cluster_id >= 0)
				colors->InsertNextTypedTuple(color_[p[i].cluster_id+1]);
			else
				colors->InsertNextTypedTuple(color_[0]);
		}
		polydata->GetPointData()->SetScalars(colors);
	}

//	#if VTK_MAJOR_VERSION <= 5
//	  mapper->SetInput(polydata);
//	#else
//	  mapper->SetInputData(polydata);
//	#endif

	mapper->SetInputData(polydata);

	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(3);
//	actor->GetProperty()->SetColor(1.0, 0.0, 0.0);

	renderWindow->SetSize(640,480); //(width, height)
	renderWindow->AddRenderer(renderer);
	renderWindowInteractor->SetRenderWindow(renderWindow);

	if(labeling)
	{
		style->setLeftButton(true);
		textActor = vtkSmartPointer<vtkTextActor>::New();
		printf(">>>>> Pick a location...\n");
		if(!label.empty()) textActor->SetInput ( label[0].c_str() );
		textActor->SetPosition ( 10, 470 );
		textActor->GetTextProperty()->SetFontSize ( 10 );
		textActor->GetTextProperty()->SetColor ( 1.0, 1.0, 1.0 );
		renderer->AddActor2D ( textActor );
	}
	else
	{
		style->setLeftButton(false);
		for(int i=0;i<num_locations+1;i++)
		{
			if(label.empty()) continue;
			if(label[i].empty()) continue;
			textActor = vtkSmartPointer<vtkTextActor>::New();
			textActor->SetInput(label[i].c_str());
			textActor->SetPosition(10, 470-i*10);
			textActor->GetTextProperty()->SetFontSize(10);
			textActor->GetTextProperty()
					 ->SetColor((double)color_[i][0]/255,
								(double)color_[i][1]/255,
								(double)color_[i][2]/255);
			renderer->AddActor2D(textActor);
		}
	}

	// Add the actor to the scene
	renderer->AddActor(actor);
	//renderer->SetBackground(nc->GetColor3d("MidnightBlue").GetData());
	style->SetDefaultRenderer(renderer);
	style->setNumberOfLabels(num_locations);
	style->setLabels(label);
	style->setColors(color_);
	renderWindowInteractor->SetInteractorStyle( style );
	renderWindow->Render();
	renderWindowInteractor->Start();

	label = style->getLabels();
}

void showConnection(
	vector<vector<vector<sector_t> > > 	sector,
	vector<vector<vector<double> > > sector_constraint,
	vector<point_t> loc_loc_vec,
	vector<point_t> loc_loc_normal,
	vector<double>  loc_loc_norm,
	vector<point_t> location,
	int num_location_intervals,
	int num_sector_intervals,
	vector<unsigned char*> color_)
{
	int num_locations = location.size();

	vtkSmartPointer<vtkLineSource> 			lineSource	[Sqr(num_locations)];
	vtkSmartPointer<vtkPolyDataMapper> 		lineMapper	[Sqr(num_locations)];
	vtkSmartPointer<vtkActor> 				lineActor 	[Sqr(num_locations)];
	vtkSmartPointer<vtkPoints> 				points		[Sqr(num_locations)];
	vtkSmartPointer<vtkCellArray> 			lines 	   	[Sqr(num_locations)];
	vtkSmartPointer<vtkPolyData> 			polyData   	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkDoubleArray> 		tubeRadius 	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkTubeFilter> 			tubeFilter 	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkPolyDataMapper> 		tubeMapper 	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkActor> 				tubeActor  	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkUnsignedCharArray> 	colors     	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkPolyData> 			polyData2  	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkDoubleArray> 		tubeRadius2	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkTubeFilter> 			tubeFilter2	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkPolyDataMapper> 		tubeMapper2	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkActor> 				tubeActor2 	[Sqr(num_locations)][num_sector_intervals];
	vtkSmartPointer<vtkUnsignedCharArray> 	colors2    	[Sqr(num_locations)][num_sector_intervals];

	vtkSmartPointer<vtkRenderer> 				renderer;
	vtkSmartPointer<vtkRenderWindow> 			renderWindow;
	vtkSmartPointer<vtkRenderWindowInteractor> 	renderWindowInteractor;
	vtkSmartPointer<customMouseInteractorStyle> style;

	for(int i=0;i<Sqr(num_locations);i++)
//	for(int i=0;i<2;i++)
	{
		lineSource[i] = vtkSmartPointer<vtkLineSource>::New();
		lineMapper[i] = vtkSmartPointer<vtkPolyDataMapper>::New();
		lineActor [i] = vtkSmartPointer<vtkActor>::New();

		points[i] = vtkSmartPointer<vtkPoints>::New();
		lines [i] = vtkSmartPointer<vtkCellArray>::New();

		int c = i/num_locations;

		if(i == c*num_locations+c) continue;

		// Create a line between each location
		lineSource[i]->SetPoint1(location[c].x,
				          	     location[c].y,
				          	     location[c].z);
		lineSource[i]->SetPoint2(location[c].x + (loc_loc_vec[i].x * loc_loc_norm[i]),
							     location[c].y + (loc_loc_vec[i].y * loc_loc_norm[i]),
							     location[c].z + (loc_loc_vec[i].z * loc_loc_norm[i]));

		lineMapper[i]->SetInputConnection(lineSource[i]->GetOutputPort());

		lineActor[i]->GetProperty()->SetLineWidth(5); // Give some color to the line
		lineActor[i]->GetProperty()->SetColor(0.0,0.0,1.0); // Give some color to the line
		lineActor[i]->SetMapper(lineMapper[i]);

		// Create points between each location
		lines[i]->InsertNextCell(num_location_intervals*2);

		for(int ii=0;ii<num_location_intervals;ii++)
		{
			points[i]->InsertPoint(ii*2+0,
				location[c].x + loc_loc_vec[i].x * loc_loc_norm[i] * (ii+0)/num_location_intervals,
				location[c].y + loc_loc_vec[i].y * loc_loc_norm[i] * (ii+0)/num_location_intervals,
				location[c].z + loc_loc_vec[i].z * loc_loc_norm[i] * (ii+0)/num_location_intervals);
			points[i]->InsertPoint(ii*2+1,
				location[c].x + loc_loc_vec[i].x * loc_loc_norm[i] * (ii+1)/num_location_intervals * 0.99,
				location[c].y + loc_loc_vec[i].y * loc_loc_norm[i] * (ii+1)/num_location_intervals * 0.99,
				location[c].z + loc_loc_vec[i].z * loc_loc_norm[i] * (ii+1)/num_location_intervals * 0.99);
			lines[i]->InsertCellPoint(ii*2+0);
			lines[i]->InsertCellPoint(ii*2+1);
		}

		for(int ii=0;ii<num_sector_intervals;ii++)
		{
			colors     [i][ii] = vtkSmartPointer<vtkUnsignedCharArray>::New();
			polyData   [i][ii] = vtkSmartPointer<vtkPolyData>::New();
			tubeRadius [i][ii] = vtkSmartPointer<vtkDoubleArray>::New();
			tubeFilter [i][ii] = vtkSmartPointer<vtkTubeFilter>::New();
			colors2    [i][ii] = vtkSmartPointer<vtkUnsignedCharArray>::New();
			polyData2  [i][ii] = vtkSmartPointer<vtkPolyData>::New();
			tubeRadius2[i][ii] = vtkSmartPointer<vtkDoubleArray>::New();
			tubeFilter2[i][ii] = vtkSmartPointer<vtkTubeFilter>::New();

			colors[i][ii]->SetNumberOfComponents(3);
			colors[i][ii]->SetName ("Colors");
			colors2[i][ii]->SetNumberOfComponents(3);
			colors2[i][ii]->SetName ("Colors");

			polyData[i][ii]->SetPoints(points[i]);
			polyData[i][ii]->SetLines(lines[i]);
			polyData2[i][ii]->SetPoints(points[i]);
			polyData2[i][ii]->SetLines(lines[i]);

			tubeRadius[i][ii]->SetName("TubeRadius");
			tubeRadius[i][ii]->SetNumberOfTuples(num_location_intervals*2); //to create the small gap to line between sectors
			tubeRadius2[i][ii]->SetName("TubeRadius");
			tubeRadius2[i][ii]->SetNumberOfTuples(num_location_intervals*2); //to create the small gap to line between sectors

			for (int iii=0;iii<num_location_intervals;iii++)
			{
				if (sector_constraint[i][iii][ii]==0)
				{
					colors[i][ii]->InsertNextTypedTuple(color_[5]);
					colors[i][ii]->InsertNextTypedTuple(color_[5]);
				}
				else
				{
					colors[i][ii]->InsertNextTypedTuple(color_[1]);
					colors[i][ii]->InsertNextTypedTuple(color_[1]);
				}

				colors2[i][ii]->InsertNextTypedTuple(color_[3]);
				colors2[i][ii]->InsertNextTypedTuple(color_[3]);

				if(sector[i][iii][ii].min<0)
				{
					tubeRadius2[i][ii]->SetTuple1(iii*2+0, 0.0);
					tubeRadius2[i][ii]->SetTuple1(iii*2+1, 0.0);
				}
				else if(sector[i][iii][ii].min==INFINITY)
				{
					tubeRadius2[i][ii]->SetTuple1(iii*2+0, 0.0);
					tubeRadius2[i][ii]->SetTuple1(iii*2+1, 0.0);
				}
				else
				{
					tubeRadius2[i][ii]->SetTuple1(iii*2+0, sector[i][iii][ii].min);
					tubeRadius2[i][ii]->SetTuple1(iii*2+1, sector[i][iii][ii].min);
				}

				if(sector[i][iii][ii].max<=0)
				{
					tubeRadius[i][ii]->SetTuple1(iii*2+0, 0.0);
					tubeRadius[i][ii]->SetTuple1(iii*2+1, 0.0);
				}
				else
				{
					tubeRadius[i][ii]->SetTuple1(iii*2+0, sector[i][iii][ii].max);
					tubeRadius[i][ii]->SetTuple1(iii*2+1, sector[i][iii][ii].max);
				}
			}

			polyData[i][ii]->GetPointData()->AddArray(tubeRadius[i][ii]);
			polyData[i][ii]->GetPointData()->SetActiveScalars("TubeRadius");
			polyData[i][ii]->GetPointData()->AddArray(colors[i][ii]);
			polyData2[i][ii]->GetPointData()->AddArray(tubeRadius2[i][ii]);
			polyData2[i][ii]->GetPointData()->SetActiveScalars("TubeRadius");
			polyData2[i][ii]->GetPointData()->AddArray(colors2[i][ii]);

//#if VTK_MAJOR_VERSION <= 5
//			tubeFilter[i][ii]->SetInputConnection(polyData[i][ii]->GetProducerPort());
//#else
//			tubeFilter[i][ii]->SetInputData(polyData[i][ii]);
//#endif

			tubeFilter[i][ii]->SetInputData(polyData[i][ii]);
			tubeFilter[i][ii]->SetNumberOfSides(num_sector_intervals);
			tubeFilter[i][ii]->SetVaryRadiusToVaryRadiusByAbsoluteScalar();
			tubeFilter[i][ii]->SidesShareVerticesOff();
			tubeFilter[i][ii]->SetOnRatio(num_sector_intervals);
			tubeFilter[i][ii]->SetOffset(ii);
			tubeFilter[i][ii]->Update();
			tubeFilter2[i][ii]->SetInputData(polyData2[i][ii]);
			tubeFilter2[i][ii]->SetNumberOfSides(num_sector_intervals);
			tubeFilter2[i][ii]->SetVaryRadiusToVaryRadiusByAbsoluteScalar();
			tubeFilter2[i][ii]->SidesShareVerticesOff();
			tubeFilter2[i][ii]->SetOnRatio(num_sector_intervals);
			tubeFilter2[i][ii]->SetOffset(ii);
			//cout << num_sector_intervals << ii <<endl;
			tubeFilter2[i][ii]->Update();

			tubeMapper[i][ii] = vtkSmartPointer<vtkPolyDataMapper>::New();
			tubeMapper[i][ii]->SetInputConnection(tubeFilter[i][ii]->GetOutputPort());
			tubeMapper[i][ii]->ScalarVisibilityOn();
			tubeMapper[i][ii]->SetScalarModeToUsePointFieldData();
			tubeMapper[i][ii]->SelectColorArray("Colors");
			tubeMapper2[i][ii] = vtkSmartPointer<vtkPolyDataMapper>::New();
			tubeMapper2[i][ii]->SetInputConnection(tubeFilter2[i][ii]->GetOutputPort());
			tubeMapper2[i][ii]->ScalarVisibilityOn();
			tubeMapper2[i][ii]->SetScalarModeToUsePointFieldData();
			tubeMapper2[i][ii]->SelectColorArray("Colors");

			tubeActor[i][ii] = vtkSmartPointer<vtkActor>::New();
			tubeActor[i][ii]->GetProperty()->SetOpacity(0.75); //Make the tube have some transparency.
			tubeActor[i][ii]->SetMapper(tubeMapper[i][ii]);
			tubeActor2[i][ii] = vtkSmartPointer<vtkActor>::New();
			tubeActor2[i][ii]->GetProperty()->SetOpacity(1.0); //Make the tube have some transparency.
			tubeActor2[i][ii]->SetMapper(tubeMapper2[i][ii]);
		}
	}

	style 				   	= vtkSmartPointer<customMouseInteractorStyle>::New();
	renderer               	= vtkSmartPointer<vtkRenderer>::New();
	renderWindow           	= vtkSmartPointer<vtkRenderWindow>::New();
	renderWindowInteractor 	= vtkSmartPointer<vtkRenderWindowInteractor>::New();

	for(int i=0;i<Sqr(num_locations);i++)
//	for(int i=1;i<2;i++)
	{
//		int i = 6;
		renderer->AddActor(lineActor[i]);

		for(int ii=0;ii<num_sector_intervals;ii++)
		{
			renderer->AddActor(tubeActor[i][ii]);
			renderer->AddActor(tubeActor2[i][ii]);
		}
	}

	style->SetDefaultRenderer(renderer);
	style->setLeftButton(false);
	renderer->SetBackground(1.0,1.0,1.0);
	renderWindow->SetSize(1280,800); //(width, height)
	renderWindow->AddRenderer(renderer);
	renderWindowInteractor->SetRenderWindow(renderWindow);
	renderWindowInteractor->SetInteractorStyle(style);
	renderWindow->Render();
	renderWindowInteractor->Start();
}




