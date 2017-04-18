/*
 * vtkExtra.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#include "vtkExtra.h"

// ============================================================================
// MOUSE
// ============================================================================

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
		int 					num_locations;
		bool 					pick_;
		vector<string> 			LABEL;
		vector<unsigned char*> 	color_;

		void writeText(
			const char* text,
			double *rgb,
			int x,
			int y);
};

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

void customMouseInteractorStyle::writeText(
	const char* text,
	double *rgb,
	int x,
	int y)
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

// ============================================================================
// COLORCODE
// ============================================================================

void colorCode(
	vector<unsigned char*> &container_)
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
	copy(cw, 	cw+3, 	container_[0]);
	copy(cy, 	cy+3, 	container_[1]);
	copy(co, 	co+3, 	container_[2]);
	copy(cr, 	cr+3, 	container_[3]);
	copy(clg, 	clg+3,	container_[4]);
	copy(cg, 	cg+3, 	container_[5]);
	copy(cgb, 	cgb+3, 	container_[6]);
	copy(cc, 	cc+3, 	container_[7]);
	copy(clb, 	clb+3, 	container_[8]);
	copy(cb, 	cb+3, 	container_[9]);
	copy(cpb, 	cpb+3, 	container_[10]);
	copy(cpr, 	cpr+3, 	container_[11]);
}

vtkSmartPointer<vtkPolyDataMapper> dataPoints(
	vector<point_t> points_,
	int num_locations_,
	vector<unsigned char*> color_,
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
	vtkSmartPointer<vtkPoints> 					points;
	vtkSmartPointer<vtkPolyData> 				pointsPolydata;
	vtkSmartPointer<vtkVertexGlyphFilter>		vertexFilter;
	vtkSmartPointer<vtkPolyData> 				polydata;
	vtkSmartPointer<vtkUnsignedCharArray> 		colors;
	vtkSmartPointer<vtkPolyDataMapper>			mapper;
//	vtkSmartPointer<vtkActor> 					actor;
//	vtkSmartPointer<vtkRenderer> 				renderer;
//	vtkSmartPointer<vtkRenderWindow> 			renderWindow;
//	vtkSmartPointer<vtkRenderWindowInteractor> 	renderWindowInteractor;
//	vtkSmartPointer<customMouseInteractorStyle> style;
//	vtkSmartPointer<vtkTextActor> 				textActor;

	points 					= vtkSmartPointer<vtkPoints>::New();
	pointsPolydata 			= vtkSmartPointer<vtkPolyData>::New();
	vertexFilter			= vtkSmartPointer<vtkVertexGlyphFilter>::New();
	polydata 				= vtkSmartPointer<vtkPolyData>::New();
	colors 					= vtkSmartPointer<vtkUnsignedCharArray>::New();
	mapper 					= vtkSmartPointer<vtkPolyDataMapper>::New();

	for(int i=0;i<points_.size();i++)
		points->InsertNextPoint(points_[i].x, points_[i].y, points_[i].z);

	pointsPolydata->SetPoints(points);

#if VTK_MAJOR_VERSION <= 5
	vertexFilter->SetInputConnection(pointsPolydata->GetProducerPort());
	vertexFilter->Update();
#else
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();
#endif

	polydata->ShallowCopy(vertexFilter->GetOutput());

	if(cluster_)
	{
		colors->SetNumberOfComponents(3);
		colors->SetName ("Colors");
		for(int i=0;i<points_.size();i++)
		{
			if (points_[i].cluster_id < num_locations &&
				points_[i].cluster_id >= 0)
				colors->InsertNextTypedTuple(color_[points_[i].cluster_id+1]);
			else
				colors->InsertNextTypedTuple(color_[0]);
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

void showData(
	vector<point_t> points_,
	vector<string> &labels_,
	vector<unsigned char*> color_,
	bool cluster_,
	bool labeling_)
{
	int num_locations = labels_.size() - 1;

	vtkSmartPointer<vtkPolyDataMapper>			mapper;
	vtkSmartPointer<vtkActor> 					actor;
	vtkSmartPointer<vtkRenderer> 				renderer;
	vtkSmartPointer<vtkRenderWindow> 			renderWindow;
	vtkSmartPointer<vtkRenderWindowInteractor> 	renderWindowInteractor;
	vtkSmartPointer<customMouseInteractorStyle> style;
	vtkSmartPointer<vtkTextActor> 				textActor;

	mapper 					= vtkSmartPointer<vtkPolyDataMapper>::New();
	actor 					= vtkSmartPointer<vtkActor>::New();
	renderer 				= vtkSmartPointer<vtkRenderer>::New();
	renderWindow 			= vtkSmartPointer<vtkRenderWindow>::New();
	renderWindowInteractor 	= vtkSmartPointer<vtkRenderWindowInteractor>::New();
	style 					= vtkSmartPointer<customMouseInteractorStyle>::New();
//	textActor 				= vtkSmartPointer<vtkTextActor>::New();

	mapper =  dataPoints(points_, num_locations, color_, cluster_);

	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(3);
//	actor->GetProperty()->SetColor(1.0, 0.0, 0.0);

	renderWindow->SetSize(1280,800); //(width, height)
	renderWindow->AddRenderer(renderer);
	renderWindowInteractor->SetRenderWindow(renderWindow);

	if(labeling_)
	{
		style->setLeftButton(true);
		textActor = vtkSmartPointer<vtkTextActor>::New();
		printf(">>>>> Pick a location...\n");
		if(!labels_.empty()) textActor->SetInput (labels_[0].c_str());
		textActor->SetPosition ( 10, 470 );
		textActor->GetTextProperty()->SetFontSize ( 10 );
		textActor->GetTextProperty()->SetColor ( 1.0, 1.0, 1.0 );
		renderer->AddActor2D (textActor);
	}
	else
	{
		style->setLeftButton(false);
		for(int i=0;i<num_locations+1;i++)
		{
			if(labels_.empty()) continue;
			if(labels_[i].empty()) continue;
			textActor = vtkSmartPointer<vtkTextActor>::New();
			textActor->SetInput(labels_[i].c_str());
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
	style->setLabels(labels_);
	style->setColors(color_);
	renderWindowInteractor->SetInteractorStyle( style );
	renderWindow->Render();
	renderWindowInteractor->Start();

	labels_ = style->getLabels();
}

void showConnectionOnly(
	Graph Graph_,
	vector<unsigned char*> color_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();
	vector<point_t> locations(num_locations);
	for(int i=0;i<num_locations;i++) {locations[i] = nodes[i].location;}

	sector_para_t sector_para = Graph_.getSectorPara();

	vtkSmartPointer<vtkLineSource> 			lineSource	[Sqr(num_locations)];
	vtkSmartPointer<vtkPolyDataMapper> 		lineMapper	[Sqr(num_locations)];
	vtkSmartPointer<vtkActor> 				lineActor 	[Sqr(num_locations)];
	vtkSmartPointer<vtkPoints> 				points		[Sqr(num_locations)];
	vtkSmartPointer<vtkCellArray> 			lines 	   	[Sqr(num_locations)];
	vtkSmartPointer<vtkPolyData> 			polyData   	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkDoubleArray> 		tubeRadius 	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkTubeFilter> 			tubeFilter 	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkPolyDataMapper> 		tubeMapper 	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkActor> 				tubeActor  	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkUnsignedCharArray> 	colors     	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkPolyData> 			polyData2  	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkDoubleArray> 		tubeRadius2	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkTubeFilter> 			tubeFilter2	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkPolyDataMapper> 		tubeMapper2	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkActor> 				tubeActor2 	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkUnsignedCharArray> 	colors2    	[Sqr(num_locations)][sector_para.sec_int];

	vtkSmartPointer<vtkRenderer> 				renderer;
	vtkSmartPointer<vtkRenderWindow> 			renderWindow;
	vtkSmartPointer<vtkRenderWindowInteractor> 	renderWindowInteractor;
	vtkSmartPointer<customMouseInteractorStyle> style;

	vector<vector<edge_tt> > sector = Graph_.getEdgeList();

	for(int i=0;i<Sqr(num_locations);i++)
	{
		lineSource[i] = vtkSmartPointer<vtkLineSource>::New();
		lineMapper[i] = vtkSmartPointer<vtkPolyDataMapper>::New();
		lineActor [i] = vtkSmartPointer<vtkActor>::New();

		points[i] = vtkSmartPointer<vtkPoints>::New();
		lines [i] = vtkSmartPointer<vtkCellArray>::New();

		int c = i/num_locations;

		if(i == c*num_locations+c) continue;

		// Create a line between each location
		lineSource[i]->SetPoint1(locations[c].x,
				          	     locations[c].y,
				          	     locations[c].z);
		lineSource[i]->SetPoint2(locations[c].x + (sector_para.dir[i].x * sector_para.dist[i]),
							     locations[c].y + (sector_para.dir[i].y * sector_para.dist[i]),
							     locations[c].z + (sector_para.dir[i].z * sector_para.dist[i]));

		lineMapper[i]->SetInputConnection(lineSource[i]->GetOutputPort());

		lineActor[i]->GetProperty()->SetLineWidth(5); // Give some color to the line
		lineActor[i]->GetProperty()->SetColor(0.0,0.0,1.0); // Give some color to the line
		lineActor[i]->SetMapper(lineMapper[i]);

		// Create points between each locations
		lines[i]->InsertNextCell(sector_para.loc_int*2);

		for(int ii=0;ii<sector_para.loc_int;ii++)
		{
			points[i]->InsertPoint(ii*2+0,
				locations[c].x + sector_para.dir[i].x * sector_para.dist[i] * (ii+0)/sector_para.loc_int,
				locations[c].y + sector_para.dir[i].y * sector_para.dist[i] * (ii+0)/sector_para.loc_int,
				locations[c].z + sector_para.dir[i].z * sector_para.dist[i] * (ii+0)/sector_para.loc_int);
			points[i]->InsertPoint(ii*2+1,
				locations[c].x + sector_para.dir[i].x * sector_para.dist[i] * (ii+1)/sector_para.loc_int * 0.99,
				locations[c].y + sector_para.dir[i].y * sector_para.dist[i] * (ii+1)/sector_para.loc_int * 0.99,
				locations[c].z + sector_para.dir[i].z * sector_para.dist[i] * (ii+1)/sector_para.loc_int * 0.99);
			lines[i]->InsertCellPoint(ii*2+0);
			lines[i]->InsertCellPoint(ii*2+1);
		}

		for(int ii=0;ii<sector_para.sec_int;ii++)
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
			tubeRadius[i][ii]->SetNumberOfTuples(sector_para.loc_int*2); //to create the small gap to line between sectors
			tubeRadius2[i][ii]->SetName("TubeRadius");
			tubeRadius2[i][ii]->SetNumberOfTuples(sector_para.loc_int*2); //to create the small gap to line between sectors

			for (int iii=0;iii<sector_para.loc_int;iii++)
			{
				if (sector[i][0].sector_const[iii*sector_para.sec_int+ii]==0)
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

				if(sector[i][0].sector_map[iii*sector_para.sec_int+ii].min<0)
				{
					tubeRadius2[i][ii]->SetTuple1(iii*2+0, 0.0);
					tubeRadius2[i][ii]->SetTuple1(iii*2+1, 0.0);
				}
				else if(sector[i][0].sector_map[iii*sector_para.sec_int+ii].min==INFINITY)
				{
					tubeRadius2[i][ii]->SetTuple1(iii*2+0, 0.0);
					tubeRadius2[i][ii]->SetTuple1(iii*2+1, 0.0);
				}
				else
				{
					tubeRadius2[i][ii]->SetTuple1(iii*2+0, sector[i][0].sector_map[iii*sector_para.sec_int+ii].min);
					tubeRadius2[i][ii]->SetTuple1(iii*2+1, sector[i][0].sector_map[iii*sector_para.sec_int+ii].min);
				}

				if(sector[i][0].sector_map[iii*sector_para.sec_int+ii].max<=0)
				{
					tubeRadius[i][ii]->SetTuple1(iii*2+0, 0.0);
					tubeRadius[i][ii]->SetTuple1(iii*2+1, 0.0);
				}
				else
				{
					tubeRadius[i][ii]->SetTuple1(iii*2+0, sector[i][0].sector_map[iii*sector_para.sec_int+ii].max);
					tubeRadius[i][ii]->SetTuple1(iii*2+1, sector[i][0].sector_map[iii*sector_para.sec_int+ii].max);
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
			tubeFilter[i][ii]->SetNumberOfSides(sector_para.sec_int);
			tubeFilter[i][ii]->SetVaryRadiusToVaryRadiusByAbsoluteScalar();
			tubeFilter[i][ii]->SidesShareVerticesOff();
			tubeFilter[i][ii]->SetOnRatio(sector_para.sec_int);
			tubeFilter[i][ii]->SetOffset(ii);
			tubeFilter[i][ii]->Update();
			tubeFilter2[i][ii]->SetInputData(polyData2[i][ii]);
			tubeFilter2[i][ii]->SetNumberOfSides(sector_para.sec_int);
			tubeFilter2[i][ii]->SetVaryRadiusToVaryRadiusByAbsoluteScalar();
			tubeFilter2[i][ii]->SidesShareVerticesOff();
			tubeFilter2[i][ii]->SetOnRatio(sector_para.sec_int);
			tubeFilter2[i][ii]->SetOffset(ii);
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
	{
		renderer->AddActor(lineActor[i]);

		for(int ii=0;ii<sector_para.sec_int;ii++)
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

void showConnection(
	vector<point_t> points_,
	vector<string> &labels_,
	Graph Graph_,
	vector<unsigned char*> color_,
	bool show_points)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();
	vector<point_t> locations(num_locations);
	for(int i=0;i<num_locations;i++) {locations[i] = nodes[i].location;}

	sector_para_t sector_para = Graph_.getSectorPara();

	vtkSmartPointer<vtkLineSource> 			lineSource	[Sqr(num_locations)];
	vtkSmartPointer<vtkPolyDataMapper> 		lineMapper	[Sqr(num_locations)];
	vtkSmartPointer<vtkActor> 				lineActor 	[Sqr(num_locations)];
	vtkSmartPointer<vtkPoints> 				points		[Sqr(num_locations)];
	vtkSmartPointer<vtkCellArray> 			lines 	   	[Sqr(num_locations)];
	vtkSmartPointer<vtkPolyData> 			polyData   	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkDoubleArray> 		tubeRadius 	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkTubeFilter> 			tubeFilter 	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkPolyDataMapper> 		tubeMapper 	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkActor> 				tubeActor  	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkUnsignedCharArray> 	colors     	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkPolyData> 			polyData2  	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkDoubleArray> 		tubeRadius2	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkTubeFilter> 			tubeFilter2	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkPolyDataMapper> 		tubeMapper2	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkActor> 				tubeActor2 	[Sqr(num_locations)][sector_para.sec_int];
	vtkSmartPointer<vtkUnsignedCharArray> 	colors2    	[Sqr(num_locations)][sector_para.sec_int];

	vtkSmartPointer<vtkRenderer> 				renderer;
	vtkSmartPointer<vtkRenderWindow> 			renderWindow;
	vtkSmartPointer<vtkRenderWindowInteractor> 	renderWindowInteractor;
	vtkSmartPointer<customMouseInteractorStyle> style;

	vector<vector<edge_tt> > sector = Graph_.getEdgeList();
	double orientation[3];

	for(int i=0;i<Sqr(num_locations);i++)
	{
		lineSource[i] = vtkSmartPointer<vtkLineSource>::New();
		lineMapper[i] = vtkSmartPointer<vtkPolyDataMapper>::New();
		lineActor [i] = vtkSmartPointer<vtkActor>::New();

		points[i] = vtkSmartPointer<vtkPoints>::New();
		lines [i] = vtkSmartPointer<vtkCellArray>::New();

		int c = i/num_locations;

		if(i == c*num_locations+c) continue;

		orientation[0] = Graph_.getSectorPara().dir_n[i].x;
		orientation[1] = Graph_.getSectorPara().dir_n[i].y;
		orientation[2] = Graph_.getSectorPara().dir_n[i].z;

		// Create a line between each location
		lineSource[i]->SetPoint1(locations[c].x,
				          	     locations[c].y,
				          	     locations[c].z);
		lineSource[i]->SetPoint2(locations[c].x + (sector_para.dir[i].x * sector_para.dist[i]),
							     locations[c].y + (sector_para.dir[i].y * sector_para.dist[i]),
							     locations[c].z + (sector_para.dir[i].z * sector_para.dist[i]));

		lineMapper[i]->SetInputConnection(lineSource[i]->GetOutputPort());

		lineActor[i]->GetProperty()->SetLineWidth(5); // Give some color to the line
		lineActor[i]->GetProperty()->SetColor(0.0,0.0,1.0); // Give some color to the line
		lineActor[i]->SetMapper(lineMapper[i]);

		// Create points between each locations
		lines[i]->InsertNextCell(sector_para.loc_int*2);

		for(int ii=0;ii<sector_para.loc_int;ii++)
		{
			points[i]->InsertPoint(ii*2+0,
				locations[c].x + sector_para.dir[i].x * sector_para.dist[i] * (ii+0)/sector_para.loc_int,
				locations[c].y + sector_para.dir[i].y * sector_para.dist[i] * (ii+0)/sector_para.loc_int,
				locations[c].z + sector_para.dir[i].z * sector_para.dist[i] * (ii+0)/sector_para.loc_int);
			points[i]->InsertPoint(ii*2+1,
				locations[c].x + sector_para.dir[i].x * sector_para.dist[i] * (ii+1)/sector_para.loc_int * 0.99,
				locations[c].y + sector_para.dir[i].y * sector_para.dist[i] * (ii+1)/sector_para.loc_int * 0.99,
				locations[c].z + sector_para.dir[i].z * sector_para.dist[i] * (ii+1)/sector_para.loc_int * 0.99);
			lines[i]->InsertCellPoint(ii*2+0);
			lines[i]->InsertCellPoint(ii*2+1);
		}

		for(int ii=0;ii<sector_para.sec_int;ii++)
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
			tubeRadius[i][ii]->SetNumberOfTuples(sector_para.loc_int*2); //to create the small gap to line between sectors
			tubeRadius2[i][ii]->SetName("TubeRadius");
			tubeRadius2[i][ii]->SetNumberOfTuples(sector_para.loc_int*2); //to create the small gap to line between sectors

			for (int iii=0;iii<sector_para.loc_int;iii++)
			{
				// Adding different color based on constraint
				if (sector[i][0].sector_const[iii*sector_para.sec_int+ii]==0)
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

				if(sector[i][0].sector_map[iii*sector_para.sec_int+ii].min<0)
				{
					tubeRadius2[i][ii]->SetTuple1(iii*2+0, 0.0);
					tubeRadius2[i][ii]->SetTuple1(iii*2+1, 0.0);
				}
				else if(sector[i][0].sector_map[iii*sector_para.sec_int+ii].min==INFINITY)
				{
					tubeRadius2[i][ii]->SetTuple1(iii*2+0, 0.0);
					tubeRadius2[i][ii]->SetTuple1(iii*2+1, 0.0);
				}
				else
				{
					tubeRadius2[i][ii]->SetTuple1(iii*2+0, sector[i][0].sector_map[iii*sector_para.sec_int+ii].min);
					tubeRadius2[i][ii]->SetTuple1(iii*2+1, sector[i][0].sector_map[iii*sector_para.sec_int+ii].min);
				}

				if(sector[i][0].sector_map[iii*sector_para.sec_int+ii].max<=0)
				{
					tubeRadius[i][ii]->SetTuple1(iii*2+0, 0.0);
					tubeRadius[i][ii]->SetTuple1(iii*2+1, 0.0);
				}
				else
				{
					tubeRadius[i][ii]->SetTuple1(iii*2+0, sector[i][0].sector_map[iii*sector_para.sec_int+ii].max);
					tubeRadius[i][ii]->SetTuple1(iii*2+1, sector[i][0].sector_map[iii*sector_para.sec_int+ii].max);
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
			tubeFilter[i][ii]->SetNumberOfSides(sector_para.sec_int);
			tubeFilter[i][ii]->SetVaryRadiusToVaryRadiusByAbsoluteScalar();
			tubeFilter[i][ii]->SidesShareVerticesOff();
			tubeFilter[i][ii]->SetOnRatio(sector_para.sec_int);
			tubeFilter[i][ii]->SetOffset((ii+9)%sector_para.sec_int); //###quickfix need to turn it 90° anticlockwise
			tubeFilter[i][ii]->SetDefaultNormal(orientation);
			tubeFilter[i][ii]->UseDefaultNormalOn();
			tubeFilter[i][ii]->Update();

			tubeFilter2[i][ii]->SetInputData(polyData2[i][ii]);
			tubeFilter2[i][ii]->SetNumberOfSides(sector_para.sec_int);
			tubeFilter2[i][ii]->SetVaryRadiusToVaryRadiusByAbsoluteScalar();
			tubeFilter2[i][ii]->SidesShareVerticesOff();
			tubeFilter2[i][ii]->SetOnRatio(sector_para.sec_int);
			tubeFilter2[i][ii]->SetOffset((ii+9)%sector_para.sec_int); //###quickfix need to turn it 90° anticlockwise
			tubeFilter2[i][ii]->SetDefaultNormal(orientation);
			tubeFilter2[i][ii]->UseDefaultNormalOn();
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
			tubeActor[i][ii]->GetProperty()->SetOpacity(0.50); //Make the tube have some transparency.
			tubeActor[i][ii]->SetMapper(tubeMapper[i][ii]);
			tubeActor2[i][ii] = vtkSmartPointer<vtkActor>::New();
			tubeActor2[i][ii]->GetProperty()->SetOpacity(0.75); //Make the tube have some transparency.
			tubeActor2[i][ii]->SetMapper(tubeMapper2[i][ii]);
		}
	}

	style 				   	= vtkSmartPointer<customMouseInteractorStyle>::New();
	renderer               	= vtkSmartPointer<vtkRenderer>::New();
	renderWindow           	= vtkSmartPointer<vtkRenderWindow>::New();
	renderWindowInteractor 	= vtkSmartPointer<vtkRenderWindowInteractor>::New();

	for(int i=0;i<Sqr(num_locations);i++)
	{
		renderer->AddActor(lineActor[i]);

		for(int ii=0;ii<sector_para.sec_int;ii++)
		{
			renderer->AddActor(tubeActor[i][ii]);
			renderer->AddActor(tubeActor2[i][ii]);
		}
	}


//	vtkSmartPointer<vtkLineSource> 			lineSource2;
//	vtkSmartPointer<vtkPolyDataMapper> 		lineMapper2;
//	vtkSmartPointer<vtkActor> 				lineActor2;
//	vtkSmartPointer<vtkCellArray> 			lines2;
//	lineSource2 = vtkSmartPointer<vtkLineSource>::New();
//	lineMapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
//	lineActor2 = vtkSmartPointer<vtkActor>::New();
//	lines2 = vtkSmartPointer<vtkCellArray>::New();
//	lineSource2->SetPoint1(locations[2].x + (sector_para.dir[11].x * sector_para.dist[11]*0.5),
//						   locations[2].y + (sector_para.dir[11].y * sector_para.dist[11]*0.5),
//						   locations[2].z + (sector_para.dir[11].z * sector_para.dist[11]*0.5));
//	lineSource2->SetPoint2(locations[2].x + (sector_para.dir[11].x * sector_para.dist[11]*0.5) + sector_para.dir_n[11].x,
//						   locations[2].y + (sector_para.dir[11].y * sector_para.dist[11]*0.5) + sector_para.dir_n[11].y,
//						   locations[2].z + (sector_para.dir[11].z * sector_para.dist[11]*0.5) + sector_para.dir_n[11].z);
//	lineMapper2->SetInputConnection(lineSource2->GetOutputPort());
//	lineActor2->GetProperty()->SetLineWidth(5); // Give some color to the line
//	lineActor2->GetProperty()->SetColor(0.0,1.0,0.0); // Give some color to the line
//	lineActor2->SetMapper(lineMapper2);
//	renderer->AddActor(lineActor2);


	// [ADDING DATA]***********************************************************
	if (show_points)
	{
		vtkSmartPointer<vtkPolyDataMapper> 	mapper;
		vtkSmartPointer<vtkActor> 			actor;
		mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		actor  = vtkSmartPointer<vtkActor>::New();
		mapper =  dataPoints(points_, num_locations, color_, true);
		actor->SetMapper(mapper);
		actor->GetProperty()->SetPointSize(5);
		renderer->AddActor(actor);
		style->setNumberOfLabels(num_locations);
		style->setLabels(labels_);
		style->setColors(color_);
	}
	// ***********************************************************[ADDING DATA]

	style->SetDefaultRenderer(renderer);
	style->setLeftButton(false);
	renderer->SetBackground(0.0,0.0,0.0);
	renderWindow->SetSize(1280,800); //(width, height)
	renderWindow->AddRenderer(renderer);
	renderWindowInteractor->SetRenderWindow(renderWindow);
	renderWindowInteractor->SetInteractorStyle(style);
	renderWindow->Render();
	renderWindowInteractor->Start();
}

void plotData(
	vector<double> x,
	vector<double> y)
{
	vtkSmartPointer<vtkTable> 		table;
	vtkSmartPointer<vtkFloatArray> 	arrX;
	vtkSmartPointer<vtkFloatArray> 	arrY;
	vtkSmartPointer<vtkContextView> view;
	vtkSmartPointer<vtkChartXY> 	chart;

	table =	vtkSmartPointer<vtkTable>::New();
	arrX  = vtkSmartPointer<vtkFloatArray>::New();
	arrY  = vtkSmartPointer<vtkFloatArray>::New();
	view  = vtkSmartPointer<vtkContextView>::New();
	chart =	vtkSmartPointer<vtkChartXY>::New();

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
	view->GetRenderer()->GetRenderWindow()->SetSize(1280,800); //(width, height)

	// Add multiple line plots, setting the colors etc
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

