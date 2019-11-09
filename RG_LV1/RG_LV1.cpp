// RG_LV1.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"

#include <conio.h>
#include <vector>
#include <vtkAutoInit.h>
#include <math.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkAxesActor.h>
#include <vtkTextActor.h>
#include <vtkTransform.h>
#include "RVL3DTools.h"
#include "Display.h"
#include "Body.h"
#include <cmath>

#define PI	3.14159265359

double* rotateX(int angle){

	double rad = angle * PI / 180;

	double Tnew[16] = {
		1, 0, 0, 0,
		0, cos(rad), -sin(rad), 0,
		0, sin(rad), cos(rad), 0,
		0, 0, 0, 1
	};
	return Tnew;
}

double* rotateY(int angle) {

	double rad = angle * PI / 180;

	double Tnew[16] = {
		cos(rad), 0, sin(rad), 0,
		0, 1, 0, 0,
		-sin(rad), 0, cos(rad), 0,
		0, 0, 0, 1
	};
	return Tnew;
}

double* rotateZ(int angle) {

	double rad = angle * PI / 180;

	double Tnew[16] = {
		cos(rad), -sin(rad), 0, 0,
		sin(rad), cos(rad), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	};
	return Tnew;
}


int main(int argc, char *argv[])
{
	int rotation1 = 0, rotation2 = 0, rotation3 = 0;
	while (1){
		int mode;		

		cout << "input 1 to set rotations, input 2 to apply rotations, input 3 to exit\n";

		cin >> mode;

		if (mode == 3)
		{
			return 0;
		}
		else if (mode == 2){
			//render screen

			vtkSmartPointer<vtkMatrix4x4> T = vtkSmartPointer<vtkMatrix4x4>::New();

			//**** Create display. ****//

			Display window;

			window.Create(800, 600, "Racunalna geometrija LV1", 1.0, 1.0, 1.0);

			//**** Create scene. ****//

			Body scene;

			//1.  Base
			Body base;

			base.CreateBox(0.4, 0.4, 0.15, 112/255.0, 22/255.0, 20/255.0);

			scene.AddPart(&base);

			//**** Axis 1 ****//
			//Fixed position 1 relative to global coordinate system
			//position 0.1 above the centre of mass of base(1 / 2 base = 0.075 + 1 / 2 cylinder = 0.025)
			double T10[16] = {
				1, 0, 0, 0.0,
				0, 1, 0, 0.0,
				0, 0, 1, 0.1,
				0, 0, 0, 1
			};

			T->Multiply4x4(T10, rotateZ(rotation1), T10);


			//**** Axis 2 ****//
			////Fixed position 2 relative to position 1
			double T21[16] = {
				1, 0, 0, 0.275,
				0, 1, 0, 0.0,
				0, 0, 1, 0.15,
				0, 0, 0, 1
			};										

			//Determine T20 
			double T20[16];

			T->Multiply4x4(T10, T21, T20);
			T->Multiply4x4(T20, rotateZ(rotation2), T20);

			//**** Axis 3 ****//
			double T32[16] = {
				1, 0, 0, 0.275,
				0, 1, 0, -0.0625,
				0, 0, 1, 0.075,
				0, 0, 0, 1
			};			

			double T30[16];

			T->Multiply4x4(T20, T32, T30);
			T->Multiply4x4(T30, rotateY(rotation3), T30);

			//2. Joint 1

			Body joint1;

			scene.AddPart(&joint1);

			joint1.CreateCylinder(0.025, 0.05, 20, 0.2, 0.2, 0.2);

			//Joint 1 relative to Axis 1
			//Rotate joint about x axis by 90° 
			double TJ11[16] = {
				1, 0, 0, 0.0,
				0, 0, -1, 0.0,
				0, 1, 0, 0.0,
				0, 0, 0, 1
			};						

			double TJ10[16];

			T->Multiply4x4(T10, TJ11, TJ10);			

			joint1.Transform(TJ10);


			//3. Link 1
			Body link1;

			scene.AddPart(&link1);

			link1.CreateBox(0.1, 0.4, 0.1, 112 / 255.0, 22 / 255.0, 20 / 255.0);

			//Link 1 relative to Axis 1
			double TL11[16] = {
				0, -1, 0, 0.1375,
				1, 0, 0, 0.0,
				0, 0, 1, 0.075,
				0, 0, 0, 1
			};

			double TL10[16];

			T->Multiply4x4(T10, TL11, TL10);

			link1.Transform(TL10);

			//4.  Joint 2
			Body joint2;

			scene.AddPart(&joint2);

			joint2.CreateCylinder(0.025, 0.05, 20, 0.2, 0.2, 0.2);

			//Joint 2 relative to Axis 2 (Rotate joint about x axis by 90° )
			double TJ22[16] = {
				1, 0, 0, 0.0,
				0, 0, -1, 0.0,
				0, 1, 0, 0.0,
				0, 0, 0, 1
			};


			double TJ20[16];

			T->Multiply4x4(T20, TJ22, TJ20);

			joint2.Transform(TJ20);


			//5. Link 2
			Body link2;

			scene.AddPart(&link2);

			link2.CreateBox(0.1, 0.4, 0.1, 112 / 255.0, 22 / 255.0, 20 / 255.0);

			//Link 2 relative to Axis 2
			//Position link 2 above axis 2 (link 2 needs to be rotated about the x axis)
			double TL22[16] = {
				0, -1, 0, 0.1375,
				1, 0, 0, 0.0,
				0, 0, 1, 0.075,
				0, 0, 0, 1
			};


			double TL20[16];

			T->Multiply4x4(T20, TL22, TL20);

			link2.Transform(TL20);

			//5.  Joint 3
			Body joint3;

			scene.AddPart(&joint3);

			joint3.CreateCylinder(0.05, 0.025, 20, 0.2, 0.2, 0.2);

			double TJ33[16] = {
				1, 0, 0, 0.0,
				0, 1, 0, 0.0,
				0, 0, 1, 0.0,
				0, 0, 0, 1
			};

			double TJ30[16];

			T->Multiply4x4(T30, TJ33, TJ30);

			joint3.Transform(TJ30);

			//6. Link 3
			Body link3;

			scene.AddPart(&link3);

			link3.CreateBox(0.1, 0.1, 0.6, 112 / 255.0, 22 / 255.0, 20 / 255.0);

			double TL33[16] = {
				1, 0, 0, 0.0,
				0, 1, 0, -0.05,
				0, 0, 1, 0.25,
				0, 0, 0, 1,
			};

			double TL30[16];

			T->Multiply4x4(T30, TL33, TL30);

			link3.Transform(TL30);

			//**** Add reference axes ****//
			vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

			// The axes are positioned with a user transform
			vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
			transform->Translate(-0.5, 0.0, 0.0);

			axes->SetUserTransform(transform);
			axes->SetTotalLength(0.2, 0.2, 0.2);
			axes->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
			axes->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
			axes->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
			window.renderer->AddActor(axes);


			//**** Render scene. ****//
			scene.AddToDisplay(&window);

			window.Run();			
		}
		else if (mode == 1){
			cout << "rotation 1: ";
			cin >> rotation1;
			cout << "rotation 2: ";
			cin >> rotation2;
			cout << "rotation 3: ";
			cin >> rotation3;

			printf("rotation 1 is %d°, rotation 2 is %d°, rotation 3 is %d°\n", rotation1, rotation1, rotation3);
		}
	}
}
