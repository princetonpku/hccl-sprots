#ifndef DEFORMABLEREGISTRATION_H
#define DEFORMABLEREGISTRATION_H

#define HAO_LI

#include <QtGui/QMainWindow>
#include "ui_DeformableRegistration.h"

class DeformableRegistration : public QMainWindow
{
	Q_OBJECT

public:
	DeformableRegistration(QWidget *parent = 0, Qt::WFlags flags = 0);
	~DeformableRegistration();

//private:	// юс╫ц
	Ui::DeformableRegistrationClass ui;

private slots:
	void OnFileNew();
	void OnFileOpenTemplate();
	void OnFileOpenTarget();
	void OnFileSaveTemplate();
	void OnFileSaveTarget();
	void OnFileExit();

	void OnToolsDecimate();
	void OnToolsSample_Random();			// Random Sampling
	void OnToolsSample_Quad();				// Sampling by Quadric Fitting
	void OnToolsSample_Uniform_Dart();		// Sampling by user-defined distance
	void OnToolsEmbededDeformation();

	void OnViewDeformationGraph();


	/////////////////
	// Hoa Li part //
	/////////////////
	void OnToolsInitHaoLi();
	void OnToolsRunHaoLi();

	void OnInitGeo();
	void OnToolsGeodesic();
};

#endif // DEFORMABLEREGISTRATION_H
