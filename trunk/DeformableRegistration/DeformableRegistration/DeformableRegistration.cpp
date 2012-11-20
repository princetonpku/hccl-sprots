#include "DeformableRegistration.h"
#include <QFileDialog>
#include <QMessageBox>

DeformableRegistration::DeformableRegistration(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
	ui.view->pParentDlg = this;
	ui.horizontalLayout->setContentsMargins(0, 0, 0, 0);

	connect(ui.actionNew, SIGNAL(triggered()), this, SLOT(OnFileNew()));
	connect(ui.actionOpenTemplate, SIGNAL(triggered()), this, SLOT(OnFileOpenTemplate()));
	connect(ui.actionOpenTarget, SIGNAL(triggered()), this, SLOT(OnFileOpenTarget()));
	connect(ui.actionSaveTemplate, SIGNAL(triggered()), this, SLOT(OnFileSaveTemplate()));
	connect(ui.actionSaveTarget, SIGNAL(triggered()), this, SLOT(OnFileSaveTarget()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(OnFileExit()));

	connect(ui.actionPoint, SIGNAL(triggered()), this, SLOT(OnUpdateGL()));
	connect(ui.actionWireframe, SIGNAL(triggered()), this, SLOT(OnUpdateGL()));
	connect(ui.actionFace, SIGNAL(triggered()), this, SLOT(OnUpdateGL()));
	connect(ui.actionSmooth, SIGNAL(triggered()), this, SLOT(OnUpdateGL()));
	connect(ui.actionTemplateVisible, SIGNAL(triggered()), this, SLOT(OnUpdateGL()));

	connect(ui.actionPoints_2, SIGNAL(triggered()), this, SLOT(OnUpdateGL()));
	connect(ui.actionWireframe_2, SIGNAL(triggered()), this, SLOT(OnUpdateGL()));
	connect(ui.actionFlat_2, SIGNAL(triggered()), this, SLOT(OnUpdateGL()));
	connect(ui.actionSmooth_2, SIGNAL(triggered()), this, SLOT(OnUpdateGL()));
	connect(ui.actionTargetVisible, SIGNAL(triggered()), this, SLOT(OnUpdateGL()));

	connect(ui.actionDecimate, SIGNAL(triggered()), this, SLOT(OnToolsDecimate()));
	connect(ui.actionRANDOM, SIGNAL(triggered()), this, SLOT(OnToolsSample_Random()));
	connect(ui.actionQuadricFitting, SIGNAL(triggered()), this, SLOT(OnToolsSample_Quad()));
	connect(ui.actionDartThrowing, SIGNAL(triggered()), this, SLOT(OnToolsSample_Uniform_Dart()));
	connect(ui.actionEmbededDeformation, SIGNAL(triggered()), this, SLOT(OnToolsEmbededDeformation()));
	srand((unsigned)time(NULL));			// 매번 다른 random number 생성을 위해
	
#ifdef HAO_LI
	connect(ui.actionInitalize_HaoLi, SIGNAL(triggered()), this, SLOT(OnToolsInitHaoLi()));
	connect(ui.actionStart_HaoLi, SIGNAL(triggered()), this, SLOT(OnToolsRunHaoLi()));
	connect(ui.actionGeodesic, SIGNAL(triggered()), this, SLOT(OnToolsGeodesic()));
	connect(ui.actionInitGeo, SIGNAL(triggered()), this, SLOT(OnInitGeo()));
#endif

}

DeformableRegistration::~DeformableRegistration()
{

}

void DeformableRegistration::OnFileNew()
{
	ui.view->Clear();
}

void DeformableRegistration::OnFileOpenTemplate()
{
	char szFilter[] = "All supported formats (*.ply *.obj *.stl *.off *.om);;\
					  Stanford Polygon Library (*.ply);;\
					  Object File Format(*.off);;\
					  Wavefront Object (*.obj) ;;\
					  Stereolithography (*.stl)";

	QString file = QFileDialog::getOpenFileName(this, "Select a model file to open", "", szFilter);
	if(file == "")
		return;
	setCursor(Qt::WaitCursor);
	ui.view->templ.Clear();
	ui.view->graph.Clear();
	ui.view->templ.Read(file.toStdString().c_str());

	ui.view->templ.UpdateCOG();
	ui.view->templ.UpdateBoundingSphere();
	double r = ui.view->templ.GetBoundingSphereRadius();
	
	std::cout << "the num of vertexes : " << ui.view->templ.n_vertices() << std::endl;
	std::cout << "the num of faces : " << ui.view->templ.n_faces() << std::endl;
	std::cout << "cog : " << ui.view->templ.cog[0] << ", " << ui.view->templ.cog[1] << ", " << ui.view->templ.cog[2] << std::endl;

// 	ui.view->camera()->setPosition(qglviewer::Vec(0,0,0));
// 	ui.view->camera()->lookAt(qglviewer::Vec(0,0,1));	
// 	ui.view->camera()->setUpVector(qglviewer::Vec(0,-1,0));

	ui.view->camera()->setSceneRadius(r/**0.2*/);
	ui.view->camera()->showEntireScene();	

	ui.view->updateGL();
	setCursor(Qt::ArrowCursor);
}

void DeformableRegistration::OnFileOpenTarget()
{	
	char szFilter[] = "All supported formats (*.ply *.obj *.stl *.off *.om);;\
					  Stanford Polygon Library (*.ply);;\
					  Object File Format(*.off);;\
					  Wavefront Object (*.obj) ;;\
					  Stereolithography (*.stl)";

	QString file = QFileDialog::getOpenFileName(this, "Select a model file to open", "", szFilter);
	if(file == "")
		return;
	setCursor(Qt::WaitCursor);
	ui.view->target.Clear();
	ui.view->target.Read(file.toStdString().c_str());

	ui.view->target.UpdateCOG();
	ui.view->target.UpdateBoundingSphere();
	double r = ui.view->target.GetBoundingSphereRadius();
	
	std::cout << "the num of vertexes : " << ui.view->target.n_vertices() << std::endl;
	std::cout << "the num of faces : " << ui.view->target.n_faces() << std::endl;
	std::cout << "cog : " << ui.view->target.cog[0] << ", " << ui.view->target.cog[1] << ", " << ui.view->target.cog[2] << std::endl;	

	ui.view->updateGL();
	setCursor(Qt::ArrowCursor);
}

void DeformableRegistration::OnFileSaveTemplate()
{

}

void DeformableRegistration::OnFileSaveTarget()
{

}

void DeformableRegistration::OnFileExit()
{
	close();
}

void DeformableRegistration::OnToolsDecimate()
{
	setCursor(Qt::WaitCursor);
	ui.view->templ.Decimate(0.7);
	setCursor(Qt::ArrowCursor);
	ui.view->updateGL();
}

void DeformableRegistration::OnToolsSample_Random()
{

// 	int NumOfSamples = 50000;
// 	ui.view->templ.SampleRandom(NumOfSamples, );
	
	// Get a Set of vertices from shuffled list (That's nodes!)
// 	QMessageBox msgBox;
// 	msgBox.setText("RANDOM Sampling");
// 	msgBox.exec();

}

void DeformableRegistration::OnToolsSample_Quad()
{
	QMessageBox msgBox;
	msgBox.setText("Quadric Fitting Sampling");
	msgBox.exec();
}

void DeformableRegistration::OnToolsSample_Uniform_Dart()
{
	//ui.view->templ.SampleUniform(2000, TM_SAMPLE_UNIFORM_DART);
}

void DeformableRegistration::OnToolsEmbededDeformation()
{
	ui.view->InitOptimization();
	ui.view->onEmbededDeformation = true;
	ui.view->target = ui.view->templ;
	ui.actionTemplateVisible->setChecked(false);
	ui.actionTargetVisible->setChecked(true);
	ui.view->updateGL();
}

void DeformableRegistration::OnViewDeformationGraph()
{
	ui.view->updateGL();
}

void DeformableRegistration::OnToolsInitHaoLi()
{
// 	double r = ui.view->templ.GetBoundingSphereRadius();
// 	std::cout << r << std::endl;
// 	ui.view->camera()->setPosition(qglviewer::Vec(0,0,0));
// 	ui.view->camera()->lookAt(qglviewer::Vec(0,0,1));	
// 	ui.view->camera()->setUpVector(qglviewer::Vec(0,-1,0));
// 	ui.view->camera()->setSceneRadius(r*0.2);
// 	ui.view->camera()->showEntireScene();
// 	ui.view->updateGL();
	ui.actionTemplateVisible->setCheckable(false);
	ui.view->InitOptimization_HaoLi();
	ui.actionDeformationGraph->setChecked(true);
	ui.view->updateGL();
}

void DeformableRegistration::OnToolsRunHaoLi()
{
	ui.view->RunOptimization_HaoLi();
}

void DeformableRegistration::OnToolsGeodesic()
{
	ui.view->Propagate();
}

void DeformableRegistration::OnInitGeo()
{
	ui.view->InitGeo();
}

void DeformableRegistration::OnUpdateGL()
{
	ui.view->updateGL();
}
