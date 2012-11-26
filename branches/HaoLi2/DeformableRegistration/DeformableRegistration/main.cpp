#include "DeformableRegistration.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	DeformableRegistration w;
	w.show();
	return a.exec();
}
