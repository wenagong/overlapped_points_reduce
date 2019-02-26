#include "stdafx.h"
#include "overlapped_points_reduce.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	overlapped_points_reduce w;
	w.show();
	return a.exec();
}
