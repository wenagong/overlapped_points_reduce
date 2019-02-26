#pragma once

#include <QtWidgets/QDialog>
#include "ui_overlapped_points_reduce.h"

class overlapped_points_reduce : public QDialog
{
	Q_OBJECT

public:
	overlapped_points_reduce(QWidget *parent = Q_NULLPTR);

private:
	Ui::overlapped_points_reduceClass ui;

public slots:
    void OnGetPcd();
	void CalculateOverlap();
	void GetFinalResult();
	void GetWholeDatas();
	void RemoveBackgroundDatas();
	void FilterDenoise();
};
