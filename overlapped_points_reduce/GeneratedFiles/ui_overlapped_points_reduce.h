/********************************************************************************
** Form generated from reading UI file 'overlapped_points_reduce.ui'
**
** Created by: Qt User Interface Compiler version 5.9.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OVERLAPPED_POINTS_REDUCE_H
#define UI_OVERLAPPED_POINTS_REDUCE_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_overlapped_points_reduceClass
{
public:
    QPushButton *pushButton_1;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
    QPushButton *pushButton_5;
    QPushButton *pushButton_6;

    void setupUi(QDialog *overlapped_points_reduceClass)
    {
        if (overlapped_points_reduceClass->objectName().isEmpty())
            overlapped_points_reduceClass->setObjectName(QStringLiteral("overlapped_points_reduceClass"));
        overlapped_points_reduceClass->resize(600, 400);
        pushButton_1 = new QPushButton(overlapped_points_reduceClass);
        pushButton_1->setObjectName(QStringLiteral("pushButton_1"));
        pushButton_1->setGeometry(QRect(150, 80, 91, 31));
        pushButton_2 = new QPushButton(overlapped_points_reduceClass);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        pushButton_2->setGeometry(QRect(150, 140, 91, 31));
        pushButton_3 = new QPushButton(overlapped_points_reduceClass);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));
        pushButton_3->setGeometry(QRect(150, 200, 91, 31));
        pushButton_4 = new QPushButton(overlapped_points_reduceClass);
        pushButton_4->setObjectName(QStringLiteral("pushButton_4"));
        pushButton_4->setGeometry(QRect(310, 80, 91, 31));
        pushButton_5 = new QPushButton(overlapped_points_reduceClass);
        pushButton_5->setObjectName(QStringLiteral("pushButton_5"));
        pushButton_5->setGeometry(QRect(310, 140, 91, 31));
        pushButton_6 = new QPushButton(overlapped_points_reduceClass);
        pushButton_6->setObjectName(QStringLiteral("pushButton_6"));
        pushButton_6->setGeometry(QRect(310, 200, 91, 31));

        retranslateUi(overlapped_points_reduceClass);
        QObject::connect(pushButton_1, SIGNAL(clicked()), overlapped_points_reduceClass, SLOT(OnGetPcd()));
        QObject::connect(pushButton_2, SIGNAL(clicked()), overlapped_points_reduceClass, SLOT(CalculateOverlap()));
        QObject::connect(pushButton_3, SIGNAL(clicked()), overlapped_points_reduceClass, SLOT(GetFinalResult()));
        QObject::connect(pushButton_4, SIGNAL(clicked()), overlapped_points_reduceClass, SLOT(GetWholeDatas()));
        QObject::connect(pushButton_5, SIGNAL(clicked()), overlapped_points_reduceClass, SLOT(RemoveBackgroundDatas()));
        QObject::connect(pushButton_6, SIGNAL(clicked()), overlapped_points_reduceClass, SLOT(FilterDenoise()));

        QMetaObject::connectSlotsByName(overlapped_points_reduceClass);
    } // setupUi

    void retranslateUi(QDialog *overlapped_points_reduceClass)
    {
        overlapped_points_reduceClass->setWindowTitle(QApplication::translate("overlapped_points_reduceClass", "overlapped_points_reduce", Q_NULLPTR));
        pushButton_1->setText(QApplication::translate("overlapped_points_reduceClass", "PCD\346\240\274\345\274\217\350\275\254\346\215\242", Q_NULLPTR));
        pushButton_2->setText(QApplication::translate("overlapped_points_reduceClass", "\350\256\241\347\256\227\351\207\215\345\217\240\347\202\271\344\272\221", Q_NULLPTR));
        pushButton_3->setText(QApplication::translate("overlapped_points_reduceClass", "\351\207\215\345\217\240\347\202\271\344\272\221\346\266\210\345\206\227", Q_NULLPTR));
        pushButton_4->setText(QApplication::translate("overlapped_points_reduceClass", "\345\220\210\346\210\220\345\256\214\346\225\264\347\202\271\344\272\221", Q_NULLPTR));
        pushButton_5->setText(QApplication::translate("overlapped_points_reduceClass", "\345\216\273\351\231\244\350\203\214\346\231\257\347\202\271\344\272\221", Q_NULLPTR));
        pushButton_6->setText(QApplication::translate("overlapped_points_reduceClass", "\347\202\271\344\272\221\346\273\244\346\263\242\345\216\273\345\231\252", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class overlapped_points_reduceClass: public Ui_overlapped_points_reduceClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OVERLAPPED_POINTS_REDUCE_H
