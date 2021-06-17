/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *change_point;
    QPushButton *open_source;
    QPushButton *insert_point;
    QPushButton *delete_point;
    QPushButton *Start;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_5;
    QComboBox *Paraments;
    QLabel *label_4;
    QPushButton *Mode_Add_ONE;
    QPushButton *MODE_ADD_TWO;
    QLabel *label_6;
    QVTKWidget *CloudViewer;
    QPushButton *open_txt;
    QLabel *label_7;
    QCheckBox *deviation_keep;
    QCheckBox *deviation_random;
    QLineEdit *deviation_set_tough;
    QCheckBox *deviation_set;
    QLineEdit *deviation_set_fine;
    QComboBox *Translation_Estimate;
    QComboBox *Correspond;
    QLineEdit *Paraments_Value;
    QLabel *label_8;
    QLabel *Mode_Add_2;
    QPushButton *Reset_Mode;
    QPushButton *save_txt;
    QTextEdit *Selected_Mode_1;
    QTextEdit *Selected_Mode_2;
    QPushButton *add_point;
    QPushButton *select_point;
    QCheckBox *center_rotation;
    QCheckBox *random_transform;
    QLineEdit *xc_angle;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QLineEdit *yc_angle;
    QLineEdit *zc_angle;
    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_14;
    QLineEdit *yr_angle;
    QLineEdit *zr_angle;
    QLineEdit *xr_angle;
    QLabel *label_15;
    QLabel *label_16;
    QLabel *label_17;
    QLineEdit *yr_distance;
    QLineEdit *zr_distance;
    QLineEdit *xr_distance;
    QLabel *label_18;
    QLineEdit *sample_rate;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1857, 1032);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        change_point = new QPushButton(centralWidget);
        change_point->setObjectName(QString::fromUtf8("change_point"));
        change_point->setGeometry(QRect(30, 790, 89, 25));
        open_source = new QPushButton(centralWidget);
        open_source->setObjectName(QString::fromUtf8("open_source"));
        open_source->setGeometry(QRect(580, 740, 89, 25));
        insert_point = new QPushButton(centralWidget);
        insert_point->setObjectName(QString::fromUtf8("insert_point"));
        insert_point->setGeometry(QRect(420, 740, 89, 25));
        delete_point = new QPushButton(centralWidget);
        delete_point->setObjectName(QString::fromUtf8("delete_point"));
        delete_point->setGeometry(QRect(250, 790, 89, 25));
        Start = new QPushButton(centralWidget);
        Start->setObjectName(QString::fromUtf8("Start"));
        Start->setGeometry(QRect(1020, 770, 89, 25));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(1520, 20, 67, 17));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(1460, 70, 181, 31));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(1500, 310, 141, 31));
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(1500, 170, 101, 21));
        Paraments = new QComboBox(centralWidget);
        Paraments->addItem(QString());
        Paraments->addItem(QString());
        Paraments->addItem(QString());
        Paraments->addItem(QString());
        Paraments->addItem(QString());
        Paraments->addItem(QString());
        Paraments->addItem(QString());
        Paraments->addItem(QString());
        Paraments->addItem(QString());
        Paraments->addItem(QString());
        Paraments->addItem(QString());
        Paraments->addItem(QString());
        Paraments->setObjectName(QString::fromUtf8("Paraments"));
        Paraments->setGeometry(QRect(1440, 210, 91, 21));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(1380, 470, 121, 21));
        Mode_Add_ONE = new QPushButton(centralWidget);
        Mode_Add_ONE->setObjectName(QString::fromUtf8("Mode_Add_ONE"));
        Mode_Add_ONE->setGeometry(QRect(1620, 520, 89, 25));
        MODE_ADD_TWO = new QPushButton(centralWidget);
        MODE_ADD_TWO->setObjectName(QString::fromUtf8("MODE_ADD_TWO"));
        MODE_ADD_TWO->setGeometry(QRect(1620, 610, 89, 25));
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(1540, 470, 121, 21));
        CloudViewer = new QVTKWidget(centralWidget);
        CloudViewer->setObjectName(QString::fromUtf8("CloudViewer"));
        CloudViewer->setGeometry(QRect(10, 10, 1281, 701));
        open_txt = new QPushButton(centralWidget);
        open_txt->setObjectName(QString::fromUtf8("open_txt"));
        open_txt->setGeometry(QRect(580, 780, 89, 25));
        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(790, 730, 141, 31));
        deviation_keep = new QCheckBox(centralWidget);
        deviation_keep->setObjectName(QString::fromUtf8("deviation_keep"));
        deviation_keep->setGeometry(QRect(750, 770, 101, 31));
        deviation_random = new QCheckBox(centralWidget);
        deviation_random->setObjectName(QString::fromUtf8("deviation_random"));
        deviation_random->setEnabled(true);
        deviation_random->setGeometry(QRect(840, 770, 101, 31));
        deviation_random->setChecked(true);
        deviation_set_tough = new QLineEdit(centralWidget);
        deviation_set_tough->setObjectName(QString::fromUtf8("deviation_set_tough"));
        deviation_set_tough->setGeometry(QRect(840, 820, 113, 25));
        deviation_set = new QCheckBox(centralWidget);
        deviation_set->setObjectName(QString::fromUtf8("deviation_set"));
        deviation_set->setGeometry(QRect(750, 820, 101, 31));
        deviation_set_fine = new QLineEdit(centralWidget);
        deviation_set_fine->setObjectName(QString::fromUtf8("deviation_set_fine"));
        deviation_set_fine->setGeometry(QRect(840, 860, 113, 25));
        Translation_Estimate = new QComboBox(centralWidget);
        Translation_Estimate->addItem(QString());
        Translation_Estimate->addItem(QString());
        Translation_Estimate->addItem(QString());
        Translation_Estimate->addItem(QString());
        Translation_Estimate->addItem(QString());
        Translation_Estimate->addItem(QString());
        Translation_Estimate->addItem(QString());
        Translation_Estimate->addItem(QString());
        Translation_Estimate->addItem(QString());
        Translation_Estimate->setObjectName(QString::fromUtf8("Translation_Estimate"));
        Translation_Estimate->setGeometry(QRect(1500, 110, 86, 25));
        Correspond = new QComboBox(centralWidget);
        Correspond->addItem(QString());
        Correspond->addItem(QString());
        Correspond->addItem(QString());
        Correspond->addItem(QString());
        Correspond->setObjectName(QString::fromUtf8("Correspond"));
        Correspond->setGeometry(QRect(1460, 360, 151, 25));
        Paraments_Value = new QLineEdit(centralWidget);
        Paraments_Value->setObjectName(QString::fromUtf8("Paraments_Value"));
        Paraments_Value->setGeometry(QRect(1550, 210, 113, 25));
        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(1540, 530, 121, 21));
        Mode_Add_2 = new QLabel(centralWidget);
        Mode_Add_2->setObjectName(QString::fromUtf8("Mode_Add_2"));
        Mode_Add_2->setGeometry(QRect(1540, 610, 121, 21));
        Reset_Mode = new QPushButton(centralWidget);
        Reset_Mode->setObjectName(QString::fromUtf8("Reset_Mode"));
        Reset_Mode->setGeometry(QRect(1620, 10, 89, 25));
        save_txt = new QPushButton(centralWidget);
        save_txt->setObjectName(QString::fromUtf8("save_txt"));
        save_txt->setGeometry(QRect(580, 820, 89, 25));
        Selected_Mode_1 = new QTextEdit(centralWidget);
        Selected_Mode_1->setObjectName(QString::fromUtf8("Selected_Mode_1"));
        Selected_Mode_1->setGeometry(QRect(1380, 510, 104, 91));
        Selected_Mode_2 = new QTextEdit(centralWidget);
        Selected_Mode_2->setObjectName(QString::fromUtf8("Selected_Mode_2"));
        Selected_Mode_2->setGeometry(QRect(1380, 610, 104, 91));
        add_point = new QPushButton(centralWidget);
        add_point->setObjectName(QString::fromUtf8("add_point"));
        add_point->setGeometry(QRect(140, 790, 89, 25));
        select_point = new QPushButton(centralWidget);
        select_point->setObjectName(QString::fromUtf8("select_point"));
        select_point->setGeometry(QRect(140, 740, 89, 25));
        center_rotation = new QCheckBox(centralWidget);
        center_rotation->setObjectName(QString::fromUtf8("center_rotation"));
        center_rotation->setGeometry(QRect(1200, 750, 141, 31));
        random_transform = new QCheckBox(centralWidget);
        random_transform->setObjectName(QString::fromUtf8("random_transform"));
        random_transform->setGeometry(QRect(1200, 840, 151, 31));
        random_transform->setChecked(true);
        xc_angle = new QLineEdit(centralWidget);
        xc_angle->setObjectName(QString::fromUtf8("xc_angle"));
        xc_angle->setGeometry(QRect(1510, 710, 113, 25));
        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(1410, 720, 67, 17));
        label_10 = new QLabel(centralWidget);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(1410, 750, 67, 17));
        label_11 = new QLabel(centralWidget);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(1410, 780, 67, 17));
        yc_angle = new QLineEdit(centralWidget);
        yc_angle->setObjectName(QString::fromUtf8("yc_angle"));
        yc_angle->setGeometry(QRect(1510, 740, 113, 25));
        zc_angle = new QLineEdit(centralWidget);
        zc_angle->setObjectName(QString::fromUtf8("zc_angle"));
        zc_angle->setGeometry(QRect(1510, 770, 113, 25));
        label_12 = new QLabel(centralWidget);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(1400, 840, 67, 17));
        label_13 = new QLabel(centralWidget);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(1400, 810, 67, 17));
        label_14 = new QLabel(centralWidget);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(1400, 870, 67, 17));
        yr_angle = new QLineEdit(centralWidget);
        yr_angle->setObjectName(QString::fromUtf8("yr_angle"));
        yr_angle->setGeometry(QRect(1490, 840, 113, 25));
        zr_angle = new QLineEdit(centralWidget);
        zr_angle->setObjectName(QString::fromUtf8("zr_angle"));
        zr_angle->setGeometry(QRect(1490, 870, 113, 25));
        xr_angle = new QLineEdit(centralWidget);
        xr_angle->setObjectName(QString::fromUtf8("xr_angle"));
        xr_angle->setGeometry(QRect(1490, 810, 113, 25));
        label_15 = new QLabel(centralWidget);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(1650, 840, 67, 17));
        label_16 = new QLabel(centralWidget);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(1650, 810, 67, 17));
        label_17 = new QLabel(centralWidget);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setGeometry(QRect(1650, 870, 67, 17));
        yr_distance = new QLineEdit(centralWidget);
        yr_distance->setObjectName(QString::fromUtf8("yr_distance"));
        yr_distance->setGeometry(QRect(1730, 840, 113, 25));
        zr_distance = new QLineEdit(centralWidget);
        zr_distance->setObjectName(QString::fromUtf8("zr_distance"));
        zr_distance->setGeometry(QRect(1730, 870, 113, 25));
        xr_distance = new QLineEdit(centralWidget);
        xr_distance->setObjectName(QString::fromUtf8("xr_distance"));
        xr_distance->setGeometry(QRect(1730, 810, 113, 25));
        label_18 = new QLabel(centralWidget);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setGeometry(QRect(1210, 930, 91, 17));
        sample_rate = new QLineEdit(centralWidget);
        sample_rate->setObjectName(QString::fromUtf8("sample_rate"));
        sample_rate->setGeometry(QRect(1320, 930, 113, 25));
        MainWindow->setCentralWidget(centralWidget);
        CloudViewer->raise();
        change_point->raise();
        open_source->raise();
        insert_point->raise();
        delete_point->raise();
        Start->raise();
        label->raise();
        label_2->raise();
        label_3->raise();
        label_5->raise();
        Paraments->raise();
        label_4->raise();
        Mode_Add_ONE->raise();
        MODE_ADD_TWO->raise();
        label_6->raise();
        open_txt->raise();
        label_7->raise();
        deviation_keep->raise();
        deviation_random->raise();
        deviation_set_tough->raise();
        deviation_set->raise();
        deviation_set_fine->raise();
        Translation_Estimate->raise();
        Correspond->raise();
        Paraments_Value->raise();
        label_8->raise();
        Mode_Add_2->raise();
        Reset_Mode->raise();
        save_txt->raise();
        Selected_Mode_1->raise();
        Selected_Mode_2->raise();
        add_point->raise();
        select_point->raise();
        center_rotation->raise();
        random_transform->raise();
        xc_angle->raise();
        label_9->raise();
        label_10->raise();
        label_11->raise();
        yc_angle->raise();
        zc_angle->raise();
        label_12->raise();
        label_13->raise();
        label_14->raise();
        yr_angle->raise();
        zr_angle->raise();
        xr_angle->raise();
        label_15->raise();
        label_16->raise();
        label_17->raise();
        yr_distance->raise();
        zr_distance->raise();
        xr_distance->raise();
        label_18->raise();
        sample_rate->raise();
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1857, 28));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        Translation_Estimate->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        change_point->setText(QApplication::translate("MainWindow", "change point", nullptr));
        open_source->setText(QApplication::translate("MainWindow", "open source", nullptr));
        insert_point->setText(QApplication::translate("MainWindow", "insert point", nullptr));
        delete_point->setText(QApplication::translate("MainWindow", "delete point", nullptr));
        Start->setText(QApplication::translate("MainWindow", "start", nullptr));
        label->setText(QApplication::translate("MainWindow", "MODE", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "TransformationEstimate", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "Correspondence", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "Parameter", nullptr));
        Paraments->setItemText(0, QApplication::translate("MainWindow", "NONE", nullptr));
        Paraments->setItemText(1, QApplication::translate("MainWindow", "PNORM", nullptr));
        Paraments->setItemText(2, QApplication::translate("MainWindow", "TUKEY", nullptr));
        Paraments->setItemText(3, QApplication::translate("MainWindow", "HUBER", nullptr));
        Paraments->setItemText(4, QApplication::translate("MainWindow", "MEDIAN", nullptr));
        Paraments->setItemText(5, QApplication::translate("MainWindow", "DIS_TUKEY", nullptr));
        Paraments->setItemText(6, QApplication::translate("MainWindow", "FAIR", nullptr));
        Paraments->setItemText(7, QApplication::translate("MainWindow", "LOGISTIC", nullptr));
        Paraments->setItemText(8, QApplication::translate("MainWindow", "TRIMMED", nullptr));
        Paraments->setItemText(9, QApplication::translate("MainWindow", "DIS_PNORM", nullptr));
        Paraments->setItemText(10, QApplication::translate("MainWindow", "TANH", nullptr));
        Paraments->setItemText(11, QApplication::translate("MainWindow", "SET", nullptr));

        label_4->setText(QApplication::translate("MainWindow", "Selected Mode", nullptr));
        Mode_Add_ONE->setText(QApplication::translate("MainWindow", "\346\267\273\345\212\240/\344\277\256\346\224\271", nullptr));
        MODE_ADD_TWO->setText(QApplication::translate("MainWindow", "\346\267\273\345\212\240/\344\277\256\346\224\271", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "Order", nullptr));
        open_txt->setText(QApplication::translate("MainWindow", "open txt", nullptr));
        label_7->setText(QApplication::translate("MainWindow", "Set deviation", nullptr));
        deviation_keep->setText(QApplication::translate("MainWindow", "keep", nullptr));
        deviation_random->setText(QApplication::translate("MainWindow", "random", nullptr));
        deviation_set->setText(QApplication::translate("MainWindow", "set", nullptr));
        Translation_Estimate->setItemText(0, QApplication::translate("MainWindow", "NONE", nullptr));
        Translation_Estimate->setItemText(1, QApplication::translate("MainWindow", "ICP", nullptr));
        Translation_Estimate->setItemText(2, QApplication::translate("MainWindow", "PTP", nullptr));
        Translation_Estimate->setItemText(3, QApplication::translate("MainWindow", "DSPR", nullptr));
        Translation_Estimate->setItemText(4, QApplication::translate("MainWindow", "SAICP", nullptr));
        Translation_Estimate->setItemText(5, QApplication::translate("MainWindow", "PSO", nullptr));
        Translation_Estimate->setItemText(6, QApplication::translate("MainWindow", "PTL", nullptr));
        Translation_Estimate->setItemText(7, QApplication::translate("MainWindow", "Combin", nullptr));
        Translation_Estimate->setItemText(8, QApplication::translate("MainWindow", "RAICP", nullptr));

        Correspond->setItemText(0, QApplication::translate("MainWindow", "NONE", nullptr));
        Correspond->setItemText(1, QApplication::translate("MainWindow", "Trandition", nullptr));
        Correspond->setItemText(2, QApplication::translate("MainWindow", "Direction Constraint", nullptr));
        Correspond->setItemText(3, QApplication::translate("MainWindow", "Fit Plane", nullptr));

        label_8->setText(QApplication::translate("MainWindow", "Left", nullptr));
        Mode_Add_2->setText(QApplication::translate("MainWindow", "Right", nullptr));
        Reset_Mode->setText(QApplication::translate("MainWindow", "\351\207\215\347\275\256", nullptr));
        save_txt->setText(QApplication::translate("MainWindow", "save txt", nullptr));
        Selected_Mode_1->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">ICP</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">MEDIAN</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">0</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Trandition</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /"
                        "></p></body></html>", nullptr));
        Selected_Mode_2->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">ICP</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">HUBER</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">0.75</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Trandition</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br"
                        " /></p></body></html>", nullptr));
        add_point->setText(QApplication::translate("MainWindow", "add point", nullptr));
        select_point->setText(QApplication::translate("MainWindow", "select point", nullptr));
        center_rotation->setText(QApplication::translate("MainWindow", "center rotation", nullptr));
        random_transform->setText(QApplication::translate("MainWindow", "random transform", nullptr));
        xc_angle->setText(QApplication::translate("MainWindow", "0.0", nullptr));
        label_9->setText(QApplication::translate("MainWindow", "x\350\275\264\350\247\222\345\272\246", nullptr));
        label_10->setText(QApplication::translate("MainWindow", "y\350\275\264\350\247\222\345\272\246", nullptr));
        label_11->setText(QApplication::translate("MainWindow", "z\350\275\264\350\247\222\345\272\246", nullptr));
        yc_angle->setText(QApplication::translate("MainWindow", "0.0", nullptr));
        zc_angle->setText(QApplication::translate("MainWindow", "40.0", nullptr));
        label_12->setText(QApplication::translate("MainWindow", "y\350\275\264\350\247\222\345\272\246", nullptr));
        label_13->setText(QApplication::translate("MainWindow", "x\350\275\264\350\247\222\345\272\246", nullptr));
        label_14->setText(QApplication::translate("MainWindow", "z\350\275\264\350\247\222\345\272\246", nullptr));
        yr_angle->setText(QApplication::translate("MainWindow", "60.0", nullptr));
        zr_angle->setText(QApplication::translate("MainWindow", "90.0", nullptr));
        xr_angle->setText(QApplication::translate("MainWindow", "30.0", nullptr));
        label_15->setText(QApplication::translate("MainWindow", "y\350\275\264\345\271\263\347\247\273", nullptr));
        label_16->setText(QApplication::translate("MainWindow", "x\350\275\264\345\271\263\347\247\273", nullptr));
        label_17->setText(QApplication::translate("MainWindow", "z\350\275\264\345\271\263\347\247\273", nullptr));
        yr_distance->setText(QApplication::translate("MainWindow", "4.0", nullptr));
        zr_distance->setText(QApplication::translate("MainWindow", "6.0", nullptr));
        xr_distance->setText(QApplication::translate("MainWindow", "2.0", nullptr));
        label_18->setText(QApplication::translate("MainWindow", "sample rate", nullptr));
        sample_rate->setText(QApplication::translate("MainWindow", "1.0", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
