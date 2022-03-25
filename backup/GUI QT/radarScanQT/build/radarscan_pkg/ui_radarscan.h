/********************************************************************************
** Form generated from reading UI file 'radarscan.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RADARSCAN_H
#define UI_RADARSCAN_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_radarScan
{
public:
    QGraphicsView *graphicsView_radarScan;
    QPushButton *btnFindxy;
    QPushButton *btnAdd;
    QPushButton *btnClr;

    void setupUi(QWidget *radarScan)
    {
        if (radarScan->objectName().isEmpty())
            radarScan->setObjectName(QStringLiteral("radarScan"));
        radarScan->resize(929, 628);
        radarScan->setStyleSheet(QStringLiteral("background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(43, 71, 106, 255), stop:1 rgba(67, 53, 91, 255));"));
        graphicsView_radarScan = new QGraphicsView(radarScan);
        graphicsView_radarScan->setObjectName(QStringLiteral("graphicsView_radarScan"));
        graphicsView_radarScan->setGeometry(QRect(150, 30, 225, 460));
        graphicsView_radarScan->setStyleSheet(QLatin1String("border-radius: 5px;\n"
"background-color: rgba(255, 255, 255, 0);\n"
"border-radius: 10px;\n"
"border: 5px solid rgb(75, 85, 117);\n"
""));
        btnFindxy = new QPushButton(radarScan);
        btnFindxy->setObjectName(QStringLiteral("btnFindxy"));
        btnFindxy->setGeometry(QRect(420, 332, 152, 25));
        btnAdd = new QPushButton(radarScan);
        btnAdd->setObjectName(QStringLiteral("btnAdd"));
        btnAdd->setGeometry(QRect(420, 396, 152, 25));
        btnClr = new QPushButton(radarScan);
        btnClr->setObjectName(QStringLiteral("btnClr"));
        btnClr->setGeometry(QRect(420, 460, 152, 25));

        retranslateUi(radarScan);

        QMetaObject::connectSlotsByName(radarScan);
    } // setupUi

    void retranslateUi(QWidget *radarScan)
    {
        radarScan->setWindowTitle(QApplication::translate("radarScan", "Form", Q_NULLPTR));
        btnFindxy->setText(QApplication::translate("radarScan", "find", Q_NULLPTR));
        btnAdd->setText(QApplication::translate("radarScan", "add", Q_NULLPTR));
        btnClr->setText(QApplication::translate("radarScan", "clear", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class radarScan: public Ui_radarScan {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RADARSCAN_H
