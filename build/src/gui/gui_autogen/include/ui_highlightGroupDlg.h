/********************************************************************************
** Form generated from reading UI file 'highlightGroupDlg.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_HIGHLIGHTGROUPDLG_H
#define UI_HIGHLIGHTGROUPDLG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_HighlightGroupDlg
{
public:
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QRadioButton *grp3RadioButton;
    QRadioButton *grp5RadioButton;
    QRadioButton *grp4RadioButton;
    QRadioButton *grp6RadioButton;
    QRadioButton *grp1RadioButton;
    QRadioButton *grp8RadioButton;
    QRadioButton *grp2RadioButton;
    QRadioButton *grp7RadioButton;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QDialogButtonBox *buttonBox;
    QSpacerItem *horizontalSpacer_2;

    void setupUi(QDialog *HighlightGroupDlg)
    {
        if (HighlightGroupDlg->objectName().isEmpty())
            HighlightGroupDlg->setObjectName(QString::fromUtf8("HighlightGroupDlg"));
        HighlightGroupDlg->resize(230, 230);
        HighlightGroupDlg->setMinimumSize(QSize(230, 230));
        gridLayout = new QGridLayout(HighlightGroupDlg);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        groupBox = new QGroupBox(HighlightGroupDlg);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        grp3RadioButton = new QRadioButton(groupBox);
        grp3RadioButton->setObjectName(QString::fromUtf8("grp3RadioButton"));

        gridLayout_2->addWidget(grp3RadioButton, 2, 0, 1, 1);

        grp5RadioButton = new QRadioButton(groupBox);
        grp5RadioButton->setObjectName(QString::fromUtf8("grp5RadioButton"));

        gridLayout_2->addWidget(grp5RadioButton, 0, 1, 1, 1);

        grp4RadioButton = new QRadioButton(groupBox);
        grp4RadioButton->setObjectName(QString::fromUtf8("grp4RadioButton"));

        gridLayout_2->addWidget(grp4RadioButton, 3, 0, 1, 1);

        grp6RadioButton = new QRadioButton(groupBox);
        grp6RadioButton->setObjectName(QString::fromUtf8("grp6RadioButton"));

        gridLayout_2->addWidget(grp6RadioButton, 1, 1, 1, 1);

        grp1RadioButton = new QRadioButton(groupBox);
        grp1RadioButton->setObjectName(QString::fromUtf8("grp1RadioButton"));

        gridLayout_2->addWidget(grp1RadioButton, 0, 0, 1, 1);

        grp8RadioButton = new QRadioButton(groupBox);
        grp8RadioButton->setObjectName(QString::fromUtf8("grp8RadioButton"));

        gridLayout_2->addWidget(grp8RadioButton, 3, 1, 1, 1);

        grp2RadioButton = new QRadioButton(groupBox);
        grp2RadioButton->setObjectName(QString::fromUtf8("grp2RadioButton"));

        gridLayout_2->addWidget(grp2RadioButton, 1, 0, 1, 1);

        grp7RadioButton = new QRadioButton(groupBox);
        grp7RadioButton->setObjectName(QString::fromUtf8("grp7RadioButton"));

        gridLayout_2->addWidget(grp7RadioButton, 2, 1, 1, 1);


        verticalLayout->addWidget(groupBox);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        buttonBox = new QDialogButtonBox(HighlightGroupDlg);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(buttonBox->sizePolicy().hasHeightForWidth());
        buttonBox->setSizePolicy(sizePolicy);
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Ok);

        horizontalLayout->addWidget(buttonBox);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);


        verticalLayout->addLayout(horizontalLayout);


        gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);


        retranslateUi(HighlightGroupDlg);
        QObject::connect(buttonBox, SIGNAL(accepted()), HighlightGroupDlg, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), HighlightGroupDlg, SLOT(reject()));

        QMetaObject::connectSlotsByName(HighlightGroupDlg);
    } // setupUi

    void retranslateUi(QDialog *HighlightGroupDlg)
    {
        HighlightGroupDlg->setWindowTitle(QCoreApplication::translate("HighlightGroupDlg", "Highlight Group", nullptr));
        groupBox->setTitle(QCoreApplication::translate("HighlightGroupDlg", "Choose Highlight Group :", nullptr));
        grp3RadioButton->setText(QCoreApplication::translate("HighlightGroupDlg", "Group3", nullptr));
        grp5RadioButton->setText(QCoreApplication::translate("HighlightGroupDlg", "Group 5", nullptr));
        grp4RadioButton->setText(QCoreApplication::translate("HighlightGroupDlg", "Group 4", nullptr));
        grp6RadioButton->setText(QCoreApplication::translate("HighlightGroupDlg", "Group 6", nullptr));
        grp1RadioButton->setText(QCoreApplication::translate("HighlightGroupDlg", "Group 1", nullptr));
        grp8RadioButton->setText(QCoreApplication::translate("HighlightGroupDlg", "Group 8", nullptr));
        grp2RadioButton->setText(QCoreApplication::translate("HighlightGroupDlg", "Group 2", nullptr));
        grp7RadioButton->setText(QCoreApplication::translate("HighlightGroupDlg", "Group 7", nullptr));
    } // retranslateUi

};

namespace Ui {
    class HighlightGroupDlg: public Ui_HighlightGroupDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_HIGHLIGHTGROUPDLG_H
