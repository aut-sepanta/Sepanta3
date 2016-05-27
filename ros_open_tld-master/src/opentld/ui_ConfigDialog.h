/********************************************************************************
** Form generated from reading UI file 'ConfigDialog.ui'
**
** Created: Wed Mar 26 18:43:25 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONFIGDIALOG_H
#define UI_CONFIGDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QScrollArea>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ConfigDialog
{
public:
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget;
    QWidget *tab_general;
    QHBoxLayout *horizontalLayout_3;
    QScrollArea *scrollArea_general;
    QWidget *scrollAreaWidgetContents_general;
    QGridLayout *gridLayout_3;
    QGroupBox *groupBox_loadModel;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_modelPath;
    QLineEdit *lineEdit_modelPath;
    QPushButton *pushButton_modelPath;
    QGroupBox *groupBox_initialBoundingBox;
    QGridLayout *gridLayout_4;
    QLabel *label_initialBoundingBoxX;
    QLabel *label_initialBoundingBoxY;
    QLabel *label_initialBoundingBoxW;
    QLabel *label_initialBoundingBoxH;
    QLineEdit *lineEdit_initialBoundingBoxX;
    QLineEdit *lineEdit_initialBoundingBoxY;
    QLineEdit *lineEdit_initialBoundingBoxW;
    QLineEdit *lineEdit_initialBoundingBoxH;
    QLabel *label_theta;
    QLineEdit *lineEdit_theta;
    QGroupBox *groupBox_trajectory;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_trajectory;
    QLineEdit *lineEdit_trajectory;
    QCheckBox *checkBox_learning;
    QCheckBox *checkBox_selectManually;
    QCheckBox *checkBox_showOutput;
    QCheckBox *checkBox_showNotConfident;
    QCheckBox *checkBox_showColorImage;
    QCheckBox *checkBox_showDetections;
    QCheckBox *checkBox_showForeground;
    QGroupBox *groupBox_saveOutput;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_outputDir;
    QLineEdit *lineEdit_outputDir;
    QPushButton *pushButton_outputDir;
    QGroupBox *groupBox_printResults;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_printResults;
    QLineEdit *lineEdit_printResults;
    QPushButton *pushButton_printResults;
    QGroupBox *groupBox_printTiming;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_printTiming;
    QLineEdit *lineEdit_printTiming;
    QPushButton *pushButton_printTiming;
    QCheckBox *checkBox_alternating;
    QCheckBox *checkBox_exportModel;
    QWidget *tab_aquisition;
    QHBoxLayout *horizontalLayout_4;
    QScrollArea *scrollArea_aquisition;
    QWidget *scrollAreaWidgetContents_aquisition;
    QGridLayout *gridLayout_6;
    QPushButton *pushButton_imagePath;
    QLabel *label_method;
    QLabel *label_imagePath;
    QComboBox *comboBox_method;
    QLineEdit *lineEdit_imagePath;
    QGroupBox *groupBox_lastFrame;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_lastFrame;
    QLineEdit *lineEdit_lastFrame;
    QLabel *label_startFrame;
    QLineEdit *lineEdit_startFrame;
    QWidget *tab_detector;
    QHBoxLayout *horizontalLayout_9;
    QScrollArea *scrollArea_detector;
    QWidget *scrollAreaWidgetContents_detector;
    QGridLayout *gridLayout;
    QGroupBox *groupBox_proportionalShift;
    QHBoxLayout *horizontalLayout;
    QLabel *label_proportionalShift;
    QLineEdit *lineEdit_proportionalShift;
    QLabel *label_minScale;
    QLineEdit *lineEdit_minScale;
    QLabel *label_maxScale;
    QLineEdit *lineEdit_maxScale;
    QLabel *label_numberOfFeatures;
    QLineEdit *lineEdit_numberOfFeatures;
    QLabel *label_numberOfTrees;
    QLineEdit *lineEdit_numberOfTrees;
    QLabel *label_minSize;
    QLineEdit *lineEdit_minSize;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *QOpenTLD)
    {
        if (QOpenTLD->objectName().isEmpty())
            QOpenTLD->setObjectName(QString::fromUtf8("QOpenTLD"));
        QOpenTLD->resize(442, 403);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/res/img/opentld.png"), QSize(), QIcon::Normal, QIcon::Off);
        QOpenTLD->setWindowIcon(icon);
        QOpenTLD->setSizeGripEnabled(false);
        verticalLayout = new QVBoxLayout(QOpenTLD);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        tabWidget = new QTabWidget(QOpenTLD);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab_general = new QWidget();
        tab_general->setObjectName(QString::fromUtf8("tab_general"));
        horizontalLayout_3 = new QHBoxLayout(tab_general);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        scrollArea_general = new QScrollArea(tab_general);
        scrollArea_general->setObjectName(QString::fromUtf8("scrollArea_general"));
        scrollArea_general->setWidgetResizable(true);
        scrollAreaWidgetContents_general = new QWidget();
        scrollAreaWidgetContents_general->setObjectName(QString::fromUtf8("scrollAreaWidgetContents_general"));
        scrollAreaWidgetContents_general->setGeometry(QRect(0, 0, 384, 810));
        gridLayout_3 = new QGridLayout(scrollAreaWidgetContents_general);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        groupBox_loadModel = new QGroupBox(scrollAreaWidgetContents_general);
        groupBox_loadModel->setObjectName(QString::fromUtf8("groupBox_loadModel"));
        groupBox_loadModel->setCheckable(true);
        groupBox_loadModel->setChecked(false);
        horizontalLayout_2 = new QHBoxLayout(groupBox_loadModel);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_modelPath = new QLabel(groupBox_loadModel);
        label_modelPath->setObjectName(QString::fromUtf8("label_modelPath"));

        horizontalLayout_2->addWidget(label_modelPath);

        lineEdit_modelPath = new QLineEdit(groupBox_loadModel);
        lineEdit_modelPath->setObjectName(QString::fromUtf8("lineEdit_modelPath"));

        horizontalLayout_2->addWidget(lineEdit_modelPath);

        pushButton_modelPath = new QPushButton(groupBox_loadModel);
        pushButton_modelPath->setObjectName(QString::fromUtf8("pushButton_modelPath"));

        horizontalLayout_2->addWidget(pushButton_modelPath);


        gridLayout_3->addWidget(groupBox_loadModel, 0, 0, 1, 2);

        groupBox_initialBoundingBox = new QGroupBox(scrollAreaWidgetContents_general);
        groupBox_initialBoundingBox->setObjectName(QString::fromUtf8("groupBox_initialBoundingBox"));
        groupBox_initialBoundingBox->setCheckable(true);
        groupBox_initialBoundingBox->setChecked(false);
        gridLayout_4 = new QGridLayout(groupBox_initialBoundingBox);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        label_initialBoundingBoxX = new QLabel(groupBox_initialBoundingBox);
        label_initialBoundingBoxX->setObjectName(QString::fromUtf8("label_initialBoundingBoxX"));

        gridLayout_4->addWidget(label_initialBoundingBoxX, 0, 0, 1, 1);

        label_initialBoundingBoxY = new QLabel(groupBox_initialBoundingBox);
        label_initialBoundingBoxY->setObjectName(QString::fromUtf8("label_initialBoundingBoxY"));

        gridLayout_4->addWidget(label_initialBoundingBoxY, 1, 0, 1, 1);

        label_initialBoundingBoxW = new QLabel(groupBox_initialBoundingBox);
        label_initialBoundingBoxW->setObjectName(QString::fromUtf8("label_initialBoundingBoxW"));

        gridLayout_4->addWidget(label_initialBoundingBoxW, 2, 0, 1, 1);

        label_initialBoundingBoxH = new QLabel(groupBox_initialBoundingBox);
        label_initialBoundingBoxH->setObjectName(QString::fromUtf8("label_initialBoundingBoxH"));

        gridLayout_4->addWidget(label_initialBoundingBoxH, 3, 0, 1, 1);

        lineEdit_initialBoundingBoxX = new QLineEdit(groupBox_initialBoundingBox);
        lineEdit_initialBoundingBoxX->setObjectName(QString::fromUtf8("lineEdit_initialBoundingBoxX"));

        gridLayout_4->addWidget(lineEdit_initialBoundingBoxX, 0, 1, 1, 1);

        lineEdit_initialBoundingBoxY = new QLineEdit(groupBox_initialBoundingBox);
        lineEdit_initialBoundingBoxY->setObjectName(QString::fromUtf8("lineEdit_initialBoundingBoxY"));

        gridLayout_4->addWidget(lineEdit_initialBoundingBoxY, 1, 1, 1, 1);

        lineEdit_initialBoundingBoxW = new QLineEdit(groupBox_initialBoundingBox);
        lineEdit_initialBoundingBoxW->setObjectName(QString::fromUtf8("lineEdit_initialBoundingBoxW"));

        gridLayout_4->addWidget(lineEdit_initialBoundingBoxW, 2, 1, 1, 1);

        lineEdit_initialBoundingBoxH = new QLineEdit(groupBox_initialBoundingBox);
        lineEdit_initialBoundingBoxH->setObjectName(QString::fromUtf8("lineEdit_initialBoundingBoxH"));

        gridLayout_4->addWidget(lineEdit_initialBoundingBoxH, 3, 1, 1, 1);


        gridLayout_3->addWidget(groupBox_initialBoundingBox, 1, 0, 1, 2);

        label_theta = new QLabel(scrollAreaWidgetContents_general);
        label_theta->setObjectName(QString::fromUtf8("label_theta"));

        gridLayout_3->addWidget(label_theta, 7, 0, 1, 1);

        lineEdit_theta = new QLineEdit(scrollAreaWidgetContents_general);
        lineEdit_theta->setObjectName(QString::fromUtf8("lineEdit_theta"));

        gridLayout_3->addWidget(lineEdit_theta, 7, 1, 1, 1);

        groupBox_trajectory = new QGroupBox(scrollAreaWidgetContents_general);
        groupBox_trajectory->setObjectName(QString::fromUtf8("groupBox_trajectory"));
        groupBox_trajectory->setCheckable(true);
        horizontalLayout_5 = new QHBoxLayout(groupBox_trajectory);
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_trajectory = new QLabel(groupBox_trajectory);
        label_trajectory->setObjectName(QString::fromUtf8("label_trajectory"));

        horizontalLayout_5->addWidget(label_trajectory);

        lineEdit_trajectory = new QLineEdit(groupBox_trajectory);
        lineEdit_trajectory->setObjectName(QString::fromUtf8("lineEdit_trajectory"));

        horizontalLayout_5->addWidget(lineEdit_trajectory);


        gridLayout_3->addWidget(groupBox_trajectory, 3, 0, 1, 2);

        checkBox_learning = new QCheckBox(scrollAreaWidgetContents_general);
        checkBox_learning->setObjectName(QString::fromUtf8("checkBox_learning"));
        checkBox_learning->setChecked(true);

        gridLayout_3->addWidget(checkBox_learning, 9, 0, 1, 2);

        checkBox_selectManually = new QCheckBox(scrollAreaWidgetContents_general);
        checkBox_selectManually->setObjectName(QString::fromUtf8("checkBox_selectManually"));

        gridLayout_3->addWidget(checkBox_selectManually, 8, 0, 1, 2);

        checkBox_showOutput = new QCheckBox(scrollAreaWidgetContents_general);
        checkBox_showOutput->setObjectName(QString::fromUtf8("checkBox_showOutput"));
        checkBox_showOutput->setChecked(true);

        gridLayout_3->addWidget(checkBox_showOutput, 10, 0, 1, 2);

        checkBox_showNotConfident = new QCheckBox(scrollAreaWidgetContents_general);
        checkBox_showNotConfident->setObjectName(QString::fromUtf8("checkBox_showNotConfident"));
        checkBox_showNotConfident->setChecked(true);

        gridLayout_3->addWidget(checkBox_showNotConfident, 11, 0, 1, 2);

        checkBox_showColorImage = new QCheckBox(scrollAreaWidgetContents_general);
        checkBox_showColorImage->setObjectName(QString::fromUtf8("checkBox_showColorImage"));

        gridLayout_3->addWidget(checkBox_showColorImage, 12, 0, 1, 2);

        checkBox_showDetections = new QCheckBox(scrollAreaWidgetContents_general);
        checkBox_showDetections->setObjectName(QString::fromUtf8("checkBox_showDetections"));

        gridLayout_3->addWidget(checkBox_showDetections, 13, 0, 1, 2);

        checkBox_showForeground = new QCheckBox(scrollAreaWidgetContents_general);
        checkBox_showForeground->setObjectName(QString::fromUtf8("checkBox_showForeground"));

        gridLayout_3->addWidget(checkBox_showForeground, 14, 0, 1, 2);

        groupBox_saveOutput = new QGroupBox(scrollAreaWidgetContents_general);
        groupBox_saveOutput->setObjectName(QString::fromUtf8("groupBox_saveOutput"));
        groupBox_saveOutput->setCheckable(true);
        groupBox_saveOutput->setChecked(false);
        horizontalLayout_6 = new QHBoxLayout(groupBox_saveOutput);
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_outputDir = new QLabel(groupBox_saveOutput);
        label_outputDir->setObjectName(QString::fromUtf8("label_outputDir"));

        horizontalLayout_6->addWidget(label_outputDir);

        lineEdit_outputDir = new QLineEdit(groupBox_saveOutput);
        lineEdit_outputDir->setObjectName(QString::fromUtf8("lineEdit_outputDir"));

        horizontalLayout_6->addWidget(lineEdit_outputDir);

        pushButton_outputDir = new QPushButton(groupBox_saveOutput);
        pushButton_outputDir->setObjectName(QString::fromUtf8("pushButton_outputDir"));

        horizontalLayout_6->addWidget(pushButton_outputDir);


        gridLayout_3->addWidget(groupBox_saveOutput, 4, 0, 1, 2);

        groupBox_printResults = new QGroupBox(scrollAreaWidgetContents_general);
        groupBox_printResults->setObjectName(QString::fromUtf8("groupBox_printResults"));
        groupBox_printResults->setCheckable(true);
        groupBox_printResults->setChecked(false);
        horizontalLayout_7 = new QHBoxLayout(groupBox_printResults);
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        label_printResults = new QLabel(groupBox_printResults);
        label_printResults->setObjectName(QString::fromUtf8("label_printResults"));

        horizontalLayout_7->addWidget(label_printResults);

        lineEdit_printResults = new QLineEdit(groupBox_printResults);
        lineEdit_printResults->setObjectName(QString::fromUtf8("lineEdit_printResults"));

        horizontalLayout_7->addWidget(lineEdit_printResults);

        pushButton_printResults = new QPushButton(groupBox_printResults);
        pushButton_printResults->setObjectName(QString::fromUtf8("pushButton_printResults"));

        horizontalLayout_7->addWidget(pushButton_printResults);


        gridLayout_3->addWidget(groupBox_printResults, 2, 0, 1, 2);

        groupBox_printTiming = new QGroupBox(scrollAreaWidgetContents_general);
        groupBox_printTiming->setObjectName(QString::fromUtf8("groupBox_printTiming"));
        groupBox_printTiming->setCheckable(true);
        groupBox_printTiming->setChecked(false);
        horizontalLayout_8 = new QHBoxLayout(groupBox_printTiming);
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        label_printTiming = new QLabel(groupBox_printTiming);
        label_printTiming->setObjectName(QString::fromUtf8("label_printTiming"));

        horizontalLayout_8->addWidget(label_printTiming);

        lineEdit_printTiming = new QLineEdit(groupBox_printTiming);
        lineEdit_printTiming->setObjectName(QString::fromUtf8("lineEdit_printTiming"));

        horizontalLayout_8->addWidget(lineEdit_printTiming);

        pushButton_printTiming = new QPushButton(groupBox_printTiming);
        pushButton_printTiming->setObjectName(QString::fromUtf8("pushButton_printTiming"));

        horizontalLayout_8->addWidget(pushButton_printTiming);


        gridLayout_3->addWidget(groupBox_printTiming, 6, 0, 1, 2);

        checkBox_alternating = new QCheckBox(scrollAreaWidgetContents_general);
        checkBox_alternating->setObjectName(QString::fromUtf8("checkBox_alternating"));

        gridLayout_3->addWidget(checkBox_alternating, 15, 0, 1, 2);

        checkBox_exportModel = new QCheckBox(scrollAreaWidgetContents_general);
        checkBox_exportModel->setObjectName(QString::fromUtf8("checkBox_exportModel"));

        gridLayout_3->addWidget(checkBox_exportModel, 16, 0, 1, 2);

        scrollArea_general->setWidget(scrollAreaWidgetContents_general);

        horizontalLayout_3->addWidget(scrollArea_general);

        tabWidget->addTab(tab_general, QString());
        tab_aquisition = new QWidget();
        tab_aquisition->setObjectName(QString::fromUtf8("tab_aquisition"));
        tab_aquisition->setEnabled(true);
        horizontalLayout_4 = new QHBoxLayout(tab_aquisition);
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        scrollArea_aquisition = new QScrollArea(tab_aquisition);
        scrollArea_aquisition->setObjectName(QString::fromUtf8("scrollArea_aquisition"));
        scrollArea_aquisition->setWidgetResizable(true);
        scrollAreaWidgetContents_aquisition = new QWidget();
        scrollAreaWidgetContents_aquisition->setObjectName(QString::fromUtf8("scrollAreaWidgetContents_aquisition"));
        scrollAreaWidgetContents_aquisition->setGeometry(QRect(0, 0, 400, 299));
        gridLayout_6 = new QGridLayout(scrollAreaWidgetContents_aquisition);
        gridLayout_6->setSpacing(6);
        gridLayout_6->setContentsMargins(11, 11, 11, 11);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        pushButton_imagePath = new QPushButton(scrollAreaWidgetContents_aquisition);
        pushButton_imagePath->setObjectName(QString::fromUtf8("pushButton_imagePath"));
        pushButton_imagePath->setEnabled(false);

        gridLayout_6->addWidget(pushButton_imagePath, 1, 2, 1, 1);

        label_method = new QLabel(scrollAreaWidgetContents_aquisition);
        label_method->setObjectName(QString::fromUtf8("label_method"));

        gridLayout_6->addWidget(label_method, 0, 0, 1, 1);

        label_imagePath = new QLabel(scrollAreaWidgetContents_aquisition);
        label_imagePath->setObjectName(QString::fromUtf8("label_imagePath"));
        label_imagePath->setEnabled(false);

        gridLayout_6->addWidget(label_imagePath, 1, 0, 1, 1);

        comboBox_method = new QComboBox(scrollAreaWidgetContents_aquisition);
        comboBox_method->setObjectName(QString::fromUtf8("comboBox_method"));

        gridLayout_6->addWidget(comboBox_method, 0, 1, 1, 2);

        lineEdit_imagePath = new QLineEdit(scrollAreaWidgetContents_aquisition);
        lineEdit_imagePath->setObjectName(QString::fromUtf8("lineEdit_imagePath"));
        lineEdit_imagePath->setEnabled(false);

        gridLayout_6->addWidget(lineEdit_imagePath, 1, 1, 1, 1);

        groupBox_lastFrame = new QGroupBox(scrollAreaWidgetContents_aquisition);
        groupBox_lastFrame->setObjectName(QString::fromUtf8("groupBox_lastFrame"));
        groupBox_lastFrame->setCheckable(true);
        groupBox_lastFrame->setChecked(false);
        horizontalLayout_10 = new QHBoxLayout(groupBox_lastFrame);
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_lastFrame = new QLabel(groupBox_lastFrame);
        label_lastFrame->setObjectName(QString::fromUtf8("label_lastFrame"));

        horizontalLayout_10->addWidget(label_lastFrame);

        lineEdit_lastFrame = new QLineEdit(groupBox_lastFrame);
        lineEdit_lastFrame->setObjectName(QString::fromUtf8("lineEdit_lastFrame"));

        horizontalLayout_10->addWidget(lineEdit_lastFrame);


        gridLayout_6->addWidget(groupBox_lastFrame, 3, 0, 1, 3);

        label_startFrame = new QLabel(scrollAreaWidgetContents_aquisition);
        label_startFrame->setObjectName(QString::fromUtf8("label_startFrame"));

        gridLayout_6->addWidget(label_startFrame, 2, 0, 1, 1);

        lineEdit_startFrame = new QLineEdit(scrollAreaWidgetContents_aquisition);
        lineEdit_startFrame->setObjectName(QString::fromUtf8("lineEdit_startFrame"));

        gridLayout_6->addWidget(lineEdit_startFrame, 2, 1, 1, 2);

        scrollArea_aquisition->setWidget(scrollAreaWidgetContents_aquisition);

        horizontalLayout_4->addWidget(scrollArea_aquisition);

        tabWidget->addTab(tab_aquisition, QString());
        tab_detector = new QWidget();
        tab_detector->setObjectName(QString::fromUtf8("tab_detector"));
        horizontalLayout_9 = new QHBoxLayout(tab_detector);
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        scrollArea_detector = new QScrollArea(tab_detector);
        scrollArea_detector->setObjectName(QString::fromUtf8("scrollArea_detector"));
        scrollArea_detector->setWidgetResizable(true);
        scrollAreaWidgetContents_detector = new QWidget();
        scrollAreaWidgetContents_detector->setObjectName(QString::fromUtf8("scrollAreaWidgetContents_detector"));
        scrollAreaWidgetContents_detector->setGeometry(QRect(0, 0, 400, 299));
        gridLayout = new QGridLayout(scrollAreaWidgetContents_detector);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        groupBox_proportionalShift = new QGroupBox(scrollAreaWidgetContents_detector);
        groupBox_proportionalShift->setObjectName(QString::fromUtf8("groupBox_proportionalShift"));
        groupBox_proportionalShift->setAutoFillBackground(false);
        groupBox_proportionalShift->setFlat(false);
        groupBox_proportionalShift->setCheckable(true);
        horizontalLayout = new QHBoxLayout(groupBox_proportionalShift);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_proportionalShift = new QLabel(groupBox_proportionalShift);
        label_proportionalShift->setObjectName(QString::fromUtf8("label_proportionalShift"));

        horizontalLayout->addWidget(label_proportionalShift);

        lineEdit_proportionalShift = new QLineEdit(groupBox_proportionalShift);
        lineEdit_proportionalShift->setObjectName(QString::fromUtf8("lineEdit_proportionalShift"));
        lineEdit_proportionalShift->setInputMethodHints(Qt::ImhNone);

        horizontalLayout->addWidget(lineEdit_proportionalShift);


        gridLayout->addWidget(groupBox_proportionalShift, 0, 0, 1, 2);

        label_minScale = new QLabel(scrollAreaWidgetContents_detector);
        label_minScale->setObjectName(QString::fromUtf8("label_minScale"));

        gridLayout->addWidget(label_minScale, 1, 0, 1, 1);

        lineEdit_minScale = new QLineEdit(scrollAreaWidgetContents_detector);
        lineEdit_minScale->setObjectName(QString::fromUtf8("lineEdit_minScale"));

        gridLayout->addWidget(lineEdit_minScale, 1, 1, 1, 1);

        label_maxScale = new QLabel(scrollAreaWidgetContents_detector);
        label_maxScale->setObjectName(QString::fromUtf8("label_maxScale"));

        gridLayout->addWidget(label_maxScale, 2, 0, 1, 1);

        lineEdit_maxScale = new QLineEdit(scrollAreaWidgetContents_detector);
        lineEdit_maxScale->setObjectName(QString::fromUtf8("lineEdit_maxScale"));

        gridLayout->addWidget(lineEdit_maxScale, 2, 1, 1, 1);

        label_numberOfFeatures = new QLabel(scrollAreaWidgetContents_detector);
        label_numberOfFeatures->setObjectName(QString::fromUtf8("label_numberOfFeatures"));

        gridLayout->addWidget(label_numberOfFeatures, 3, 0, 1, 1);

        lineEdit_numberOfFeatures = new QLineEdit(scrollAreaWidgetContents_detector);
        lineEdit_numberOfFeatures->setObjectName(QString::fromUtf8("lineEdit_numberOfFeatures"));

        gridLayout->addWidget(lineEdit_numberOfFeatures, 3, 1, 1, 1);

        label_numberOfTrees = new QLabel(scrollAreaWidgetContents_detector);
        label_numberOfTrees->setObjectName(QString::fromUtf8("label_numberOfTrees"));

        gridLayout->addWidget(label_numberOfTrees, 4, 0, 1, 1);

        lineEdit_numberOfTrees = new QLineEdit(scrollAreaWidgetContents_detector);
        lineEdit_numberOfTrees->setObjectName(QString::fromUtf8("lineEdit_numberOfTrees"));

        gridLayout->addWidget(lineEdit_numberOfTrees, 4, 1, 1, 1);

        label_minSize = new QLabel(scrollAreaWidgetContents_detector);
        label_minSize->setObjectName(QString::fromUtf8("label_minSize"));

        gridLayout->addWidget(label_minSize, 5, 0, 1, 1);

        lineEdit_minSize = new QLineEdit(scrollAreaWidgetContents_detector);
        lineEdit_minSize->setObjectName(QString::fromUtf8("lineEdit_minSize"));

        gridLayout->addWidget(lineEdit_minSize, 5, 1, 1, 1);

        scrollArea_detector->setWidget(scrollAreaWidgetContents_detector);

        horizontalLayout_9->addWidget(scrollArea_detector);

        tabWidget->addTab(tab_detector, QString());

        verticalLayout->addWidget(tabWidget);

        buttonBox = new QDialogButtonBox(QOpenTLD);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        verticalLayout->addWidget(buttonBox);

        QWidget::setTabOrder(pushButton_modelPath, lineEdit_proportionalShift);
        QWidget::setTabOrder(lineEdit_proportionalShift, buttonBox);
        QWidget::setTabOrder(buttonBox, lineEdit_modelPath);

        retranslateUi(QOpenTLD);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(QOpenTLD);
    } // setupUi

    void retranslateUi(QDialog *QOpenTLD)
    {
        QOpenTLD->setWindowTitle(QApplication::translate("ConfigDialog", "QOpenTLD", 0, QApplication::UnicodeUTF8));
        groupBox_loadModel->setTitle(QApplication::translate("ConfigDialog", "Load Model", 0, QApplication::UnicodeUTF8));
        label_modelPath->setText(QApplication::translate("ConfigDialog", "Model", 0, QApplication::UnicodeUTF8));
        pushButton_modelPath->setText(QApplication::translate("ConfigDialog", "Browse...", 0, QApplication::UnicodeUTF8));
        groupBox_initialBoundingBox->setTitle(QApplication::translate("ConfigDialog", "Initial Bounding Box", 0, QApplication::UnicodeUTF8));
        label_initialBoundingBoxX->setText(QApplication::translate("ConfigDialog", "x", 0, QApplication::UnicodeUTF8));
        label_initialBoundingBoxY->setText(QApplication::translate("ConfigDialog", "y", 0, QApplication::UnicodeUTF8));
        label_initialBoundingBoxW->setText(QApplication::translate("ConfigDialog", "w", 0, QApplication::UnicodeUTF8));
        label_initialBoundingBoxH->setText(QApplication::translate("ConfigDialog", "h", 0, QApplication::UnicodeUTF8));
        lineEdit_initialBoundingBoxX->setText(QApplication::translate("ConfigDialog", "100", 0, QApplication::UnicodeUTF8));
        lineEdit_initialBoundingBoxY->setText(QApplication::translate("ConfigDialog", "100", 0, QApplication::UnicodeUTF8));
        lineEdit_initialBoundingBoxW->setText(QApplication::translate("ConfigDialog", "100", 0, QApplication::UnicodeUTF8));
        lineEdit_initialBoundingBoxH->setText(QApplication::translate("ConfigDialog", "100", 0, QApplication::UnicodeUTF8));
        label_theta->setText(QApplication::translate("ConfigDialog", "Theta", 0, QApplication::UnicodeUTF8));
        lineEdit_theta->setText(QApplication::translate("ConfigDialog", "0.8", 0, QApplication::UnicodeUTF8));
        groupBox_trajectory->setTitle(QApplication::translate("ConfigDialog", "Trajectory", 0, QApplication::UnicodeUTF8));
        label_trajectory->setText(QApplication::translate("ConfigDialog", "Trajectory Frames", 0, QApplication::UnicodeUTF8));
        lineEdit_trajectory->setText(QApplication::translate("ConfigDialog", "20", 0, QApplication::UnicodeUTF8));
        checkBox_learning->setText(QApplication::translate("ConfigDialog", "Learning", 0, QApplication::UnicodeUTF8));
        checkBox_selectManually->setText(QApplication::translate("ConfigDialog", "Select Manually", 0, QApplication::UnicodeUTF8));
        checkBox_showOutput->setText(QApplication::translate("ConfigDialog", "Show Output", 0, QApplication::UnicodeUTF8));
        checkBox_showNotConfident->setText(QApplication::translate("ConfigDialog", "Show Not Confident", 0, QApplication::UnicodeUTF8));
        checkBox_showColorImage->setText(QApplication::translate("ConfigDialog", "Show Color Image", 0, QApplication::UnicodeUTF8));
        checkBox_showDetections->setText(QApplication::translate("ConfigDialog", "Show Detections", 0, QApplication::UnicodeUTF8));
        checkBox_showForeground->setText(QApplication::translate("ConfigDialog", "Show Foreground", 0, QApplication::UnicodeUTF8));
        groupBox_saveOutput->setTitle(QApplication::translate("ConfigDialog", "Save Output", 0, QApplication::UnicodeUTF8));
        label_outputDir->setText(QApplication::translate("ConfigDialog", "Directory", 0, QApplication::UnicodeUTF8));
        pushButton_outputDir->setText(QApplication::translate("ConfigDialog", "Browse...", 0, QApplication::UnicodeUTF8));
        groupBox_printResults->setTitle(QApplication::translate("ConfigDialog", "Print Results", 0, QApplication::UnicodeUTF8));
        label_printResults->setText(QApplication::translate("ConfigDialog", "File", 0, QApplication::UnicodeUTF8));
        pushButton_printResults->setText(QApplication::translate("ConfigDialog", "Browse...", 0, QApplication::UnicodeUTF8));
        groupBox_printTiming->setTitle(QApplication::translate("ConfigDialog", "Print Timing", 0, QApplication::UnicodeUTF8));
        label_printTiming->setText(QApplication::translate("ConfigDialog", "File", 0, QApplication::UnicodeUTF8));
        pushButton_printTiming->setText(QApplication::translate("ConfigDialog", "Browse...", 0, QApplication::UnicodeUTF8));
        checkBox_alternating->setText(QApplication::translate("ConfigDialog", "Alternating", 0, QApplication::UnicodeUTF8));
        checkBox_exportModel->setText(QApplication::translate("ConfigDialog", "Export Model", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_general), QApplication::translate("ConfigDialog", "General", 0, QApplication::UnicodeUTF8));
        pushButton_imagePath->setText(QApplication::translate("ConfigDialog", "Browse...", 0, QApplication::UnicodeUTF8));
        label_method->setText(QApplication::translate("ConfigDialog", "Method", 0, QApplication::UnicodeUTF8));
        label_imagePath->setText(QApplication::translate("ConfigDialog", "File", 0, QApplication::UnicodeUTF8));
        comboBox_method->clear();
        comboBox_method->insertItems(0, QStringList()
         << QApplication::translate("ConfigDialog", "Camera", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("ConfigDialog", "Images", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("ConfigDialog", "Video", 0, QApplication::UnicodeUTF8)
        );
        groupBox_lastFrame->setTitle(QApplication::translate("ConfigDialog", "Last Frame", 0, QApplication::UnicodeUTF8));
        label_lastFrame->setText(QApplication::translate("ConfigDialog", "Last Frame", 0, QApplication::UnicodeUTF8));
        lineEdit_lastFrame->setText(QApplication::translate("ConfigDialog", "0", 0, QApplication::UnicodeUTF8));
        label_startFrame->setText(QApplication::translate("ConfigDialog", "Start Frame", 0, QApplication::UnicodeUTF8));
        lineEdit_startFrame->setText(QApplication::translate("ConfigDialog", "1", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_aquisition), QApplication::translate("ConfigDialog", "Aquisition", 0, QApplication::UnicodeUTF8));
        groupBox_proportionalShift->setTitle(QApplication::translate("ConfigDialog", "Proportional Shift", 0, QApplication::UnicodeUTF8));
        label_proportionalShift->setText(QApplication::translate("ConfigDialog", "Proportional Shift", 0, QApplication::UnicodeUTF8));
        lineEdit_proportionalShift->setInputMask(QString());
        lineEdit_proportionalShift->setText(QApplication::translate("ConfigDialog", "0.1", 0, QApplication::UnicodeUTF8));
        label_minScale->setText(QApplication::translate("ConfigDialog", "Min Scale", 0, QApplication::UnicodeUTF8));
        lineEdit_minScale->setInputMask(QString());
        lineEdit_minScale->setText(QApplication::translate("ConfigDialog", "-10", 0, QApplication::UnicodeUTF8));
        label_maxScale->setText(QApplication::translate("ConfigDialog", "Max Scale", 0, QApplication::UnicodeUTF8));
        lineEdit_maxScale->setText(QApplication::translate("ConfigDialog", "10", 0, QApplication::UnicodeUTF8));
        label_numberOfFeatures->setText(QApplication::translate("ConfigDialog", "Number of Features", 0, QApplication::UnicodeUTF8));
        lineEdit_numberOfFeatures->setText(QApplication::translate("ConfigDialog", "10", 0, QApplication::UnicodeUTF8));
        label_numberOfTrees->setText(QApplication::translate("ConfigDialog", "Number of Trees", 0, QApplication::UnicodeUTF8));
        lineEdit_numberOfTrees->setText(QApplication::translate("ConfigDialog", "10", 0, QApplication::UnicodeUTF8));
        label_minSize->setText(QApplication::translate("ConfigDialog", "Min Size", 0, QApplication::UnicodeUTF8));
        lineEdit_minSize->setText(QApplication::translate("ConfigDialog", "25", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_detector), QApplication::translate("ConfigDialog", "Detector", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ConfigDialog: public Ui_ConfigDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONFIGDIALOG_H
