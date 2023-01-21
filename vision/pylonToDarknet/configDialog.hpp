// Copyright 2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CONFIGDIALOG_HPP
#define CONFIGDIALOG_HPP

#include <QDialog>
#include <QString>

#include "config.hpp"

class mainWidget; //forward declaration

namespace Ui {
class configDialog;
}

class configDialog : public QDialog {
   Q_OBJECT

public:
   explicit configDialog(mainWidget *parent = nullptr);
   ~configDialog();

   void setup();
   void update();

   float xOffset, yOffset; // range 0.0 to 1.0
   float rZ; // range 0.0 to 2 Pi

private slots:
   void on_balanceRatioBlueSlider_valueChanged(int value);
   void on_balanceRatioGreenSlider_valueChanged(int value);
   void on_balanceRatioRedSlider_valueChanged(int value);
   void on_blackLevelSlider_valueChanged(int value);
   void on_brightnessSlider_valueChanged(int value);
   void on_contrastSlider_valueChanged(int value);
   void on_exposureModeSlider_valueChanged(int value);
   void on_exposureTimeSlider_valueChanged(int value);
   void on_gainSlider_valueChanged(int value);
   void on_gammaModeSlider_valueChanged(int value);
   void on_frameRateSlider_valueChanged(int value);
   void on_gammaSlider_valueChanged(int value);
   void on_hueSlider_valueChanged(int value);
   void on_lightSourcePresetSlider_valueChanged(int value);
   void on_pushButton_clicked();
   void on_saturationSlider_valueChanged(int value);
   void on_whiteBalanceModeSlider_valueChanged(int value);

   void on_floorHueMin_valueChanged(int value);
   void on_floorHueMax_valueChanged(int value);
   void on_floorSatMin_valueChanged(int value);
   void on_floorSatMax_valueChanged(int value);
   void on_floorValMin_valueChanged(int value);
   void on_floorValMax_valueChanged(int value);
   void on_lineHueMin_valueChanged(int value);
   void on_lineHueMax_valueChanged(int value);
   void on_lineSatMin_valueChanged(int value);
   void on_lineSatMax_valueChanged(int value);
   void on_lineValMin_valueChanged(int value);
   void on_lineValMax_valueChanged(int value);
   void on_floorMinWidth_valueChanged(int value);
   void on_lineMinWidth_valueChanged(int value);
   void on_floorDec_valueChanged(int value);
   void on_xOffset_valueChanged(int value);
   void on_yOffset_valueChanged(int value);
   void on_rotate_valueChanged(int value);

private:
   Ui::configDialog *ui = nullptr;
   mainWidget *parent = nullptr;
   config *conf = nullptr;

   QString lightSourcePresetValueToString(int value);
   int lightSourcePresetStringToValue(QString string);
};

#endif // CONFIGDIALOG_HPP
