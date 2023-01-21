// Copyright 2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// TODO: update sliders when reset button is pressed

#include <cfloat> // for float defines

#include "configDialog.hpp"
#include "mainWidget.hpp"
#include "ui_configDialog.h"

configDialog::configDialog(mainWidget *parent) :
   QDialog(parent),
   ui(new Ui::configDialog) {
   ui->setupUi(this);
   this->parent = parent;
   this->conf = parent->conf;

   xOffset = -1.0; // set to invalid, has to be configured from gui
   yOffset = -1.0; // set to invalid, has to be configured from gui
   rZ = -1.0; // set to invalid, has to be configured from gui
}

configDialog::~configDialog() {
   delete ui;
}

void configDialog::setup() {
   setWindowTitle("config");

   // set the slider to the default value (and indirectly also the value field)
   ui->gainSlider->setValue(conf->gain);
   ui->exposureTimeSlider->setValue(conf-> exposureTime/ 1000);
   ui->exposureModeValue->setText(conf->testPattern);
   ui->blackLevelSlider->setValue(conf->blackLevel);
   ui->gammaSlider->setValue(conf->gamma * 50);

   ui->gammaModeValue->setText(conf->gammaMode); // update anyway, because not sure if slider value changes
   if( conf->gammaMode == "Off" ) {
      ui->gammaModeSlider->setValue(0);
   } else {
      ui->gammaModeSlider->setValue(1);
   }
   ui->frameRateSlider->setValue(conf->frameRate);
   ui->lightSourcePresetSlider->setValue(lightSourcePresetStringToValue(conf->lightSourcePreset));
   ui->hueSlider->setValue(conf->hue);
   ui->saturationSlider->setValue(conf->saturation*100);
   ui->contrastSlider->setValue(conf->contrast*100);
   ui->brightnessSlider->setValue(conf->brightness*100);
   //    ui->balanceRatioRedSlider->setValue((conf->balanceRatioRed - 0.25) * 10);
   //    ui->balanceRatioGreenSlider->setValue((conf->balanceRatioGreen - 0.25) * 10);
   //    ui->balanceRatioBlueSlider->setValue((conf->balanceRatioBlue - 0.25) * 10);
   ui->whiteBalanceModeValue->setText(conf->whiteBalanceMode);

   if ( parent->robot == 9 ) {
      ui->floorHueMin->setValue(101); // range [0:359]
      ui->floorHueMax->setValue(166);
      ui->floorSatMin->setValue(91); // range [0:255]
      ui->floorSatMax->setValue(255); // not used in multiCam
      ui->floorValMin->setValue(25); // range [0:255]
      ui->floorValMax->setValue(255); // not used in multiCam

      ui->lineHueMin->setValue(0);
      ui->lineHueMax->setValue(359);
      ui->lineSatMin->setValue(0);
      ui->lineSatMax->setValue(110);
      ui->lineValMin->setValue(85);
      ui->lineValMax->setValue(255);
   } else {
      ui->floorHueMin->setValue(70); // range [0:359]
      ui->floorHueMax->setValue(170);
      ui->floorSatMin->setValue(75); // range [0:255]
      ui->floorSatMax->setValue(255); // not used in multiCam
      ui->floorValMin->setValue(50); // range [0:255]
      ui->floorValMax->setValue(255); // not used in multiCam

      ui->lineHueMin->setValue(0);
      ui->lineHueMax->setValue(359);
      ui->lineSatMin->setValue(0);
      ui->lineSatMax->setValue(75);
      ui->lineValMin->setValue(160);
      ui->lineValMax->setValue(255);
   }

   ui->floorMinWidth->setValue(5); // range [1:20]
   ui->floorDec->setValue(1);
   ui->lineMinWidth->setValue(1);
   // ui->lineDec->setValue(2); // TODO: add again to GUI
   parent->lPoints->lineDec = 2; // TODO: remove when GUI slider is added again

   ui->xOffset->setValue(500); // range 0 to 1000, center of the floor
   ui->yOffset->setValue(500); // range 0 to 1000, center of the floor
   ui->rotate->setValue(0); // range -180 to 180 degrees
}

void configDialog::update() {
   // TODO: dependency problem balance ratio (which can be set from slider or auto whitebalance)
   //    ui->balanceRatioRedSlider->setValue(conf->balanceRatioRedActual);
   //    ui->balanceRatioGreenSlider->setValue(conf->balanceRatioGreenActual);
   //    ui->balanceRatioBlueSlider->setValue(conf->balanceRatioBlueActual);
   ui->balanceRatioRedValue->setText(QString().asprintf("%5.3f ", conf->balanceRatioRedActual));
   ui->balanceRatioGreenValue->setText(QString().asprintf("%5.3f ", conf->balanceRatioGreenActual));
   ui->balanceRatioBlueValue->setText(QString().asprintf("%5.3f ", conf->balanceRatioBlueActual));
}

// converstion method
QString configDialog::lightSourcePresetValueToString(int value) {
   QString string;
   if (value == 1 ) {
      string = "Daylight5000K";
   } else if (value == 2 ) {
      string = "Daylight6500K";
   } else if (value == 3 ) {
      string = "Tungsten";
   } else if (value == 4 ) {
      string = "Led";
   } else {
      string = "Off";
   }
   return string;
}

// converstion method
int configDialog::lightSourcePresetStringToValue(QString string) {
   int value;
   if (string == "Daylight5000K" ) {
      value = 1;
   } else if (string == "Daylight6500K" ) {
      value = 2;
   } else if ( string == "Tungsten" ) {
      value = 3;
   } else if ( string == "Led" ) {
      value = 4;
   } else {
      value = 0;
   }
   return value;
}

void configDialog::on_gainSlider_valueChanged(int value) {
   if( value >= 48 ) {
      conf->gain = 48 - 48.0 * FLT_EPSILON;
   } else {
      conf->gain = (double)value;
   }

   ui->gainValue->setText(QString().asprintf("%4.2f ", conf->gain));
}

void configDialog::on_exposureTimeSlider_valueChanged(int value) {
   if( value < 1 ) {
      conf->exposureTime = 500; // the lowest value 19.0 results in a lot of noise
   } else {
      conf->exposureTime = 1000.0 * value;
   }
   ui->exposureTimeValue->setText(QString().asprintf("%4.1f ms", conf->exposureTime/1000.0));
}

void configDialog::on_exposureModeSlider_valueChanged(int value) {
   if( value == 0 ) {
      conf->testPattern = "Off";
   } else if (value == 1 ) {
      conf->testPattern = "Black";
   } else if (value == 2 ) {
      conf->testPattern = "White";
   } else if (value == 3 ) {
      conf->testPattern = "Testimage1";
   } else if (value == 4 ) {
      conf->testPattern = "Testimage2";
   } else if (value == 5 ) {
      conf->testPattern = "Testimage3";
   } else if (value == 6 ) {
      conf->testPattern = "Testimage6";
   } else {
      conf->testPattern = "Off";
   }
   ui->exposureModeValue->setText(conf->testPattern);
}

void configDialog::on_blackLevelSlider_valueChanged(int value) {
   conf->blackLevel = value;
   ui->blackLevelValue->setText(QString().asprintf("%4.2f ", conf->blackLevel));
}

void configDialog::on_gammaSlider_valueChanged(int value) {
   // gamma range 0 to 3.9998 but interesting range is from 0 to 2.0
   // slider range is set from 0 to 100 and default set to 50
   conf->gamma = value/50.0;
   ui->gammaValue->setText(QString().asprintf("%4.2f ", conf->gamma));
}

void configDialog::on_gammaModeSlider_valueChanged(int value) {
   if( value == 0 ) {
      conf->gammaMode = "Off";
   } else {
      conf->gammaMode = "sRgb";
   }
   ui->gammaModeValue->setText(conf->gammaMode);
}

void configDialog::on_frameRateSlider_valueChanged(int value) {
   conf->frameRate = value;
   ui->frameRateValue->setText(QString::number(conf->frameRate));
}

void configDialog::on_lightSourcePresetSlider_valueChanged(int value) {
   QString string = lightSourcePresetValueToString(value);
   conf->lightSourcePreset = string;
   ui->lightSourcePresetValue->setText(string);
}

void configDialog::on_hueSlider_valueChanged(int value) {
   double tmp = (double)value;
   if( tmp < -180.0 ) { tmp = -180.0; }
   if( tmp > 180.0 ) { tmp = 180.0; }
   conf->hue = tmp;
   ui->hueValue->setText(QString().asprintf("%4.1f ", conf->hue));
}

void configDialog::on_saturationSlider_valueChanged(int value) {
   double tmp = value / 100.0;
   if( tmp < -1.0 ) { tmp = -1.0; }
   if( tmp > 1.0 ) { tmp = 1.0; }
   conf->saturation = tmp;
   ui->saturationValue->setText(QString().asprintf("%4.2f ", conf->saturation));
}

void configDialog::on_contrastSlider_valueChanged(int value) {
   double tmp = value / 100.0;
   if( tmp < -1.0 ) { tmp = -1.0; }
   if( tmp > 0.998 ) { tmp = 0.998; } // 0.999969
   conf->contrast = tmp;
   ui->contrastValue->setText(QString().asprintf("%4.2f ", conf->contrast));
}

void configDialog::on_brightnessSlider_valueChanged(int value) {
   double tmp = value / 100.0;
   if( tmp < -1.0 ) { tmp = -1.0; }
   if( tmp > 0.998 ) { tmp = 0.998; } // 0.999969
   conf->brightness = tmp;
   ui->brightnessValue->setText(QString().asprintf("%4.2f ", conf->brightness));
}

void configDialog::on_balanceRatioRedSlider_valueChanged(int value) {
   double tmp = 0.25 + value / 10.0;
   if( tmp < 0.25 ) { tmp = 0.25; }
   if( tmp > 15.999576 ) { tmp = 15.999576; }
   conf->balanceRatioRed = tmp;
   ui->balanceRatioRedValue->setText(QString::number(conf->balanceRatioRed));
}

void configDialog::on_balanceRatioGreenSlider_valueChanged(int value) {
   double tmp = 0.25 + value / 10.0;
   if( tmp < 0.25 ) { tmp = 0.25; }
   if( tmp > 15.999576 ) { tmp = 15.999576; }
   conf->balanceRatioGreen = tmp;
   ui->balanceRatioGreenValue->setText(QString::number(conf->balanceRatioGreen));
}

void configDialog::on_balanceRatioBlueSlider_valueChanged(int value) {
   double tmp = 0.25 + value / 10.0;
   if( tmp < 0.25 ) { tmp = 0.25; }
   if( tmp > 15.999576 ) { tmp = 15.999576; }
   conf->balanceRatioBlue = tmp;
   ui->balanceRatioBlueValue->setText(QString::number(conf->balanceRatioBlue));
}

void configDialog::on_whiteBalanceModeSlider_valueChanged(int value) {
   if( value == 0 ) {
      conf->whiteBalanceMode = "Off";
   } else if (value == 1 ) {
      conf->whiteBalanceMode = "Once";
   } else if (value == 2 ) {
      conf->whiteBalanceMode = "Continuous";
   } else {
      conf->whiteBalanceMode = "Off";
   }
   ui->whiteBalanceModeValue->setText(conf->whiteBalanceMode);
}

void configDialog::on_pushButton_clicked() {
   close();
}

void configDialog::on_floorHueMin_valueChanged(int value) {
   parent->lPoints->hueMin[0] = value;
   ui->floorHueMinShow->setText(QString().asprintf("%3d", parent->lPoints->hueMin[0]));
}

void configDialog::on_floorHueMax_valueChanged(int value) {
   parent->lPoints->hueMax[0] = value;
   ui->floorHueMaxShow->setText(QString().asprintf("%3d", parent->lPoints->hueMax[0]));
}

void configDialog::on_floorSatMin_valueChanged(int value) {
   parent->lPoints->satMin[0] = value;
   ui->floorSatMinShow->setText(QString().asprintf("%3d", parent->lPoints->satMin[0]));
}

void configDialog::on_floorSatMax_valueChanged(int value) {
   parent->lPoints->satMax[0] = value;
   ui->floorSatMaxShow->setText(QString().asprintf("%3d", parent->lPoints->satMax[0]));
}

void configDialog::on_floorValMin_valueChanged(int value) {
   parent->lPoints->valMin[0] = value;
   ui->floorValMinShow->setText(QString().asprintf("%3d", parent->lPoints->valMin[0]));
}

void configDialog::on_floorValMax_valueChanged(int value) {
   parent->lPoints->valMax[0] = value;
   ui->floorValMaxShow->setText(QString().asprintf("%3d", parent->lPoints->valMax[0]));
}


void configDialog::on_lineHueMin_valueChanged(int value) {
   parent->lPoints->hueMin[1] = value;
   ui->lineHueMinShow->setText(QString().asprintf("%3d", parent->lPoints->hueMin[1]));
}

void configDialog::on_lineHueMax_valueChanged(int value) {
   parent->lPoints->hueMax[1] = value;
   ui->lineHueMaxShow->setText(QString().asprintf("%3d", parent->lPoints->hueMax[1]));
}

void configDialog::on_lineSatMin_valueChanged(int value) {
   parent->lPoints->satMin[1] = value;
   ui->lineSatMinShow->setText(QString().asprintf("%3d", parent->lPoints->satMin[1]));
}

void configDialog::on_lineSatMax_valueChanged(int value) {
   parent->lPoints->satMax[1] = value;
   ui->lineSatMaxShow->setText(QString().asprintf("%3d", parent->lPoints->satMax[1]));
}

void configDialog::on_lineValMin_valueChanged(int value) {
   parent->lPoints->valMin[1] = value;
   ui->lineValMinShow->setText(QString().asprintf("%3d", parent->lPoints->valMin[1]));
}

void configDialog::on_lineValMax_valueChanged(int value) {
   parent->lPoints->valMax[1] = value;
   ui->lineValMaxShow->setText(QString().asprintf("%3d", parent->lPoints->valMax[1]));
}

void configDialog::on_floorMinWidth_valueChanged(int value) {
   parent->lPoints->floorMinWidth = value;
   ui->floorMinWidthShow->setText(QString().asprintf("%3d", parent->lPoints->floorMinWidth));
}

void configDialog::on_lineMinWidth_valueChanged(int value) {
   parent->lPoints->lineMinWidth = value;
   ui->lineMinWidthShow->setText(QString().asprintf("%3d", parent->lPoints->lineMinWidth));
}

void configDialog::on_floorDec_valueChanged(int value) {
   parent->lPoints->floorDec = value;
   ui->floorDecShow->setText(QString().asprintf("%3d", parent->lPoints->floorDec));
}

//void configDialog::on_lineDec_valueChanged(int value) {
//    lineDec = value;
//    parent->ui->lineDecShow->setText(QString().asprintf("%3d", lineDec));
//}

void configDialog::on_xOffset_valueChanged(int value) {
   // value range 0 to 1000
   ui->xOffsetShow->setText(QString().asprintf("%3d", value));

   xOffset = value/1000.0; // remap range to (0.0 to 1.0)
   if( xOffset > 1.0 ) {
      qDebug() << "ERROR   xOffset" << xOffset << "out of range";
      xOffset = 1.0;
   } else if( xOffset < 0.0 ) {
      qDebug() << "ERROR   xOffset" << xOffset << "out of range";
      xOffset = 0.0;
   }
}

void configDialog::on_yOffset_valueChanged(int value) {
   // value range 0 to 1000
   ui->yOffsetShow->setText(QString().asprintf("%3d", value));

   yOffset = 1.0 - value/1000.0; // remap range to (0.0 to 1.0)

   if( yOffset > 1.0 ) {
      qDebug() << "ERROR   yOffset" << yOffset << "out of range";
      yOffset = 1.0;
   } else if( yOffset < 0.0 ) {
      qDebug() << "ERROR   yOffset" << yOffset << "out of range";
      yOffset = 0.0;
   }
}

void configDialog::on_rotate_valueChanged(int value) {
   // value range -180 to 180 degrees
   rZ = 2.0 * M_PI * value / 360.0;
   ui->rotateShow->setText(QString().asprintf("%4.0f", 360.0 * rZ / ( 2.0 * M_PI ) ));
}
