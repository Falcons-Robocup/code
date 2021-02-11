// Copyright 2019-2020 Martijn (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * SettingsDialog.cpp
 *
 *  Created on: December 14th, 2016
 *      Author: Diana Koenraadt
 */

#include "int/widgets/SettingsDialog.h"

using namespace Visualizer::Settings;

SettingsDialog::SettingsDialog()
{
    setupUi(this);

    // Read settings file to set control states.
    QSettings settings;
    showPathPlanning->setChecked(settings.value(showPathPlanningSetting, false).toBool());
}

void SettingsDialog::accept()
{
    // Save control settings to QSettings
    QSettings settings;

    settings.setValue(showPathPlanningSetting, showPathPlanning->isChecked());
    settings.setValue(teamColorSetting, teamColorComboBox->currentText());
    settings.setValue(playingFieldSetting, playingFieldComboBox->currentText());
    settings.setValue(ttaConfigSetting, ttaConfigComboBox->currentText());

    // Persist settings
    settings.sync();

    QDialog::accept();
}
