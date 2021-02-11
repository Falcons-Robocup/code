// Copyright 2019-2020 Martijn (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * SettingsDialog.h
 *
 *  Created on: December 11th, 2016
 *      Author: Diana Koenraadt
 */

#ifndef FIELD_SETTINGSDIALOG_H
#define FIELD_SETTINGSDIALOG_H

#include <ui_FieldWidgetSettings.h>

#include <QtGui>

namespace Visualizer
{
    namespace Settings
    {
        const QString showPathPlanningSetting = "field/showPathPlanning"; // Setting itself returns boolean value
        const QString teamColorSetting = "refbox/teamColor";
        const QString playingFieldSetting = "refbox/playingField";
        const QString ttaConfigSetting = "refbox/ttaConfig";

        class SettingsDialog : public QDialog, public Ui::FieldWidgetSettings
        {
            public:
                SettingsDialog();
                ~SettingsDialog() {};

            public Q_SLOTS:
                virtual void accept() override;
        };
    }
}

#endif // FIELD_SETTINGSDIALOG_H
