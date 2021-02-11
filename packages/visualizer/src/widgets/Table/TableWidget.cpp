// Copyright 2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * TableWidget.cpp
 *
 *  Created on: January 18, 2017
 *      Author: Diana Koenraadt
 */

#include <qheaderview.h>

// Internal:
#include "int/widgets/Table/TableWidget.h"

TableWidget::TableWidget(QWidget *parent)
    : Widget()
{
};

void TableWidget::initialize(const int nrColumns)
{
    if (_model != NULL)
    {
        delete _model;
    }
    _model = new TableViewModel(nrColumns);
    setModel(_model);
}

void TableWidget::saveState()
{
    QSettings settings;

    // Save table header state
    settings.setValue("tableWidget", 1); // QSettings versioning, increase if you rename tableview keys.
    settings.setValue("tableWidget/headerState", header()->saveState());

    settings.sync();
}

void TableWidget::restoreState()
{
    QSettings settings;
    if (settings.contains("tableWidget") && settings.value("tableWidget").toInt() == 1)
    {
        header()->restoreState(settings.value("tableWidget/headerState").toByteArray());
    }
}
