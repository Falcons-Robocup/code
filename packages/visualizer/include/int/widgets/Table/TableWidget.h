// Copyright 2017-2018 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * TableWidget.h
 *
 *  Created on: January 18, 2017
 *      Author: Diana Koenraadt
 */

#ifndef TABLEWIDGET_H
#define TABLEWIDGET_H

#include <QtGui>
#include <QStandardItemModel>
#include <QStandardItem>
#include <qtablewidget.h>
#include <qtreeview.h>
#include <qwidget.h>

// Internal:
#include "int/widgets/Widget.h"
#include "int/widgets/Table/TableViewModel.h"

class GameSignalAdapter;
class TableWidgetGameSignalSubscriber;

class TableWidget : public QTreeView, public Widget<TableWidgetGameSignalSubscriber, TableWidget>
{
        Q_OBJECT
friend TableWidgetGameSignalSubscriber;

public:
    TableWidget(QWidget* parent = 0);
    void initialize(const int nrColumns);

    virtual void saveState() override;
    virtual void restoreState() override;

private:
    TableViewModel* _model = NULL;
};

/*
* Class that handles subscriber data for the TableWidget
* Use delegation to avoid diamond of death on QObject
*/
class TableWidgetGameSignalSubscriber : public QObject, public WidgetGameSignalSubscriber<TableWidget>
{
        Q_OBJECT
public:
    using WidgetGameSignalSubscriber::WidgetGameSignalSubscriber;
    virtual void subscribe(GameSignalAdapter* gameSignalAdapter) override;

private:

    template <typename TType>
    void onValue(uint8_t senderRobotId, std::string category, std::string key, TType value)
    {
        _widget->_model->setData<TType>(category, key, (int)senderRobotId - 1, value);
    }

public Q_SLOTS:
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, float value) override;
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, bool value) override;
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, std::string value) override;
    virtual void clearColumn(uint8_t robotId);
};

#endif // TABLEWIDGET_H
