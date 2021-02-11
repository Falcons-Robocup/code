// Copyright 2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * TableViewModel.h
 *
 *  Created on: June 30, 2016
 *      Author: Diana Koenraadt
 */

#ifndef TABLEVIEWMODEL_H
#define TABLEVIEWMODEL_H

#include <QtGui>
#include <QStandardItemModel>
#include <QStandardItem>

/*
* Table ViewModel class
*/
class TableViewModel : public QStandardItemModel
{
        Q_OBJECT

public:
    TableViewModel(const int nrColumns);

    template <typename TType>
    void setData(std::string category, std::string key, int column, TType value)
    {
        setData(category, key, column, QVariant(value));
    }

    void setData(std::string category, std::string key, int column, QVariant value);
    void add(std::string category, std::string key); // Add for display
    void clearColumn(int column);

private:
    int _nrValuesPerKey;
    QList<QStandardItem *> _tableData;

    bool displayValue(std::string category, std::string key); // Whether or not to display values for the given category/key pair.
    std::map<std::tuple<std::string /* category */, std::string /* key */ >, bool> _toDisplay;
};

#endif // TABLEVIEWMODEL_H
